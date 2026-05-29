#!/usr/bin/env python3
"""Tkinter control panel for Odin driver and FAST-LIVO2."""

import os
import queue
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import ttk
import yaml


def apply_default_ros_environment(env: Optional[Dict[str, str]] = None) -> Dict[str, str]:
    target = os.environ if env is None else env
    target.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    target.setdefault("ROS_DOMAIN_ID", "33")
    target["ROS_LOCALHOST_ONLY"] = "1"
    target["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
    return target


def default_workspace() -> Path:
    env_path = os.environ.get("ODIN_LIVO_WORKSPACE")
    if env_path:
        return Path(env_path).expanduser()
    candidates = [
        Path.cwd(),
        Path.home() / "livo_workspace",
        Path("/home/alienware/livo_workspace"),
        Path("/home/nuc13/livo_workspace"),
    ]
    for candidate in candidates:
        if (candidate / "src" / "FAST-LIVO2").exists() and (candidate / "src" / "odin_ros_driver").exists():
            return candidate
    return Path.cwd()


WORKSPACE = default_workspace()
LOG_ROOT = Path("/tmp/odin_livo_control")
ROS_LOG_DIR = Path("/tmp/ros-log")
apply_default_ros_environment()

ODIN_WARMUP_S = 10.0
STOP_GAP_S = 2.0
ODIN_GRACEFUL_STOP_TIMEOUT_S = 60.0
GRACEFUL_STOP_TIMEOUT_S = 30.0
TERM_TIMEOUT_S = 5.0
LEGACY_DDS_MODE = os.environ.get("ODIN_LIVO_LEGACY_DDS", "0") == "1"


@dataclass
class ManagedProcess:
    name: str
    command: List[str]
    process: subprocess.Popen
    log_path: Path


@dataclass
class RunOptions:
    rviz: bool
    fast_livo_pcd: bool
    fast_livo_image: bool
    odin_recorddata: bool
    save_translation_m: float
    save_rotation_deg: float


class OdinLivoController:
    def __init__(self, ui_queue: "queue.Queue[Tuple[str, object]]"):
        self._ui_queue = ui_queue
        self._lock = threading.Lock()
        self._odin: Optional[ManagedProcess] = None
        self._livo: Optional[ManagedProcess] = None
        self._run_dir: Optional[Path] = None
        self._starting = False
        self._stopping = False
        self._stop_requested = threading.Event()

    def is_busy(self) -> bool:
        with self._lock:
            return self._starting or self._stopping

    def has_processes(self) -> bool:
        with self._lock:
            processes = [self._odin, self._livo]
        return any(managed is not None and managed.process.poll() is None for managed in processes)

    def start(self, options: RunOptions):
        if self.is_busy() or self.has_processes():
            self._append_log("System", "[control] Start ignored; a launch is already active")
            return
        self._stop_requested.clear()
        thread = threading.Thread(target=self._start_sequence, args=(options,), name="start-sequence", daemon=True)
        thread.start()

    def stop(self):
        self._stop_requested.set()
        with self._lock:
            already_stopping = self._stopping
        if already_stopping:
            return
        odin, livo = self._begin_stop()
        if odin is None and livo is None:
            self._set_state("Odin", "Stopped", "No active process")
            self._set_state("FAST-LIVO2", "Stopped", "No active process")
            return
        thread = threading.Thread(target=self._stop_sequence, args=(odin, livo), name="stop-sequence", daemon=True)
        thread.start()

    def stop_blocking(self):
        self._stop_requested.set()
        with self._lock:
            already_stopping = self._stopping
        if already_stopping:
            return
        odin, livo = self._begin_stop()
        if odin is not None or livo is not None:
            self._stop_sequence(odin, livo)

    def _begin_stop(self) -> Tuple[Optional[ManagedProcess], Optional[ManagedProcess]]:
        with self._lock:
            if self._stopping:
                return None, None
            self._stopping = True
            odin = self._odin
            livo = self._livo
        if odin is None and livo is None:
            with self._lock:
                self._stopping = False
        return odin, livo

    def poll_processes(self):
        with self._lock:
            processes = [self._odin, self._livo]
        for managed in processes:
            if managed is None:
                continue
            rc = managed.process.poll()
            if rc is not None:
                self._ui_queue.put(("process_exit", (managed.name, rc)))
                with self._lock:
                    if managed.name == "Odin":
                        self._odin = None
                    elif managed.name == "FAST-LIVO2":
                        self._livo = None

    def _set_state(self, service: str, state: str, detail: str = ""):
        self._ui_queue.put(("state", (service, state, detail)))

    def _append_log(self, service: str, line: str):
        self._ui_queue.put(("log", (service, line)))

    def _start_sequence(self, options: RunOptions):
        with self._lock:
            self._starting = True
        try:
            if not WORKSPACE.exists():
                self._set_state("System", "Failed", f"Workspace not found: {WORKSPACE}")
                return
            self._run_dir = LOG_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
            self._run_dir.mkdir(parents=True, exist_ok=True)
            ROS_LOG_DIR.mkdir(parents=True, exist_ok=True)
            fast_livo_output_dir = WORKSPACE / "src" / "FAST-LIVO2" / "Log" / self._run_dir.name

            if not LEGACY_DDS_MODE:
                self._set_state("Odin", "Integrated", "Direct SDK input inside FAST-LIVO2")
                self._set_state("FAST-LIVO2", "Starting", "Launching direct SDK mapping")
                self._append_log("FAST-LIVO2", f"[control] Output directory: {fast_livo_output_dir}")
                odin_config = WORKSPACE / "src" / "odin_ros_driver" / "config" / "control_command_fast_livo.yaml"
                recorddata_dir = WORKSPACE / "src" / "odin_ros_driver" / "recorddata"
                livo = self._launch(
                    "FAST-LIVO2",
                    [
                        "ros2",
                        "launch",
                        "fast_livo",
                        "mapping_odin_direct.launch.py",
                        f"rviz:={'true' if options.rviz else 'false'}",
                        f"pcd_save:={'true' if options.fast_livo_pcd else 'false'}",
                        "final_map_save:=false",
                        f"image_save:={'true' if options.fast_livo_image else 'false'}",
                        f"output_run_dir:={fast_livo_output_dir}",
                        f"topic_report_dir:={fast_livo_output_dir / 'topic_reports'}",
                        f"save_translation_m:={options.save_translation_m:.6f}",
                        f"save_rotation_deg:={options.save_rotation_deg:.6f}",
                        f"odin_config:={odin_config}",
                        f"recorddata:={'true' if options.odin_recorddata else 'false'}",
                        f"recorddata_dir:={recorddata_dir}",
                        "publish_debug_topics:=false",
                    ],
                    self._run_dir / "fast_livo_direct.log",
                )
                with self._lock:
                    self._livo = livo
                self._set_state("FAST-LIVO2", "Running", "Direct SDK mapping launched")
                return

            self._set_state("Odin", "Starting", "Launching Odin driver")
            odin_command = ["ros2", "launch", "odin_ros_driver", "odin1_fast_livo_ros2.launch.py"]
            if options.odin_recorddata:
                odin_config = self._write_odin_runtime_config(recorddata=True)
                odin_command.append(f"config_file:={odin_config}")
                self._append_log(
                    "Odin",
                    f"[control] Odin recorddata enabled: {odin_config} "
                    "(SLAM cloud/odom streams enabled for OLX recording)",
                )
            odin = self._launch(
                "Odin",
                odin_command,
                self._run_dir / "odin.log",
            )
            with self._lock:
                self._odin = odin

            ok, detail = self._wait_for_odin_warmup()
            if not ok:
                if self._stop_requested.is_set():
                    self._append_log("Odin", f"[control] Start cancelled: {detail}")
                else:
                    self._set_state("Odin", "Failed", detail)
                    self._append_log("Odin", f"[control] Odin warmup failed: {detail}")
                return

            self._set_state("Odin", "Running", detail)
            self._set_state("FAST-LIVO2", "Starting", "Launching mapping")
            self._append_log("FAST-LIVO2", f"[control] Output directory: {fast_livo_output_dir}")
            livo = self._launch(
                "FAST-LIVO2",
                [
                    "ros2",
                    "launch",
                    "fast_livo",
                    "mapping_odin.launch.py",
                    f"rviz:={'true' if options.rviz else 'false'}",
                    f"pcd_save:={'true' if options.fast_livo_pcd else 'false'}",
                    f"image_save:={'true' if options.fast_livo_image else 'false'}",
                    f"output_run_dir:={fast_livo_output_dir}",
                    f"topic_report_dir:={fast_livo_output_dir / 'topic_reports'}",
                    f"save_translation_m:={options.save_translation_m:.6f}",
                    f"save_rotation_deg:={options.save_rotation_deg:.6f}",
                ],
                self._run_dir / "fast_livo.log",
            )
            with self._lock:
                self._livo = livo
            self._set_state("FAST-LIVO2", "Running", "FAST-LIVO2 launched")
        except Exception as exc:
            self._set_state("System", "Failed", str(exc))
            self._append_log("System", f"[control] Start failed: {exc}")
        finally:
            with self._lock:
                self._starting = False

    def _write_odin_runtime_config(self, recorddata: bool) -> Path:
        if self._run_dir is None:
            raise RuntimeError("Run directory is not initialized")
        source = WORKSPACE / "src" / "odin_ros_driver" / "config" / "control_command_fast_livo.yaml"
        with open(source, "r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle)
        register_keys = config.setdefault("register_keys", {})
        register_keys["recorddata"] = 1 if recorddata else 0
        if recorddata:
            register_keys["sendcloudslam"] = 1
            register_keys["sendodom"] = 1
        output = self._run_dir / "control_command_fast_livo_gui.yaml"
        with open(output, "w", encoding="utf-8") as handle:
            yaml.safe_dump(config, handle, default_flow_style=False, sort_keys=False)
        return output

    def _stop_sequence(self, odin: Optional[ManagedProcess], livo: Optional[ManagedProcess]):
        try:
            if odin is not None:
                self._set_state("Odin", "Stopping", "Sending SIGINT")
                self._stop_managed(odin, graceful_timeout=ODIN_GRACEFUL_STOP_TIMEOUT_S)
                with self._lock:
                    if self._odin is odin:
                        self._odin = None
                self._set_state("Odin", "Stopped", "Stopped")

            if odin is not None and livo is not None:
                time.sleep(STOP_GAP_S)

            if livo is not None:
                self._set_state("FAST-LIVO2", "Stopping", "Saving map and shutting down")
                self._stop_managed(livo, graceful_timeout=GRACEFUL_STOP_TIMEOUT_S)
                with self._lock:
                    if self._livo is livo:
                        self._livo = None
                self._set_state("FAST-LIVO2", "Stopped", "Stopped")
        finally:
            with self._lock:
                self._stopping = False

    def _launch(self, name: str, command: List[str], log_path: Path) -> ManagedProcess:
        env = apply_default_ros_environment(os.environ.copy())
        log_file = open(log_path, "a", encoding="utf-8", buffering=1)
        self._append_log(name, "[control] " + " ".join(command))
        process = subprocess.Popen(
            command,
            cwd=str(WORKSPACE),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        threading.Thread(
            target=self._read_process_output,
            args=(name, process, log_file),
            name=f"{name}-log-reader",
            daemon=True,
        ).start()
        return ManagedProcess(name=name, command=command, process=process, log_path=log_path)

    def _read_process_output(self, name: str, process: subprocess.Popen, log_file):
        try:
            if process.stdout is None:
                return
            for line in process.stdout:
                text = line.rstrip()
                log_file.write(text + "\n")
                self._append_log(name, text)
        finally:
            log_file.close()

    def _wait_for_odin_warmup(self) -> Tuple[bool, str]:
        deadline = time.monotonic() + ODIN_WARMUP_S
        while time.monotonic() < deadline:
            if self._stop_requested.is_set():
                return False, "Stopped by user"
            with self._lock:
                odin = self._odin
            if odin is None:
                return False, "Odin process exited"
            rc = odin.process.poll()
            if rc is not None:
                return False, f"Odin process exited with code {rc}"
            remaining = max(0.0, deadline - time.monotonic())
            self._set_state("Odin", "Starting", f"Warmup {remaining:.0f}s")
            time.sleep(0.5)
        return True, f"Odin warmup complete ({ODIN_WARMUP_S:.0f}s)"

    def _stop_managed(self, managed: ManagedProcess, graceful_timeout: float):
        process = managed.process
        if process.poll() is None:
            self._signal_group(process, signal.SIGINT)
            try:
                process.wait(timeout=graceful_timeout)
            except subprocess.TimeoutExpired:
                self._append_log(managed.name, "[control] SIGINT timeout; sending SIGTERM")
                self._signal_group(process, signal.SIGTERM)
                try:
                    process.wait(timeout=TERM_TIMEOUT_S)
                except subprocess.TimeoutExpired:
                    self._append_log(managed.name, "[control] SIGTERM timeout; sending SIGKILL")
                    self._signal_group(process, signal.SIGKILL)
                    process.wait(timeout=TERM_TIMEOUT_S)

    @staticmethod
    def _signal_group(process: subprocess.Popen, sig: signal.Signals):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass


class ControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Odin + FAST-LIVO2 Control")
        self.geometry("1180x760")
        self.minsize(980, 620)

        self._ui_queue: "queue.Queue[Tuple[str, object]]" = queue.Queue()
        self._controller = OdinLivoController(self._ui_queue)

        self._states = {
            "Odin": tk.StringVar(value="Idle"),
            "FAST-LIVO2": tk.StringVar(value="Idle"),
            "System": tk.StringVar(value="Ready"),
        }
        self._details = {
            "Odin": tk.StringVar(value=""),
            "FAST-LIVO2": tk.StringVar(value=""),
            "System": tk.StringVar(value=""),
        }
        self._rviz = tk.BooleanVar(value=False)
        self._fast_livo_pcd = tk.BooleanVar(value=False)
        self._fast_livo_image = tk.BooleanVar(value=False)
        self._odin_recorddata = tk.BooleanVar(value=False)
        self._save_translation_m = tk.StringVar(value="0.2")
        self._save_rotation_deg = tk.StringVar(value="10.0")
        self._log_widgets: Dict[str, tk.Text] = {}

        self._build_ui()
        self.after(200, self._drain_queue)
        self.after(1000, self._poll_processes)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(4, weight=1)

        top = ttk.Frame(self, padding=12)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(7, weight=1)

        ttk.Button(top, text="Start", command=self._start).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(top, text="Stop", command=self._stop).grid(row=0, column=1, padx=(0, 16))
        ttk.Checkbutton(top, text="RViz", variable=self._rviz).grid(row=0, column=2, padx=(0, 16))
        ttk.Checkbutton(top, text="FAST-LIVO2 PCD", variable=self._fast_livo_pcd).grid(row=0, column=3, padx=(0, 16))
        ttk.Checkbutton(top, text="FAST-LIVO2 Image", variable=self._fast_livo_image).grid(row=0, column=4, padx=(0, 16))
        ttk.Checkbutton(top, text="Odin Recorddata", variable=self._odin_recorddata).grid(row=0, column=5, padx=(0, 16))
        ttk.Label(top, textvariable=self._states["System"]).grid(row=0, column=6, sticky="w")
        ttk.Label(top, textvariable=self._details["System"]).grid(row=0, column=7, sticky="e")

        status = ttk.LabelFrame(self, text="Status", padding=12)
        status.grid(row=1, column=0, sticky="ew", padx=12, pady=(0, 8))
        status.columnconfigure(1, weight=1)
        status.columnconfigure(3, weight=1)
        for col, service in enumerate(("Odin", "FAST-LIVO2")):
            offset = col * 2
            ttk.Label(status, text=service).grid(row=0, column=offset, sticky="w", padx=(0, 8))
            ttk.Label(status, textvariable=self._states[service], width=14).grid(row=0, column=offset + 1, sticky="w")
            ttk.Label(status, textvariable=self._details[service]).grid(
                row=1, column=offset, columnspan=2, sticky="ew", pady=(4, 0)
            )

        env = ttk.LabelFrame(self, text="Environment", padding=12)
        env.grid(row=2, column=0, sticky="ew", padx=12, pady=(0, 8))
        env.columnconfigure(0, weight=1)
        env_text = (
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '33')}   "
            f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')}   "
            f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', '1')}   "
            f"ROS_LOG_DIR={ROS_LOG_DIR}"
        )
        ttk.Label(env, text=env_text).grid(row=0, column=0, sticky="w")

        save_gate = ttk.LabelFrame(self, text="Save Gate", padding=12)
        save_gate.grid(row=3, column=0, sticky="ew", padx=12, pady=(0, 8))
        save_gate.columnconfigure(4, weight=1)
        ttk.Label(save_gate, text="Translation m").grid(row=0, column=0, sticky="w", padx=(0, 8))
        ttk.Spinbox(
            save_gate,
            from_=0.01,
            to=100.0,
            increment=0.01,
            textvariable=self._save_translation_m,
            width=10,
        ).grid(row=0, column=1, sticky="w", padx=(0, 24))
        ttk.Label(save_gate, text="Rotation deg").grid(row=0, column=2, sticky="w", padx=(0, 8))
        ttk.Spinbox(
            save_gate,
            from_=0.1,
            to=360.0,
            increment=0.5,
            textvariable=self._save_rotation_deg,
            width=10,
        ).grid(row=0, column=3, sticky="w", padx=(0, 24))

        main = ttk.Frame(self)
        main.grid(row=4, column=0, sticky="nsew", padx=12, pady=(0, 12))
        self._build_logs(main)

    def _build_logs(self, parent: ttk.Frame):
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(0, weight=1)
        notebook = ttk.Notebook(parent)
        notebook.grid(row=0, column=0, sticky="nsew")
        for name in ("Odin", "FAST-LIVO2", "System"):
            frame = ttk.Frame(notebook)
            frame.rowconfigure(0, weight=1)
            frame.columnconfigure(0, weight=1)
            text = tk.Text(frame, wrap="word", height=20, font=("Monospace", 10))
            scroll = ttk.Scrollbar(frame, command=text.yview)
            text.configure(yscrollcommand=scroll.set)
            text.grid(row=0, column=0, sticky="nsew")
            scroll.grid(row=0, column=1, sticky="ns")
            notebook.add(frame, text=name)
            self._log_widgets[name] = text

    def _start(self):
        try:
            save_translation_m = float(self._save_translation_m.get())
            save_rotation_deg = float(self._save_rotation_deg.get())
            if save_translation_m <= 0.0 or save_rotation_deg <= 0.0:
                raise ValueError("Save gate values must be positive")
        except ValueError as exc:
            self._states["System"].set("Failed")
            self._details["System"].set("Invalid save gate")
            self._add_log("System", f"[control] Invalid save gate: {exc}")
            return

        self._states["System"].set("Starting")
        self._details["System"].set("")
        self._controller.start(RunOptions(
            rviz=self._rviz.get(),
            fast_livo_pcd=self._fast_livo_pcd.get(),
            fast_livo_image=self._fast_livo_image.get(),
            odin_recorddata=self._odin_recorddata.get(),
            save_translation_m=save_translation_m,
            save_rotation_deg=save_rotation_deg,
        ))

    def _stop(self):
        self._states["System"].set("Stopping")
        self._controller.stop()

    def _drain_queue(self):
        try:
            while True:
                kind, payload = self._ui_queue.get_nowait()
                if kind == "state":
                    service, state, detail = payload
                    self._set_state(service, state, detail)
                elif kind == "log":
                    service, line = payload
                    self._add_log(service, line)
                elif kind == "error":
                    self._set_state("System", "Failed", str(payload))
                    self._add_log("System", str(payload))
                elif kind == "process_exit":
                    service, rc = payload
                    self._add_log(service, f"[control] process exited with code {rc}")
                    self._set_state(service, "Exited", f"Exit code {rc}")
        except queue.Empty:
            pass
        self.after(200, self._drain_queue)

    def _poll_processes(self):
        self._controller.poll_processes()
        self.after(1000, self._poll_processes)

    def _set_state(self, service: str, state: str, detail: str):
        if service in self._states:
            self._states[service].set(state)
            self._details[service].set(detail)
        if service != "System":
            self._states["System"].set("Ready" if state in {"Stopped", "Running"} else state)

    def _add_log(self, service: str, line: str):
        widget = self._log_widgets.get(service) or self._log_widgets["System"]
        widget.insert("end", line + "\n")
        line_count = int(widget.index("end-1c").split(".")[0])
        if line_count > 500:
            widget.delete("1.0", f"{line_count - 500}.0")
        widget.see("end")

    def _on_close(self):
        self._controller.stop_blocking()
        self.destroy()


def main():
    app = ControlPanel()
    app.mainloop()


if __name__ == "__main__":
    main()

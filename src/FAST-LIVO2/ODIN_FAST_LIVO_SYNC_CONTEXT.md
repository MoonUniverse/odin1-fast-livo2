# Odin ROS Driver + FAST-LIVO2 Context

Date: 2026-05-22
Workspace: `/home/alienware/livo_workspace`
Branch used in both repos: `odin-ros-driver-fast-livo-sync`

## Goal

Prepare Odin ROS driver data for FAST-LIVO2. Required topics:

- `/odin1/imu`
- `/odin1/cloud_raw`
- `/odin1/image/undistorted`

These three topics must be stable and tightly time-aligned enough for FAST-LIVO2 LIO/VIO.

## Driver Runtime Context

Use the optimized Odin driver launch:

```bash
cd /home/alienware/livo_workspace
source install/setup.bash
ros2 launch odin_ros_driver odin1_fast_livo_ros2.launch.py
```

The optimized driver config keeps only needed streams:

- RGB device stream enabled only to produce undistorted image
- raw image disabled
- compressed image disabled
- cloud_slam disabled
- cloud_render disabled
- odom disabled
- recorddata disabled
- devstatus log enabled

The default full Odin launch was not suitable for FAST-LIVO2 because `/odin1/image/undistorted` dropped to about `1.02 Hz` under default load.

## FAST-LIVO2 Changes

Repository: `src/FAST-LIVO2`

Important changes:

- Added `ODIN = 8` lidar type.
- Added `Preprocess::odin_handler()` for Odin `/odin1/cloud_raw`.
- Odin parser reads fields directly from `sensor_msgs/PointCloud2`:
  - `x/y/z`: `FLOAT32`
  - `intensity`: `UINT8`
  - `confidence`: `UINT16`
  - `offset_time`: `FLOAT32`
- `offset_time` is converted from seconds to milliseconds and stored in `PointType.curvature`, matching FAST-LIVO2 timing expectations.
- Added Odin FAST-LIVO2 config:
  - `config/odin.yaml`
  - `config/camera_odin.yaml`
- Added Odin FAST-LIVO2 launch:
  - `launch/mapping_odin.launch.py`
- Configured topics:
  - image: `/odin1/image/undistorted`
  - lidar: `/odin1/cloud_raw`
  - imu: `/odin1/imu`
- Set `img_time_offset: 0.001685342`, based on measured image/cloud signed offset.

Known local/uncommitted item not part of this work:

- `src/FAST-LIVO2/.vscode/` is local editor state and should not be committed.

## Test Results

### 120 Second Optimized Driver Test

- `/odin1/imu`: `399.429 Hz`
- `/odin1/cloud_raw`: `10.259 Hz`
- `/odin1/image/undistorted`: `10.260 Hz`
- `/odin1/cloud_raw` and `/odin1/image/undistorted`: both `1231` frames
- cloud to nearest IMU p95: about `1.185 ms`
- image to nearest IMU p95: about `1.188 ms`
- image to nearest cloud p95: about `1.685 ms`

### FAST-LIVO2 Runtime Smoke Test

Command:

```bash
source install/setup.bash
ros2 launch fast_livo mapping_odin.launch.py rviz:=false
```

Result:

- FAST-LIVO2 subscribed successfully.
- IMU initialized.
- LIO processed Odin raw cloud, raw feature count around `27700`.
- VIO processed `/odin1/image/undistorted` and appended visual map points.

### 5 Minute Motion Test

The device was moved during the test.

Results:

- `/odin1/imu`: `399.434 Hz`, count `119824`, p95 interval `3.233 ms`, max interval `24.073 ms`
- `/odin1/cloud_raw`: `10.260 Hz`, count `3078`, p95 interval `106.540 ms`, max interval `134.883 ms`
- `/odin1/image/undistorted`: `10.260 Hz`, count `3078`, p95 interval `105.950 ms`, max interval `112.045 ms`
- `/odin1/cloud_raw.offset_time`: `FLOAT32`, range `0.000000` to `0.094368 s`
- cloud to nearest IMU abs p95: `1.185 ms`
- image to nearest IMU abs p95: `1.188 ms`
- image to nearest cloud abs p95: `1.685 ms`
- image stamp is consistently about `1.685 ms` earlier than corresponding cloud stamp.

Conclusion:

- The three required topics are not bit-identical in `header.stamp`.
- They are stable on the same device time axis.
- For FAST-LIVO2, `img_time_offset: 0.001685342` aligns image timestamps to cloud timestamps.
- Motion did not materially affect topic frequency in the 5 minute test.

### 2026-05-25 5 Minute Topic Stability Retest

Measured for 300 seconds on:

- `/odin1/imu`
- `/odin1/cloud_raw`
- `/odin1/image/undistorted`

Results:

- `/odin1/imu`: count `119845`, rate `399.551 Hz`, stamp p95 `2.546 ms`, stamp p99 `2.558 ms`
- `/odin1/cloud_raw`: count `3077`, rate `10.260 Hz`, stamp p95 `97.518 ms`, stamp p99 `97.526 ms`
- `/odin1/image/undistorted`: count `3077`, rate `10.259 Hz`, stamp p95 `97.518 ms`, stamp p99 `97.526 ms`, shape `1600x1296 bgr8`
- cloud to nearest IMU abs p95 `1.186 ms`
- image to nearest IMU abs p95 `1.189 ms`
- image to nearest cloud abs p95 `1.685 ms`

Conclusion:

- The retest did not reproduce topic frequency instability.
- Cloud and image counts were identical.
- Header timestamps were stable; receive interval spikes looked like host/DDS
  subscriber scheduling jitter.

## FAST-LIVO2 Odin Config Audit

Checked on 2026-05-25:

- `mapping_odin.launch.py` loads `config/odin.yaml` and `config/camera_odin.yaml`.
- Source and installed copies of the Odin launch/config files were identical.
- Topics match the driver: `/odin1/imu`, `/odin1/cloud_raw`, `/odin1/image/undistorted`.
- `preprocess.lidar_type: 8` selects `Preprocess::odin_handler()`.
- `camera_odin.yaml` image size `1600x1296` matches the current undistorted image.
- `scale: 0.5` is currently harmless because no resize occurs when the incoming
  image already matches the camera model size.
- `img_time_offset: 0.001685342` remains consistent with measured image/cloud
  nearest-neighbor offset.
- `Rcl` is near-orthonormal with determinant about `1.000002`.
- `extrinsic_T` matches the Odin driver's fixed LiDAR-to-IMU translation.
- No obvious Odin FAST-LIVO2 config error was found.

## Recommended FAST-LIVO2 Runtime

```bash
cd /home/alienware/livo_workspace
source install/setup.bash
ros2 launch fast_livo mapping_odin.launch.py rviz:=false
```

## IMU Calibration Runtime

For static IMU calibration, use the Odin IMU-only driver launch:

```bash
cd /home/alienware/livo_workspace
source install/setup.bash
ros2 launch odin_ros_driver odin1_imu_only_ros2.launch.py
```

It uses `control_command_imu_only.yaml`, with RGB, DTOF cloud, odom,
cloud_slam, cloud_render, recorddata, devstatus logging, and image outputs
disabled. The driver also creates ROS publishers according to these switches,
so only `/odin1/imu` should be advertised and publishing.

Record static data from another terminal:

```bash
cd /home/alienware/livo_workspace
source install/setup.bash
ros2 run fast_livo record_imu_static.py --topic /odin1/imu --duration 7200 --output /tmp/odin_imu_static.txt
```

## Commit and NUC Deployment

Functional commit:

```text
1d21ef3 Add Odin IMU calibration workflow
```

Synchronized and built on:

```text
nuc13@10.56.238.242:/home/nuc13/livo_workspace
```

NUC build command:

```bash
cd /home/nuc13/livo_workspace
source /opt/ros/humble/setup.bash
env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
  colcon build --packages-select livox_ros_driver2 vikit_common odin_ros_driver fast_livo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

Build result: all four packages built successfully with only existing warnings.

## 2026-05-25 Handoff: Latest Uncommitted Work

The root workspace file `/home/alienware/livo_workspace/ODIN_FAST_LIVO_SYNC_CONTEXT.md`
contains the full latest handoff. Key points:

- Latest files were synchronized to:
  - `nuc13@10.56.238.242:/home/nuc13/livo_workspace`
- FAST-LIVO2 now supports pose-gated PCD/image saving:
  - `pcd_save.trigger_mode: pose_delta`
  - `image_save.trigger_mode: pose_delta`
  - `save_pose_gate.translation_m: 0.2`
  - `save_pose_gate.rotation_deg: 10.0`
- FAST-LIVO2 saves final maps on shutdown:
  - `src/FAST-LIVO2/Log/pcd/final_map.pcd`
  - `src/FAST-LIVO2/Log/pcd/final_map_rgb.pcd`
- FAST-LIVO2 writes internal callback-based topic diagnostics on shutdown:
  - `/tmp/fast_livo_topic_reports/fast_livo_internal_report_*.json`
  - `/tmp/fast_livo_topic_reports/fast_livo_internal_report_*.md`
- External Python `topic_monitor/topic_report` is disabled by default in
  `mapping_odin.launch.py` because it can undercount large image/cloud topics.
  Use internal reports as source of truth.
- Ctrl-C/shutdown fixes were added:
  - `main.cpp` exits with `_Exit(0)` after `mapper->run()`.
  - `LIVMapper::run()` catches `rclcpp::exceptions::RCLError` around `spin_some()`.
  - `topic_report.py` writes and exits with `os._exit(0)`.
- Latest inspected internal report:
  - `/tmp/fast_livo_topic_reports/fast_livo_internal_report_20260525_154538.md`
- Latest internal callback frequencies:
  - `/odin1/imu`: `399.047 Hz`, header `399.390 Hz`
  - `/odin1/cloud_raw`: `9.017 Hz`, header `9.014 Hz`
  - `/odin1/image/undistorted`: `10.266 Hz`, header `10.259 Hz`
- Current real issue:
  - Image and IMU are stable.
  - `/odin1/cloud_raw` is genuinely less stable in FAST-LIVO2 callbacks.
  - Cloud header p95 is about `194.948 ms`, p99 about `389.894 ms`, indicating skipped cloud intervals.
- Recommended next step:
  - Instrument Odin driver cloud receive/enqueue/drop/publish counters in
    `host_sdk_sample.h` / `host_sdk_sample.cpp` to locate where cloud frames are lost.

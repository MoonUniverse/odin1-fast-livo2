# Odin Direct SDK FAST-LIVO2 Context

Last updated: 2026-06-01

## Current Branch And Commit

- Branch: `odin-fast-livo2-direct-sdk`
- Remote: `origin/odin-fast-livo2-direct-sdk`
- Commit: `197a3af Add Odin direct SDK input for FAST-LIVO2`
- Base repository: `git@github.com:MoonUniverse/odin1-fast-livo2.git`

## Goal

The goal of this branch is to remove DDS from the high-rate Odin1 to FAST-LIVO2 sensor path.

Before this change, Odin1 published IMU, DTOF cloud, and camera image as ROS 2 topics, and FAST-LIVO2 subscribed to those topics. In real hardware runs, DDS scheduling and transport jitter could make the effective arrival rate unstable, which could degrade FAST-LIVO2 performance.

This branch embeds Odin SDK access directly inside the FAST-LIVO2 mapping process. IMU, cloud, and image frames are still converted to ROS message types internally so the existing FAST-LIVO2 callbacks can be reused, but the data does not traverse DDS before entering FAST-LIVO2.

## Main Implementation

### Odin direct SDK wrapper

New files:

- `src/odin_ros_driver/include/odin_direct_sdk.h`
- `src/odin_ros_driver/src/odin_direct_sdk.cpp`

The wrapper:

- owns Odin SDK initialization and shutdown;
- configures the Odin device using the existing `control_command_fast_livo.yaml`;
- registers direct SDK callbacks for IMU, raw DTOF, and raw RGB;
- copies raw SDK payloads into bounded queues inside the SDK callback;
- converts queued data to `sensor_msgs::msg::Imu`, `sensor_msgs::msg::PointCloud2`, and `sensor_msgs::msg::Image` on worker threads;
- exposes `spinSome()` so FAST-LIVO2 can drain prepared messages from its mapping loop;
- optionally publishes debug ROS topics if `publish_debug_topics:=true`;
- optionally enables Odin recorddata output when `recorddata:=true`;
- reports SDK/delivery/drop counters during shutdown.

Important defaults:

- `publish_debug_topics` defaults to `false`
- `recorddata` defaults to `false`
- output queues are bounded and drop oldest frames when full

### FAST-LIVO2 integration

Modified files:

- `src/FAST-LIVO2/include/LIVMapper.h`
- `src/FAST-LIVO2/src/LIVMapper.cpp`
- `src/FAST-LIVO2/CMakeLists.txt`
- `src/FAST-LIVO2/package.xml`
- `src/FAST-LIVO2/config/odin.yaml`

FAST-LIVO2 now supports:

- `common.input_source: ros_topic` for the legacy DDS path;
- `common.input_source: odin_direct` for the new direct SDK path.

When `common.input_source` is `odin_direct`, `LIVMapper` starts `odin_ros_driver::OdinDirectSdk` and wires direct callbacks into the existing FAST-LIVO2 methods:

- IMU -> `LIVMapper::imu_cbk`
- DTOF cloud -> `LIVMapper::standard_pcl_cbk`
- image -> `LIVMapper::img_cbk`

`LIVMapper::run()` calls `odin_direct_sdk_->spinSome()` from the mapping loop. Internal diagnostics record callback arrival rate and header-stamp rate for IMU/cloud/image, without relying on external ROS topic subscribers.

### Direct launch file

New file:

- `src/FAST-LIVO2/launch/mapping_odin_direct.launch.py`
- `src/FAST-LIVO2/launch/mapping_odin_direct_lio.launch.py`

Key launch arguments:

- `rviz`, default `false`
- `pcd_save`, default `false`
- `final_map_save`, default `false`
- `image_save`, default `false`
- `output_run_dir`, default `""`
- `odin_config`, default installed `odin_ros_driver/config/control_command_fast_livo.yaml`
- `recorddata`, default `false`
- `recorddata_dir`, default `""`
- `publish_debug_topics`, default `false`

The `final_map_save` argument was added because `src/FAST-LIVO2/config/odin.yaml` still has `pcd_save.final_map_save_en: true`. Without an explicit override, a direct test with `pcd_save:=false` could still enter the final-map save path during shutdown.

### Direct LIO-only mode

FAST-LIVO2 now has an explicit Odin direct LIO-only launch:

- `src/FAST-LIVO2/launch/mapping_odin_direct_lio.launch.py`

This launch keeps the direct SDK input path but sets:

- `common.img_en: 0`
- `image_save.img_save_en: false`

With `common.img_en=0`, `LIVMapper` selects `ONLY_LIO`, skips camera/VIO initialization, does not subscribe to image data, and passes `enable_image_stream=false` to `OdinDirectSdk`.

`OdinDirectSdk` then forces the RGB stream off at runtime:

- `sendrgb=0`
- `sendrgbundistort=0`

This means the device should stream only IMU and DTOF cloud for FAST-LIVO2 LIO. The existing `control_command_fast_livo.yaml` is not modified, so the previously validated direct LIVO launch remains the default behavior.

### GUI integration

Modified file:

- `src/odin_livo_control/odin_livo_control/gui.py`

The GUI now defaults to direct SDK mode. In this mode it launches only FAST-LIVO2 through `mapping_odin_direct.launch.py`; it does not separately launch the Odin ROS driver for the IMU/cloud/image path.

Legacy DDS mode is still available with:

```bash
ODIN_LIVO_LEGACY_DDS=1 ros2 run odin_livo_control gui
```

In direct mode, the GUI explicitly passes:

- `publish_debug_topics:=false`
- `final_map_save:=false`
- `recorddata:=true/false` based on the GUI checkbox

## Build And Validation

Local build command:

```bash
colcon build --packages-select odin_ros_driver fast_livo odin_livo_control --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result: passed.

NUC build command:

```bash
ssh nuc13@10.56.238.242 'cd /home/nuc13/livo_workspace && source /opt/ros/humble/setup.bash && colcon build --packages-select odin_ros_driver fast_livo odin_livo_control --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

Result: passed.

## NUC13 Hardware Test

Target:

- Host: `nuc13@10.56.238.242`
- Workspace: `/home/nuc13/livo_workspace`
- Odin USB device observed by `lsusb`: `2207:0019 Fuzhou Rockchip Electronics Company hawk`

Final successful hardware run:

```bash
ssh nuc13@10.56.238.242 'cd /home/nuc13/livo_workspace && source install/setup.bash && RUN_DIR=/tmp/fast_livo_direct_hw_final_off_$(date +%Y%m%d_%H%M%S) && echo RUN_DIR=${RUN_DIR} && ROS_LOG_DIR=/tmp/ros-log timeout --signal=SIGINT 75 ros2 launch fast_livo mapping_odin_direct.launch.py rviz:=false output_run_dir:=${RUN_DIR} pcd_save:=false final_map_save:=false image_save:=false publish_debug_topics:=false recorddata:=false'
```

Successful run directory:

- `/tmp/fast_livo_direct_hw_final_off_20260529_115532`

Clean shutdown:

- `fastlivo_mapping` finished cleanly
- no FAST-LIVO2 or Odin direct process remained after the test

SDK stats from the successful 75 second run:

- SDK IMU/cloud/image: `29679/761/761`
- Delivered IMU/cloud/image: `29679/761/761`
- Dropped IMU/cloud/image: `0/0/0`

Internal callback diagnostics:

- Report: `/tmp/fast_livo_direct_hw_final_off_20260529_115532/topic_reports/fast_livo_internal_report_20260529_115647.md`
- Duration: `75.1s`
- IMU: `29611` samples, `399.563 Hz`, header rate `399.583 Hz`
- cloud_raw: `761` samples, `10.258 Hz`, header rate `10.259 Hz`
- image/undistorted: `761` samples, `10.261 Hz`, header rate `10.259 Hz`
- arrival p95:
  - IMU: `7.878 ms`
  - cloud_raw: `110.106 ms`
  - image/undistorted: `108.449 ms`
- header p95:
  - IMU: `2.547 ms`
  - cloud_raw: `97.517 ms`
  - image/undistorted: `97.517 ms`

## Shutdown Issue Found And Fixed

During early NUC testing, the direct SDK data path ran correctly, but shutdown reported:

- `exit code -11`

This happened after:

- Odin direct SDK stats were printed;
- SDK stopped;
- internal topic report was written.

The issue was reproduced with both `timeout --signal=SIGINT` and a single direct SIGINT to `ros2 launch`, so it was not only a double-SIGINT artifact.

Root cause found during testing:

- `src/FAST-LIVO2/config/odin.yaml` has `pcd_save.final_map_save_en: true`;
- the direct launch initially overrode only `pcd_save.pcd_save_en`;
- with `pcd_save:=false`, FAST-LIVO2 could still enter final-map saving during shutdown;
- this path caused the observed `-11` after report writing.

Fix:

- added `final_map_save` launch argument to `mapping_odin_direct.launch.py`, default `false`;
- set `pcd_save.final_map_save_en` from this launch argument;
- GUI direct mode passes `final_map_save:=false`;
- final NUC hardware test exited cleanly.

## NUC Sync Notes

The NUC workspace had pre-existing local changes before sync. They were intentionally preserved.

Observed pre-existing NUC differences included:

- `D src/Global-LVBA`
- modified `src/odin_ros_driver/config/control_command.yaml`
- modified `src/odin_ros_driver/config/control_command_fast_livo.yaml`
- modified `src/odin_ros_driver/src/host_sdk_sample.cpp`
- `.claude/`

Important preserved NUC setting:

- `src/odin_ros_driver/config/control_command_fast_livo.yaml`
- `cloud_raw_confidence_threshold: 75`

Do not overwrite this NUC config casually when re-syncing.

When replacing Odin SDK headers/static libraries, rebuild the NUC packages with
`--cmake-clean-first`. A normal incremental rebuild can leave a stale
`fastlivo_mapping` link/object product and cause an immediate glibc malloc
assertion on GUI start.

```bash
cd /home/nuc13/livo_workspace
source /opt/ros/humble/setup.bash
env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
  colcon build --packages-select odin_ros_driver fast_livo odin_livo_control \
  --executor sequential \
  --cmake-clean-first \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

## Common Commands

Direct launch on NUC:

```bash
cd /home/nuc13/livo_workspace
source install/setup.bash
ros2 launch fast_livo mapping_odin_direct.launch.py \
  rviz:=false \
  output_run_dir:=/tmp/fast_livo_direct_test \
  pcd_save:=false \
  final_map_save:=false \
  image_save:=false \
  publish_debug_topics:=false \
  recorddata:=false
```

Show direct launch arguments:

```bash
cd /home/nuc13/livo_workspace
source install/setup.bash
ros2 launch fast_livo mapping_odin_direct.launch.py --show-args
```

Run GUI in default direct SDK mode:

```bash
cd /home/nuc13/livo_workspace
source install/setup.bash
ros2 run odin_livo_control gui
```

Run direct SDK LIO-only from CLI:

```bash
cd /home/nuc13/livo_workspace
source install/setup.bash
ros2 launch fast_livo mapping_odin_direct_lio.launch.py \
  rviz:=false \
  output_run_dir:=/tmp/fast_livo_direct_lio_test \
  pcd_save:=false \
  final_map_save:=false \
  publish_debug_topics:=false \
  recorddata:=false
```

Run GUI direct SDK LIO-only:

- start the GUI normally;
- check `LIO only`;
- press `Start`.

Expected LIO-only runtime signs:

- FAST-LIVO2 logs `FAST-LIVO2 mode: ONLY_LIO`;
- Odin direct SDK logs `streams active: rgb=0 imu=1 dtof=1`;
- SDK image stats remain zero;
- no `[ VIO ]` or `Get image` messages are expected.

Run GUI in legacy DDS mode:

```bash
cd /home/nuc13/livo_workspace
source install/setup.bash
ODIN_LIVO_LEGACY_DDS=1 ros2 run odin_livo_control gui
```

## Save/Layering Debug Notes

User test result on 2026-06-01:

- Full LIVO with runtime PCD/image saving disabled behaved normally in the same indoor scene.
- Runs that still showed severe wall layering had GUI `FAST-LIVO2 Recorddata` enabled as an additional variable, not only PCD/image saving.
- NUC log run `20260601_173325` showed healthy SDK delivery and no async save drops: `pcd enqueued/written/dropped/failed=226/226/0/0`, `image=227/227/0/0`.
- NUC log run `20260601_173624` wrote roughly `2.2G` Odin recorddata, `2.9G` images, and `2.5G` body-frame PCDs, then exited with code `-11`.

Important interpretation:

- GUI `FAST-LIVO2 Recorddata` is a separate heavy write path. Keep it off when isolating PCD/image save impact.
- `pcd_save.type: 1` writes per-frame body-frame scans under `all_pcd_body`. Directly overlaying those PCD files without applying `lidar_poses.txt` will look layered by design; they are not world-frame fused map files.
- Next isolation test should use Full LIVO + RViz + PCD/Image save enabled, `FAST-LIVO2 Recorddata` disabled, and a coarser save gate such as translation `1.0` m and rotation `20.0` deg.

## Follow-Up Considerations

- If final map saving is needed in direct mode, test `final_map_save:=true` separately and inspect the final-map save path before relying on it.
- Runtime PCD/image saving is asynchronous by default (`pcd_save.async_save_en:=true`, `image_save.async_save_en:=true`) with bounded queues. If disk cannot keep up, old save jobs are dropped so mapping is not blocked.
- Direct launch exposes async save controls as `pcd_async_save`, `pcd_async_queue_size`, `image_async_save`, and `image_async_queue_size`.
- If recorddata is enabled, verify disk throughput and output size on NUC because SLAM cloud/odom streams are also activated.
- If debug ROS topics are enabled with `publish_debug_topics:=true`, DDS load returns for observers, but FAST-LIVO2 still consumes the direct in-process path.
- The direct SDK wrapper currently converts SDK data into ROS message objects to reuse existing FAST-LIVO2 callbacks. This keeps the change scoped, but a future optimization could pass lighter internal structs if callback overhead becomes measurable.

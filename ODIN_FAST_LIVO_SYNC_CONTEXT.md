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

## Initial Finding

The default full Odin launch was not suitable for FAST-LIVO2:

- Full launch included RViz, recorddata, raw image, compressed image, cloud_slam, cloud_render and demo nodes.
- In a 10 minute test, most streams were stable, but `/odin1/image/undistorted` averaged only about `1.02 Hz`.
- `/odin1/image/compressed` stayed near `10 Hz`, so the issue was host-side load/publication, not the device RGB stream itself.
- `recorddata` produced `6.8G` in that run and was deleted after measurement.

## Driver Changes

Repository: `src/odin_ros_driver`

Important changes:

- `host_sdk_sample` now reads the ROS2 `config_file` parameter from launch instead of always using `config/control_command.yaml`.
- Added `sendrgbraw` config support.
- `publishRgb()` now honors:
  - `sendrgbraw`
  - `sendrgbcompressed`
  - `sendrgbundistort`
- Undistortion is only computed when `sendrgbundistort=1`.
- Added FAST-LIVO optimized config:
  - `config/control_command_fast_livo.yaml`
- Added FAST-LIVO optimized launch:
  - `launch_ROS2/odin1_fast_livo_ros2.launch.py`
- Optimized config keeps only needed runtime streams:
  - RGB device stream enabled only to produce undistorted image
  - raw image disabled
  - compressed image disabled
  - cloud_slam disabled
  - cloud_render disabled
  - odom disabled
  - recorddata disabled
  - devstatus log enabled
- Ctrl-C shutdown was changed to avoid SDK cleanup paths that caused abort/segfault. On shutdown request, local threads/files are cleaned up and the process exits cleanly with `_Exit(0)`.

Known local/uncommitted item not part of this work:

- `src/odin_ros_driver/config/control_command.yaml` has local modifications (`sendrgbundistort: 1`, `recorddata: 1`). The FAST-LIVO path does not depend on this file.

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

Command:

```bash
source install/setup.bash
ros2 launch odin_ros_driver odin1_fast_livo_ros2.launch.py
```

Measured:

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
- Short run lasted about 35 seconds and exited due to `timeout`, not due to a node crash.

### 5 Minute Motion Test

The device was moved during the test.

Results:

- `/odin1/imu`
  - count: `119824`
  - rate: `399.434 Hz`
  - p95 interval: `3.233 ms`
  - p99 interval: `8.513 ms`
  - max interval: `24.073 ms`
- `/odin1/cloud_raw`
  - count: `3078`
  - rate: `10.260 Hz`
  - p95 interval: `106.540 ms`
  - p99 interval: `118.330 ms`
  - max interval: `134.883 ms`
- `/odin1/image/undistorted`
  - count: `3078`
  - rate: `10.260 Hz`
  - p95 interval: `105.950 ms`
  - p99 interval: `108.373 ms`
  - max interval: `112.045 ms`
  - shape: `1600x1296 bgr8`
- `/odin1/cloud_raw.offset_time`
  - field type: `FLOAT32`
  - range: `0.000000` to `0.094368 s`
- Synchronization:
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

Command:

```bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros-log
ros2 launch odin_ros_driver odin1_fast_livo_ros2.launch.py
```

Measured for 300 seconds with subscribers on `/odin1/imu`, `/odin1/cloud_raw`,
and `/odin1/image/undistorted`.

Results:

- `/odin1/imu`: count `119845`, rate `399.551 Hz`
  - receive interval p95 `3.206 ms`, p99 `8.836 ms`, max `24.632 ms`
  - stamp interval p95 `2.546 ms`, p99 `2.558 ms`, max `5.044 ms`
- `/odin1/cloud_raw`: count `3077`, rate `10.260 Hz`
  - receive interval p95 `114.851 ms`, p99 `121.357 ms`, max `130.725 ms`
  - stamp interval p95 `97.518 ms`, p99 `97.526 ms`, max `97.551 ms`
- `/odin1/image/undistorted`: count `3077`, rate `10.259 Hz`
  - receive interval p95 `105.906 ms`, p99 `108.416 ms`, max `111.339 ms`
  - stamp interval p95 `97.518 ms`, p99 `97.526 ms`, max `97.551 ms`
  - shape `1600x1296 bgr8`
- `/odin1/cloud_raw` fields: `x`, `y`, `z`, `intensity`, `confidence`, `offset_time`
- Synchronization:
  - cloud to nearest IMU abs p95 `1.186 ms`, p99 `1.240 ms`
  - image to nearest IMU abs p95 `1.189 ms`, p99 `1.239 ms`
  - image to nearest cloud abs p95 `1.685 ms`, p99 `1.685 ms`

Conclusion:

- The 5 minute retest did not reproduce topic frequency instability.
- Cloud and undistorted image counts were identical.
- Header stamp intervals were stable; larger receive interval p99/max values
  were attributed to host/DDS/subscriber scheduling jitter.
- `img_time_offset: 0.001685342` remains consistent with the nearest-neighbor
  image/cloud offset.

## FAST-LIVO2 Odin Config Audit

Checked on 2026-05-25:

- `mapping_odin.launch.py` loads `config/odin.yaml` and `config/camera_odin.yaml`.
- Source and installed copies of `odin.yaml`, `camera_odin.yaml`, and
  `mapping_odin.launch.py` were identical.
- Configured topics match the driver:
  - `/odin1/imu`
  - `/odin1/cloud_raw`
  - `/odin1/image/undistorted`
- `preprocess.lidar_type: 8` selects the Odin point cloud handler.
- `camera_odin.yaml` uses `1600x1296`, matching the current undistorted image.
- `camera_odin.yaml` has `scale: 0.5`, but this does not resize the current
  Odin image because the incoming image already matches the camera model size.
- `Rcl` is near-orthonormal with determinant about `1.000002`.
- `extrinsic_T: [-0.02663, 0.03447, 0.02174]` matches the Odin driver's fixed
  LiDAR-to-IMU translation.
- No obvious Odin FAST-LIVO2 config error was found.

If FAST-LIVO2 runtime looks unstable, check logs for:

- `Throw one image frame`
- `Image need Jumps`
- `IMU and LiDAR not synced`
- VIO/LIO processing time spikes

## Build Verification

Successful builds:

```bash
colcon build --packages-select odin_ros_driver fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select odin_ros_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Only existing warnings were observed, mostly ignored `system()` return values in `host_sdk_sample.cpp`.

## Recommended Runtime

Terminal 1:

```bash
cd /home/alienware/livo_workspace
source install/setup.bash
ros2 launch odin_ros_driver odin1_fast_livo_ros2.launch.py
```

Terminal 2:

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

## Recorddata Handling

- FAST-LIVO optimized config uses `recorddata: 0`.
- Enabling Odin `recorddata` does not add or rename ROS topics, but it does add
  CPU work and heavy disk writes on the driver callback path. Use a separate
  config for short raw-data capture runs when FAST-LIVO2 real-time stability
  matters.
- No recorddata directories were left after the final tests.
- Earlier full-launch test generated `recorddata/20260522_095011`, total `6.8G`; it was deleted after measurement.

## FAST-LIVO2 Save and Diagnostics

- FAST-LIVO2 Odin config supports pose-gated PCD/image saving:
  - `pcd_save.trigger_mode: pose_delta`
  - `image_save.trigger_mode: pose_delta`
  - default gate: `0.2 m` translation or `10 deg` rotation
- `pcd_save.final_map_save_en: true` saves an accumulated FAST-LIVO2 PCD map at shutdown:
  - `src/FAST-LIVO2/Log/pcd/final_map.pcd`
  - `src/FAST-LIVO2/Log/pcd/final_map_rgb.pcd` when RGB-colored points are available
- FAST-LIVO2 writes an internal callback-based topic report at shutdown:
  - `/tmp/fast_livo_topic_reports/fast_livo_internal_report_*.json`
  - `/tmp/fast_livo_topic_reports/fast_livo_internal_report_*.md`
- `mapping_odin.launch.py` keeps external `topic_monitor/topic_report` available but disabled by default.
  - Enable with `topic_report:=true` only for lightweight observer checks.
  - External Python subscription can undercount large image/point-cloud topics.

## Commit and NUC Deployment

## 2026-05-25 Handoff: Save Logic, Shutdown, and Topic Diagnostics

This section records the latest uncommitted work and current debugging state.

### Implemented FAST-LIVO2 Changes

Files changed locally and synchronized to NUC:

- `src/FAST-LIVO2/include/LIVMapper.h`
- `src/FAST-LIVO2/src/LIVMapper.cpp`
- `src/FAST-LIVO2/src/main.cpp`
- `src/FAST-LIVO2/config/odin.yaml`
- `src/FAST-LIVO2/launch/mapping_odin.launch.py`
- `src/FAST-LIVO2/package.xml`
- `src/topic_monitor/setup.py`
- `src/topic_monitor/topic_monitor/topic_report.py`
- `ODIN_FAST_LIVO_SYNC_CONTEXT.md`

Feature changes:

- PCD/image saving supports pose-gated trigger mode:
  - `pcd_save.trigger_mode: pose_delta`
  - `image_save.trigger_mode: pose_delta`
  - `save_pose_gate.translation_m: 0.2`
  - `save_pose_gate.rotation_deg: 10.0`
- Final FAST-LIVO2 map saving is enabled by default for Odin:
  - `src/FAST-LIVO2/Log/pcd/final_map.pcd`
  - `src/FAST-LIVO2/Log/pcd/final_map_rgb.pcd` when colored points exist
- FAST-LIVO2 now writes an internal callback-based report at shutdown:
  - `/tmp/fast_livo_topic_reports/fast_livo_internal_report_*.json`
  - `/tmp/fast_livo_topic_reports/fast_livo_internal_report_*.md`
- `topic_monitor/topic_report` remains available but is disabled by default in `mapping_odin.launch.py`:
  - `topic_report:=false`
  - External Python subscribers can undercount large `sensor_msgs/Image` and `PointCloud2` topics, so do not use external `topic_report` as source of truth for image/cloud frequency.

Shutdown fixes:

- `src/FAST-LIVO2/src/main.cpp` now exits with `_Exit(0)` after `mapper->run()` returns. This avoids upstream FAST-LIVO2 teardown/destructor crashes after final map save.
- `LIVMapper::run()` catches `rclcpp::exceptions::RCLError` around `spin_some()` so Ctrl-C after `rclcpp` context shutdown does not abort with:
  - `failed to create wait set`
  - `Failed to create wait set in Executor constructor`
- `topic_report.py` writes its report and exits with `os._exit(0)` to avoid ROS Python shutdown/destroy-node exceptions after launch SIGINT.

### Build and NUC Sync State

Synchronized target:

```text
nuc13@10.56.238.241:/home/nuc13/livo_workspace
```

Verified on NUC after the latest edits:

```bash
cd /home/nuc13/livo_workspace
source /opt/ros/humble/setup.bash
env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
  colcon build --packages-select topic_monitor fast_livo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

Also verified:

```bash
source install/setup.bash
ros2 launch fast_livo mapping_odin.launch.py --show-args
```

Expected arguments now include:

```text
rviz default: false
topic_report default: false
topic_report_dir default: /tmp/fast_livo_topic_reports
```

### Latest Runtime Finding

Latest internal report inspected:

```text
/tmp/fast_livo_topic_reports/fast_livo_internal_report_20260525_154538.md
```

FAST-LIVO2 internal callback statistics:

```text
/odin1/imu                 399.047 Hz, header 399.390 Hz
/odin1/cloud_raw             9.017 Hz, header   9.014 Hz
/odin1/image/undistorted    10.266 Hz, header  10.259 Hz
```

Detailed cloud stats from that report:

```text
/odin1/cloud_raw count: 211
arrival p50: 98.924 ms
arrival p95: 197.269 ms
arrival p99: 390.402 ms
header p50: 97.474 ms
header p95: 194.948 ms
header p99: 389.894 ms
```

Interpretation:

- IMU is stable.
- Image is stable at the expected Odin cadence.
- Cloud is genuinely lower/less stable in FAST-LIVO2 callbacks. This is not the previous external Python report artifact.
- Cloud header intervals show real skipped intervals: p95 around 195 ms and p99 around 390 ms.
- Current likely next target is Odin driver cloud receive/queue/publish path, not FAST-LIVO2 image sync.

### External Report Caveat

Older files named `fast_livo_topic_report_*.json/md` are from the external Python `topic_monitor/topic_report` process. These reports repeatedly undercounted large image/cloud topics, even after raw subscriptions, because Python/DDS observation of large messages could not keep up reliably.

Use `fast_livo_internal_report_*.json/md` for FAST-LIVO2 input frequency.

### Recommended Next Debug Step

Instrument Odin driver cloud path in `src/odin_ros_driver/include/host_sdk_sample.h` / `src/odin_ros_driver/src/host_sdk_sample.cpp`:

- Count SDK DTOF callbacks received.
- Count cloud frames enqueued into `g_cloud_queue`.
- Count cloud queue drops when `CLOUD_QUEUE_MAX_SIZE` is exceeded.
- Count cloud frames published on `/odin1/cloud_raw`.
- Record cloud frame header timestamps and publish timestamps.
- Emit a shutdown report or periodic log showing receive/enqueue/drop/publish rates.

The goal is to determine whether `/odin1/cloud_raw` drops occur:

- before cloud enqueue,
- due to queue overflow,
- during cloud conversion/publish,
- or at DDS/subscriber delivery.

Latest functional commit before this context update:

```text
1d21ef3 Add Odin IMU calibration workflow
```

It includes:

- IMU Allan calibration BRW averaging fix.
- `imu_calibrate.py` installed for `ros2 run`.
- Odin IMU-only config and launch.
- Publisher creation gated by Odin stream config, so IMU-only mode only
  advertises `/odin1/imu` under `/odin1/*`.
- Documentation for IMU calibration and Odin runtime.

Synchronized target:

```text
nuc13@10.56.238.241:/home/nuc13/livo_workspace
```

NUC build was verified with:

```bash
cd /home/nuc13/livo_workspace
source /opt/ros/humble/setup.bash
env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
  colcon build --packages-select livox_ros_driver2 vikit_common odin_ros_driver fast_livo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

Build result: all four packages built successfully. Only existing warnings were
observed: pcap disabled, ignored `system()` return values, CMake `PCL_ROOT`
CMP0074 warning, and Boost bind placeholder warning.

## 2026-05-25 Cloud Path Instrumentation

Implemented local Odin driver instrumentation in:

- `src/odin_ros_driver/include/host_sdk_sample.h`
- `src/odin_ros_driver/src/host_sdk_sample.cpp`

Instrumentation points:

- SDK `LIDAR_DT_RAW_DTOF` callback count.
- Valid/invalid DTOF stream and cloud buffer counts.
- `g_cloud_queue` enqueue count.
- `g_cloud_queue` drop count when `CLOUD_QUEUE_MAX_SIZE` is exceeded.
- cloud thread dequeue count.
- `/odin1/cloud_raw` successful publish count.
- invalid publish attempts from `publishIntensityCloud()` early returns.
- current and maximum cloud queue depth.
- max sensor header stamp gap at enqueue and publish.
- max `publishIntensityCloud()` processing time, including conversion and ROS publish call.

Runtime log source:

```text
cloud_path_diag
```

Example fields to inspect:

```text
sdk, valid, enq, deq, pub, drop, window_drop
rates_total sdk/enq/pub
rates_window sdk/enq/deq/pub
queue_depth, max_queue_depth
max_stamp_gap enq/pub
max_publish_processing
```

Interpretation guide:

- `sdk` already low or `max_stamp_gap enq` high: drop or skipped frames before/at SDK callback.
- `sdk` and `enq` good but `drop/window_drop` nonzero: driver cloud queue overflow.
- `enq` good but `deq` or `pub` low: cloud thread conversion/publish path is too slow or blocked.
- driver `pub` stable but FAST-LIVO2 internal `/odin1/cloud_raw` callback low: likely DDS/subscriber delivery or FAST-LIVO2 executor contention.

Verification:

```bash
source install/setup.bash
colcon build --packages-select odin_ros_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build passed. Only existing ignored `system()` return-value warnings were observed.

Short local runtime smoke test:

```bash
source install/setup.bash
ROS_LOG_DIR=/tmp/ros-log timeout 35 ros2 launch odin_ros_driver odin1_fast_livo_ros2.launch.py
```

Result:

- Sandbox run was blocked by DDS/libusb permissions.
- Escalated run started SDK, but USB device `2207:0019` was not attached, so no DTOF callback/cloud diagnostics were produced.
- Next runtime check should be performed with Odin attached.

### NUC Runtime Test With Odin Attached

Synchronized instrumentation source files to:

```text
nuc13@10.56.238.241:/home/nuc13/livo_workspace
```

Files synchronized:

- `src/odin_ros_driver/include/host_sdk_sample.h`
- `src/odin_ros_driver/src/host_sdk_sample.cpp`

Verified NUC build:

```bash
cd /home/nuc13/livo_workspace
source /opt/ros/humble/setup.bash
env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
  colcon build --packages-select odin_ros_driver \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

Build passed. Only existing ignored `system()` return-value warnings were observed.

Initial NUC issue:

- Odin USB device `2207:0019` was attached.
- First launch failed with `LIBUSB_ERROR_BUSY`.
- Cause was an old `host_sdk_sample` process still owning the USB interface.
- Stopped old `host_sdk_sample`/launch processes and reran tests.

Driver-only 75 second test log:

```text
/tmp/odin_cloud_path_diag_20260525_1604.log
```

Driver-only result:

```text
sdk=722 valid=722 enq=722 deq=722 pub=722 drop=0
rates_total sdk/enq/pub=10.263/10.263/10.263 Hz
rates_window sdk/enq/deq/pub=10.259/10.259/10.259/10.259 Hz
queue_depth=0 max_queue_depth=1
max_stamp_gap enq/pub=97.553/97.553 ms
max_publish_processing=1.789 ms
```

Conclusion:

- Odin driver receives, enqueues, dequeues, and publishes raw cloud steadily at about `10.26 Hz`.
- No driver cloud queue overflow or publish-path drop was observed.

Driver + FAST-LIVO2 concurrent test logs:

```text
/tmp/odin_cloud_path_diag_fastlivo_20260525_1606.log
/tmp/fast_livo_with_cloud_diag_20260525_1606.log
/tmp/fast_livo_with_cloud_diag_sigint_20260525_1609.log
/tmp/fast_livo_topic_reports/fast_livo_internal_report_20260525_160839.md
/tmp/fast_livo_topic_reports/fast_livo_internal_report_20260525_160839.json
```

Driver during FAST-LIVO2 result:

```text
sdk=1958 valid=1958 enq=1958 deq=1958 pub=1958 drop=0
rates_total sdk/enq/pub=10.265/10.265/10.265 Hz
rates_window sdk/enq/deq/pub=10.260/10.260/10.260/10.260 Hz
queue_depth=0 max_queue_depth=1
max_stamp_gap enq/pub=97.544/97.544 ms
max_publish_processing=9.960 ms
```

FAST-LIVO2 internal report from the same concurrent run:

```text
Duration: 54.7s
/odin1/imu: 21762 count, 398.481 Hz, header 399.214 Hz
/odin1/cloud_raw: 448 count, 8.186 Hz, header 8.189 Hz
  arrival p95 208.518 ms, p99 297.543 ms
  header p95 194.989 ms, p99 292.460 ms
/odin1/image/undistorted: 561 count, 10.259 Hz, header 10.259 Hz
  arrival p95 113.095 ms, p99 118.803 ms
  header p95 97.516 ms, p99 97.523 ms
```

FAST-LIVO2 logs also showed repeated:

```text
IMU and LiDAR not synced! delta time: about -0.50 to -0.64 s
```

Conclusion from NUC instrumentation:

- Cloud loss is not occurring before driver enqueue, from `g_cloud_queue` overflow, or during driver cloud conversion/publish.
- Driver publishes `/odin1/cloud_raw` continuously at about `10.26 Hz`, even while FAST-LIVO2 is running.
- FAST-LIVO2 receives `/odin1/image/undistorted` at the expected `10.26 Hz`, but receives `/odin1/cloud_raw` at only about `8.19 Hz` with real header gaps around `195 ms` p95.
- The next target is therefore DDS/subscriber delivery or FAST-LIVO2 callback/executor handling for large `PointCloud2`, not the Odin driver cloud receive/queue/publish path.

## 2026-05-25 Cloud Callback Drop Fix

Root cause found:

- Odin driver publishes `/odin1/cloud_raw` with RELIABLE QoS.
- FAST-LIVO2 used `rclcpp::SensorDataQoS()` for lidar subscriptions, which requests BEST_EFFORT reliability.
- FAST-LIVO2 image subscription was already configurable and set to RELIABLE in `odin.yaml`.
- Under FAST-LIVO2 load, large BEST_EFFORT `PointCloud2` samples were dropped before the cloud callback, while image remained stable.

Implemented fix:

- Added FAST-LIVO2 lidar QoS parameters:
  - `common.lidar_qos_reliable`
  - `common.lidar_queue_size`
- `initializeSubscribersAndPublishers()` now builds a dedicated lidar QoS profile instead of using `SensorDataQoS()` for lidar.
- Odin config now sets:

```yaml
common:
  lidar_qos_reliable: true
  lidar_queue_size: 50
```

Changed files:

- `src/FAST-LIVO2/include/LIVMapper.h`
- `src/FAST-LIVO2/src/LIVMapper.cpp`
- `src/FAST-LIVO2/config/odin.yaml`

Local build verification:

```bash
source install/setup.bash
colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build passed. Only existing PCL CMake warning was observed.

NUC build verification:

```bash
cd /home/nuc13/livo_workspace
source /opt/ros/humble/setup.bash
env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
  colcon build --packages-select fast_livo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

Build passed.

NUC validation logs:

```text
/tmp/odin_cloud_path_diag_reliable_20260525.log
/tmp/fast_livo_reliable_20260525.log
/tmp/fast_livo_topic_reports/fast_livo_internal_report_20260525_161722.md
/tmp/fast_livo_topic_reports/fast_livo_internal_report_20260525_161722.json
```

Driver during fixed FAST-LIVO2 run:

```text
sdk=825 valid=825 enq=825 deq=825 pub=825 drop=0
rates_total sdk/enq/pub=10.267/10.267/10.267 Hz
rates_window sdk/enq/deq/pub=10.269/10.269/10.269/10.269 Hz
max_stamp_gap enq/pub=97.549/97.549 ms
```

FAST-LIVO2 internal report after reliable lidar QoS:

```text
Duration: 54.7s
/odin1/imu: 21650 count, 399.299 Hz, header 399.316 Hz
/odin1/cloud_raw: 561 count, 10.334 Hz, header 10.259 Hz
  arrival p95 115.788 ms, p99 127.275 ms
  header p95 97.520 ms, p99 97.525 ms
/odin1/image/undistorted: 556 count, 10.259 Hz, header 10.259 Hz
  arrival p95 113.449 ms, p99 118.532 ms
  header p95 97.520 ms, p99 97.525 ms
```

Conclusion:

- Reliable lidar QoS fixes the FAST-LIVO2 cloud callback drop.
- `/odin1/cloud_raw` callback frequency now matches the driver and image cadence.
- The previous `~195 ms` cloud header p95 gap is gone; cloud header p95 is now about `97.52 ms`.

## 2026-05-25 Runtime Log Noise Reduction

Implemented after cloud QoS stabilization.

FAST-LIVO2 changes:

- Added `common.verbose` config parameter.
- Added a shared `g_fast_livo_verbose` flag in `common_lib.h`.
- Odin config sets:

```yaml
common:
  verbose: false
```

With `verbose: false`, the following high-frequency frame logs are suppressed:

- `Get image, its header time`
- `Get LiDAR, its header time`
- `[ LIO ] Raw feature num`
- `[ VIO ] Raw feature num`
- `[ LIO ] Update Voxel Map`
- `[ LIO ]: No point!!!`
- `[ VIO ] No point!!!`
- VIO/LIO timing tables
- visual sparse map retrieve/append/update counts

Odin driver change:

- Removed periodic `cloud_path_diag` emission from the cloud thread.
- `cloud_path_diag` final summary remains available at cloud thread shutdown.
- Downgraded `Software connection successful...` and `Device ready and streams activated`
  from INFO to DEBUG.

Changed files:

- `src/FAST-LIVO2/config/odin.yaml`
- `src/FAST-LIVO2/include/common_lib.h`
- `src/FAST-LIVO2/include/LIVMapper.h`
- `src/FAST-LIVO2/src/LIVMapper.cpp`
- `src/FAST-LIVO2/src/vio.cpp`
- `src/FAST-LIVO2/src/voxel_map.cpp`
- `src/odin_ros_driver/src/host_sdk_sample.cpp`

Local build verification:

```bash
source install/setup.bash
colcon build --packages-select odin_ros_driver fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build passed. Only existing warnings were observed.

NUC build verification:

```bash
cd /home/nuc13/livo_workspace
source /opt/ros/humble/setup.bash
env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
  colcon build --packages-select odin_ros_driver fast_livo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

Build passed.

NUC short quiet-run check:

```text
/tmp/odin_quiet_test_20260525.log
/tmp/fast_livo_quiet_test_20260525.log
/tmp/fast_livo_topic_reports/fast_livo_internal_report_20260525_163659.md
```

Result:

- No `cloud_path_diag` periodic lines appeared in the Odin driver log.
- No FAST-LIVO2 `Get image`, LIO/VIO timing table, raw feature count, or visual
  map retrieve/append/update frame logs appeared.
- 30 second FAST-LIVO2 log was reduced to 22 lines, mostly launch, initialization,
  shutdown, and final save/report messages.

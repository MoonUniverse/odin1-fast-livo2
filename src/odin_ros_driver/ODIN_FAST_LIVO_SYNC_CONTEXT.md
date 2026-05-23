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

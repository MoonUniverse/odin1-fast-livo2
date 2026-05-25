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
- No recorddata directories were left after the final tests.
- Earlier full-launch test generated `recorddata/20260522_095011`, total `6.8G`; it was deleted after measurement.

## Commit and NUC Deployment

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

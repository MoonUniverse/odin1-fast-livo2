# FAST-LIVO2 ROS2 Humble Migration Notes

This document records the ROS1 to ROS2 Humble migration work done in this workspace. It is intended for developers and for future Codex sessions that need to resume from the current state.

## Current State

- `FAST-LIVO2` has been migrated to ROS2-only using `ament_cmake` and `rclcpp`.
- Core algorithm files and behavior were kept intact as much as possible; changes are concentrated around ROS integration, build metadata, parameters, messages, launch, RViz, and bag testing.
- The workspace now includes two support packages:
  - `livox_ros_driver2`: minimal ROS2 message interface package for `CustomMsg` and `CustomPoint`.
  - `vikit_common`: minimal ament/header package providing the vikit/Sophus interfaces used by FAST-LIVO2.
- ROS1 generated Livox headers under `FAST-LIVO2/include/livox_ros_driver/` were removed.
- ROS1 XML launch files were replaced with ROS2 Python launch files.
- RViz configs were migrated from ROS1 plugin class names to ROS2 Humble plugin class names.

## Important Files

- `FAST-LIVO2/CMakeLists.txt`: ROS2/ament build configuration.
- `FAST-LIVO2/package.xml`: ROS2 package metadata.
- `FAST-LIVO2/include/ros2_utils.h`: ROS2 compatibility helpers for message aliases, timestamp conversion, and logging macros.
- `FAST-LIVO2/src/LIVMapper.cpp`: main ROS2 node integration layer.
- `FAST-LIVO2/launch/*.launch.py`: ROS2 launch files.
- `FAST-LIVO2/rviz_cfg/*.rviz`: RViz2-compatible configs.
- `FAST-LIVO2/scripts/convert_fast_livo_ros1_bag.py`: targeted ROS1 bag to ROS2 bag converter for FAST-LIVO2 test data.
- `livox_ros_driver2/msg/CustomMsg.msg` and `CustomPoint.msg`: Livox message definitions.
- `vikit_common/include/vikit/*` and `vikit_common/include/sophus/se3.h`: minimal algorithm dependency interfaces.

## Build

Use system Python, not conda Python, because ROS2 Humble code generation depends on system Python packages such as `catkin_pkg` and `numpy`.

```bash
cd /home/alienware/livo_workspace
source /opt/ros/humble/setup.bash
env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
  colcon build --packages-select livox_ros_driver2 vikit_common fast_livo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Known harmless build warning:

- CMake may warn about `PCL_ROOT` and CMP0074. The build still succeeds.

## Launch

Available launch files:

```bash
ros2 launch fast_livo mapping_avia.launch.py rviz:=false
ros2 launch fast_livo mapping_avia_cbd02.launch.py rviz:=false
ros2 launch fast_livo mapping_avia_marslvig.launch.py rviz:=false
ros2 launch fast_livo mapping_hesaixt32_hilti22.launch.py rviz:=false
ros2 launch fast_livo mapping_ouster_ntu.launch.py rviz:=false
```

RViz:

```bash
ros2 launch fast_livo mapping_avia.launch.py rviz:=true
```

The Avia launch files have an optional compressed image republisher:

```bash
ros2 launch fast_livo mapping_avia.launch.py rviz:=false republish:=true
```

Default `republish` is `false` because this environment only has raw `image_transport`; `compressed_sub` was not installed.

## RViz Migration Notes

The original `.rviz` files used ROS1 plugin class names such as:

- `rviz/Grid`
- `rviz/PointCloud2`
- `rviz/Odometry`
- `rviz/Path`
- `rviz/MarkerArray`

They were migrated to ROS2 Humble equivalents:

- `rviz_common/Displays`, `rviz_common/Selection`, `rviz_common/Tool Properties`, `rviz_common/Views`, `rviz_common/Time`
- `rviz_default_plugins/Grid`, `Axes`, `PointCloud2`, `Odometry`, `Path`, `Marker`, `MarkerArray`, `Image`, tools, and view controllers
- Display groups were changed to `rviz_common/Group`

Static check:

```bash
rg -n "Class: rviz/" FAST-LIVO2/rviz_cfg install/fast_livo/share/fast_livo/rviz_cfg
```

Expected result: no matches.

In the sandbox, RViz2 may still fail with X/Qt errors such as `could not connect to display` or Ogre `Couldn’t open X display`. That is a GUI environment limitation, not a config issue.

## ROS1 Bag Test Data

Test file:

```text
/home/alienware/livo_workspace/Bright_Screen_Wall.bag
```

It is a ROS1 bag v2 file containing:

- `/livox/imu`: `sensor_msgs/Imu`
- `/livox/lidar`: `livox_ros_driver/CustomMsg`
- `/left_camera/image/compressed`: `sensor_msgs/CompressedImage`

ROS2 cannot play it directly:

```bash
ros2 bag info Bright_Screen_Wall.bag
```

This fails because it is a ROS1 `.bag`, not a ROS2 rosbag2 storage.

## ROS1 Bag Conversion

A targeted converter was added:

```bash
source install/setup.bash
/usr/bin/python3 FAST-LIVO2/scripts/convert_fast_livo_ros1_bag.py \
  Bright_Screen_Wall.bag /tmp/bright_screen_wall_ros2_8s \
  --duration 8 --overwrite
```

The converter:

- Reads uncompressed ROS1 bag chunks.
- Converts `sensor_msgs/Imu` to ROS2 `sensor_msgs/msg/Imu`.
- Converts `livox_ros_driver/CustomMsg` to ROS2 `livox_ros_driver2/msg/CustomMsg`.
- Converts raw `/left_camera/image` ROS1 `sensor_msgs/Image` to ROS2 `sensor_msgs/msg/Image`.
- Decodes `/left_camera/image/compressed` JPEG/PNG data with OpenCV and writes raw ROS2 `sensor_msgs/msg/Image` on `/left_camera/image`.
- Supports `--raw-image-encoding bgr8|mono8`; default is `bgr8`. `mono8` reduces raw image message size for playback tests, while FAST-LIVO can still convert it to BGR internally through `cv_bridge`.

Verified 8-second conversion output:

```text
/livox/imu: 1640 messages
/livox/lidar: 81 messages
/left_camera/image: 81 messages
```

Check converted bag:

```bash
ros2 bag info /tmp/bright_screen_wall_ros2_8s
```

## Runtime Test With Converted Bag

Command pattern used:

```bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros-log
export ROS_LOCALHOST_ONLY=1

(timeout 18 ros2 launch fast_livo mapping_avia.launch.py rviz:=false \
  > /tmp/fast_livo_bag_test_node.log 2>&1) &
node_pid=$!

sleep 4
timeout 10 ros2 bag play /tmp/bright_screen_wall_ros2_8s --read-ahead-queue-size 1000 \
  > /tmp/fast_livo_bag_test_play.log 2>&1 || true

wait ${node_pid} || true
tail -120 /tmp/fast_livo_bag_test_node.log
```

Observed successful node behavior:

- `Get LiDAR`
- `Get image`
- `[ Preprocess ] Input point number`
- `[ VIO ] Retrieve ... points from visual sparse map`
- `[ VIO ] Append ... new visual map points`
- `[ LIO ] Raw feature num ...`
- `[ LIO ] Update Voxel Map`

This confirms the ROS2 port receives converted LiDAR, IMU, and image data and enters both VIO and LIO processing loops.

## Dataset Calibration Notes

The official calibration file contains multiple calibration groups. The default
`mapping_avia.launch.py` uses the group for:

- `Retail_Street`
- `CBD_Building_01`
- `Bright_Screen_Wall`

`CBD_Building_02` and `CBD_Building_03` use a different camera intrinsic and
LiDAR-camera extrinsic calibration. Running `CBD_Building_02` with the default
`mapping_avia.launch.py` can make LIO look cleaner than LIVO and cause
accumulated point-cloud ghosting in RViz.

Use the dedicated launch file:

```bash
ros2 launch fast_livo mapping_avia_cbd02.launch.py rviz:=true
```

The dedicated files are:

- `FAST-LIVO2/config/avia_cbd02.yaml`
- `FAST-LIVO2/config/camera_pinhole_cbd02.yaml`
- `FAST-LIVO2/launch/mapping_avia_cbd02.launch.py`

The CBD02/CBD03 calibration differs significantly from the default Avia group:

- focal length differs by about 9.1 percent
- principal point differs by about 35 px in x and 14 px in y
- LiDAR-camera translation differs by about 6 cm
- LiDAR-camera rotation differs by about 1.25 degrees

## Image Playback QoS Note

The converted raw `/left_camera/image` topic is large. In the 10-second test bag:

- `/left_camera/image` is about 3.75 MiB per `bgr8` frame, 101 frames total.
- `/livox/lidar` is about 0.46 MiB per frame, 101 frames total.

`ros2 topic hz /left_camera/image` may report less than 10 Hz even when the bag timestamps are correct, because the CLI subscriber must deserialize large `sensor_msgs/Image` messages and its subscription QoS/queue is not suitable as a lossless image-rate validator.

FAST-LIVO's image subscription now uses a separate configurable QoS:

```yaml
common:
  img_qos_reliable: true
  img_queue_size: 200
```

With the original `Bright_Screen_Wall_Ros2` raw image bag, the node-side count improved from about `76/101` received image callbacks with best-effort image QoS to `101/101` received image callbacks with reliable image QoS.

For real camera drivers that only offer best-effort image QoS, set:

```yaml
common:
  img_qos_reliable: false
```

## Odin IMU Calibration

Two helper scripts are included for static IMU noise calibration:

- `FAST-LIVO2/scripts/record_imu_static.py`: records `/odin1/imu` as text with
  columns `t gx gy gz ax ay az`.
- `FAST-LIVO2/scripts/imu_calibrate.py`: runs Allan variance analysis and prints
  FAST-LIVO2 `imu` config values.

For calibration, start the Odin driver in IMU-only mode:

```bash
ros2 launch odin_ros_driver odin1_imu_only_ros2.launch.py
```

This launch uses `control_command_imu_only.yaml`, where RGB, DTOF cloud, odom,
cloud_slam, cloud_render, recorddata, devstatus logging, and image outputs are
disabled. The driver also creates ROS publishers according to these switches,
so only `/odin1/imu` should be advertised and publishing.

In another terminal, record with:

```bash
ros2 run fast_livo record_imu_static.py \
  --topic /odin1/imu \
  --duration 7200 \
  --output /tmp/odin_imu_static.txt
```

Keep the Odin sensor still on a stable surface during recording. A short
recording can estimate white noise (`acc_cov`, `gyr_cov`), but bias random walk
(`b_acc_cov`, `b_gyr_cov`) usually needs one to two hours or longer.

Run calibration:

```bash
ros2 run fast_livo imu_calibrate.py \
  --input /tmp/odin_imu_static.txt \
  --plot \
  --plot-output /tmp/odin_allan_deviation.png
```

To update the Odin FAST-LIVO2 config directly:

```bash
ros2 run fast_livo imu_calibrate.py \
  --input /tmp/odin_imu_static.txt \
  --update-config /home/alienware/livo_workspace/src/FAST-LIVO2/config/odin.yaml
```

The calibration script only averages bias random walk values from axes where
the Allan curve contains a resolved `+1/2` slope region. Unresolved axes are
reported as `unresolved` and are not mixed into the mean as zero.

## Odin NUC Deployment State

Current Odin/FAST-LIVO2 functional commit:

```text
1d21ef3 Add Odin IMU calibration workflow
```

It was synchronized to:

```text
nuc13@10.56.238.242:/home/nuc13/livo_workspace
```

The NUC build was verified with:

```bash
cd /home/nuc13/livo_workspace
source /opt/ros/humble/setup.bash
env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
  colcon build --packages-select livox_ros_driver2 vikit_common odin_ros_driver fast_livo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

Build result: `livox_ros_driver2`, `vikit_common`, `odin_ros_driver`, and
`fast_livo` all built successfully. Only existing warnings were observed.

Normal Odin + FAST-LIVO2 runtime on the NUC:

```bash
cd /home/nuc13/livo_workspace
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros-log
ros2 launch odin_ros_driver odin1_fast_livo_ros2.launch.py
```

```bash
cd /home/nuc13/livo_workspace
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros-log
ros2 launch fast_livo mapping_odin.launch.py rviz:=false
```

IMU-only calibration runtime on the NUC:

```bash
cd /home/nuc13/livo_workspace
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros-log
ros2 launch odin_ros_driver odin1_imu_only_ros2.launch.py
```

```bash
cd /home/nuc13/livo_workspace
source install/setup.bash
ros2 run fast_livo record_imu_static.py \
  --topic /odin1/imu \
  --duration 7200 \
  --output /tmp/odin_imu_static_2h.txt
```

## Odin FAST-LIVO2 Config Check

Checked on 2026-05-25:

- `mapping_odin.launch.py` loads `config/odin.yaml` and `config/camera_odin.yaml`.
- Source and installed copies of `odin.yaml`, `camera_odin.yaml`, and
  `mapping_odin.launch.py` were identical.
- The Odin topics are `/odin1/imu`, `/odin1/cloud_raw`, and
  `/odin1/image/undistorted`.
- `preprocess.lidar_type: 8` selects the Odin point cloud handler.
- `camera_odin.yaml` uses `1600x1296`, matching the current undistorted image.
- `img_time_offset: 0.001685342` remains consistent with the measured
  image/cloud nearest-neighbor offset.
- No obvious Odin FAST-LIVO2 config error was found.

## Environment Limitations Seen Here

The sandbox/workspace environment has restrictions that are not code defects:

- ROS2 launch initially failed writing logs to `~/.ros/log`; use `ROS_LOG_DIR=/tmp/ros-log`.
- FastDDS reports socket/interface errors like `getifaddrs: Operation not permitted` and `Error creating socket: Operation not permitted`.
- RViz2 cannot open a display in the sandbox.
- Conda Python caused ROS2 build/codegen failures:
  - missing `catkin_pkg`
  - missing `numpy`

Use system Python and a normal ROS2 desktop environment for final validation.

## Known Gaps / Follow-Up Work

- The included `livox_ros_driver2` package is message-only. It is not a device driver. For real hardware, replace or overlay it with the official Livox ROS2 driver package if needed.
- The included `vikit_common` is a minimal implementation of only the interfaces used by FAST-LIVO2. It is not a full upstream vikit port.
- The ROS1 bag converter is intentionally narrow. It supports this FAST-LIVO2 test bag shape and uncompressed ROS1 chunks.
- Full dataset conversion can be done by omitting `--duration`, but it may take longer and produce a large ROS2 bag.
- For compressed image transport at runtime, install ROS2 compressed transport plugins or keep using the converter to publish raw `/left_camera/image`.

## Quick Resume Checklist

1. Source ROS2 and build:

   ```bash
   source /opt/ros/humble/setup.bash
   env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
     colcon build --packages-select livox_ros_driver2 vikit_common fast_livo \
     --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
   source install/setup.bash
   ```

2. Convert a short test bag:

   ```bash
   /usr/bin/python3 FAST-LIVO2/scripts/convert_fast_livo_ros1_bag.py \
     Bright_Screen_Wall.bag /tmp/bright_screen_wall_ros2_8s \
     --duration 8 --overwrite
   ```

3. Run node and play converted bag:

   ```bash
   ROS_LOG_DIR=/tmp/ros-log ros2 launch fast_livo mapping_avia.launch.py rviz:=false
   ros2 bag play /tmp/bright_screen_wall_ros2_8s
   ```

4. Check for processing logs:

   ```text
   [ VIO ] ...
   [ LIO ] ...
   ```

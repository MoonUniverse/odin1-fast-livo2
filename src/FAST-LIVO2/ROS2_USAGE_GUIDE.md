# FAST-LIVO2 ROS2 Usage Guide

This guide records the working ROS2 Humble setup in this workspace, including
build commands, ROS1 bag conversion, DDS settings, RViz usage, and dataset
calibration notes.

## Workspace

```bash
cd /home/alienware/livo_workspace
```

Packages used by this port:

- `FAST-LIVO2`: ROS2 port of FAST-LIVO2, package name `fast_livo`.
- `livox_ros_driver2`: message-only package for `CustomMsg` and `CustomPoint`.
- `vikit_common`: minimal ament/header package for FAST-LIVO2 dependencies.

Use system Python, not conda Python.

## Build

```bash
source /opt/ros/humble/setup.bash

env PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin \
  colcon build --packages-select livox_ros_driver2 vikit_common fast_livo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

Known harmless warning:

- CMake may warn about `PCL_ROOT` and CMP0074.

## Recommended ROS Environment

FastDDS tested better than CycloneDDS for this dataset workflow, especially for
LiDAR reception stability.

Use the same environment in every terminal:

```bash
cd /home/alienware/livo_workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=33
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/ros-log
```

If topics are not visible between terminals, restart the ROS daemon after
setting the same environment:

```bash
ros2 daemon stop
ros2 daemon start
```

Check DDS:

```bash
ros2 doctor --report | grep -A3 "RMW MIDDLEWARE"
```

Expected:

```text
middleware name    : rmw_fastrtps_cpp
```

## Launch Files

General Avia datasets using the first official calibration group:

```bash
ros2 launch fast_livo mapping_avia.launch.py rviz:=true
```

CBD Building 02 and CBD Building 03:

```bash
ros2 launch fast_livo mapping_avia_cbd02.launch.py rviz:=true
```

MARS LVIG datasets:

```bash
ros2 launch fast_livo mapping_avia_marslvig.launch.py rviz:=true
```

Set `rviz:=false` for headless testing.

## ROS1 Bag Conversion

The converter supports this FAST-LIVO2 dataset shape:

- `/livox/imu`: `sensor_msgs/Imu`
- `/livox/lidar`: `livox_ros_driver/CustomMsg`
- `/left_camera/image`: raw ROS1 `sensor_msgs/Image`
- `/left_camera/image/compressed`: compressed ROS1 `sensor_msgs/CompressedImage`

Default conversion duration is 10 seconds.

Convert a 30 second test bag:

```bash
/usr/bin/python3 FAST-LIVO2/scripts/convert_fast_livo_ros1_bag.py \
  CBD_Building_02.bag CBD_Building_02_ROS2 \
  --duration 30 --overwrite
```

Convert the full bag:

```bash
/usr/bin/python3 FAST-LIVO2/scripts/convert_fast_livo_ros1_bag.py \
  CBD_Building_02.bag CBD_Building_02_ROS2_full \
  --duration 0 --overwrite
```

For compressed image source bags, the converter decodes to raw
`sensor_msgs/msg/Image`. The default output encoding is `bgr8`. You can reduce
raw image size with:

```bash
/usr/bin/python3 FAST-LIVO2/scripts/convert_fast_livo_ros1_bag.py \
  Bright_Screen_Wall.bag Bright_Screen_Wall_Ros2_mono \
  --raw-image-encoding mono8 --overwrite
```

Check output:

```bash
ros2 bag info CBD_Building_02_ROS2
```

Expected for the 30 second CBD02 test bag:

```text
/left_camera/image: 300 messages, about 10 Hz
/livox/lidar:       301 messages, about 10 Hz
/livox/imu:         6116 messages, about 204 Hz
```

## Play Bag

Terminal 1:

```bash
ros2 launch fast_livo mapping_avia_cbd02.launch.py rviz:=true
```

Terminal 2:

```bash
ros2 bag play CBD_Building_02_ROS2 --read-ahead-queue-size 1000
```

Expected node logs:

```text
Get LiDAR
Get image
[ Preprocess ] Input point number
[ VIO ] Retrieve ...
[ VIO ] Append ...
[ LIO ] Raw feature num ...
[ LIO ] Update Voxel Map
```

## Dataset Calibration Groups

The official calibration file contains multiple calibration groups. Do not use
the first Avia calibration for all datasets.

### Group 1: Retail Street, CBD Building 01, Bright Screen Wall

Use:

```bash
ros2 launch fast_livo mapping_avia.launch.py rviz:=true
```

Important values:

```yaml
Rcl: [0.00610193,-0.999863,-0.0154172,
      -0.00615449,0.0153796,-0.999863,
       0.999962,0.00619598,-0.0060598]
Pcl: [0.0194384, 0.104689, -0.0251952]

cam_fx: 1293.56944
cam_fy: 1293.3155
cam_cx: 626.91359
cam_cy: 522.799224
```

### Group 2: CBD Building 02, CBD Building 03 and listed HKU sequences

Use:

```bash
ros2 launch fast_livo mapping_avia_cbd02.launch.py rviz:=true
```

Important values:

```yaml
Rcl: [-0.00200, -0.99975, -0.02211,
      -0.00366, 0.02212, -0.99975,
       0.99999, -0.00192, -0.00371]
Pcl: [0.00260, 0.05057, -0.00587]

cam_fx: 1176.2874292149932
cam_fy: 1176.21585445307
cam_cx: 592.1187382755453
cam_cy: 509.0864309628322
```

The difference is large enough to affect LIVO strongly:

- Focal length differs by about 9.1 percent.
- Principal point differs by about 35 px in x and 14 px in y.
- LiDAR-camera translation differs by about 6 cm.
- LiDAR-camera rotation differs by about 1.25 degrees.

Symptom when using the wrong calibration:

- LIO-only map looks cleaner.
- LIVO map becomes blurred or duplicated in RViz.
- `surround` history accumulation shows clear ghosting.

## LIO-Only Debug

Use this to determine whether the problem is in LiDAR-IMU or in visual fusion.

```bash
ros2 run fast_livo fastlivo_mapping \
  --ros-args \
  --params-file install/fast_livo/share/fast_livo/config/avia_cbd02.yaml \
  --params-file install/fast_livo/share/fast_livo/config/camera_pinhole_cbd02.yaml \
  -p common.img_en:=0
```

Then play the same bag:

```bash
ros2 bag play CBD_Building_02_ROS2 --read-ahead-queue-size 1000
```

Interpretation:

- LIO-only is clear, LIVO is blurred: check camera calibration, LiDAR-camera
  extrinsic, and image time offset.
- LIO-only is also blurred: check LiDAR, IMU, preprocessing, and IMU timing.

## RViz Notes

The RViz config includes both current-frame and accumulated point cloud views:

- `currPoints`: `/cloud_registered`, `Decay Time: 0`
- `surround`: `/cloud_registered`, long decay time for accumulated history

If pose is accurate, `surround` should build a clear accumulated map. If
`surround` shows duplicated walls or heavy ghosting while LIO-only is clean, the
visual fusion calibration is likely wrong.

## Image QoS

Raw image frames are large. For 1280x1024 `bgr8` images:

- About 3.75 MiB per frame.
- About 37.5 MiB/s at 10 Hz.

FAST-LIVO image subscription uses a separate configurable QoS:

```yaml
common:
  img_qos_reliable: true
  img_queue_size: 200
```

This fixed image drops during rosbag playback. In testing,
`Bright_Screen_Wall_Ros2` image callbacks improved from about `76/101` to
`101/101`.

For real camera drivers that only publish best-effort images, set:

```yaml
common:
  img_qos_reliable: false
```

## Odin IMU Calibration

Start the Odin driver in IMU-only mode:

```bash
ros2 launch odin_ros_driver odin1_imu_only_ros2.launch.py
```

This disables RGB, DTOF cloud, odom, cloud_slam, cloud_render, recorddata,
devstatus logging, and image outputs. The driver also creates ROS publishers
according to these switches, so only `/odin1/imu` should be advertised and
publishing.

Record static Odin IMU data in another terminal while the device is motionless:

```bash
ros2 run fast_livo record_imu_static.py \
  --topic /odin1/imu \
  --duration 7200 \
  --output /tmp/odin_imu_static.txt
```

Run Allan variance calibration:

```bash
ros2 run fast_livo imu_calibrate.py \
  --input /tmp/odin_imu_static.txt \
  --plot \
  --plot-output /tmp/odin_allan_deviation.png
```

Apply resolved parameters to `config/odin.yaml`:

```bash
ros2 run fast_livo imu_calibrate.py \
  --input /tmp/odin_imu_static.txt \
  --update-config /home/alienware/livo_workspace/src/FAST-LIVO2/config/odin.yaml
```

Short recordings can give usable `acc_cov` and `gyr_cov`; use one to two hours
or longer for reliable `b_acc_cov` and `b_gyr_cov`. If a bias random walk axis
is unresolved, the calibration script reports it as `unresolved` and excludes
that axis from the mean instead of averaging in zero.

## Odin NUC Commands

The latest synchronized NUC workspace is:

```text
nuc13@10.56.238.241:/home/nuc13/livo_workspace
```

Functional commit:

```text
1d21ef3 Add Odin IMU calibration workflow
```

Normal runtime:

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

IMU-only runtime:

```bash
cd /home/nuc13/livo_workspace
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros-log
ros2 launch odin_ros_driver odin1_imu_only_ros2.launch.py
```

## DDS Comparison Summary

Tested with `Bright_Screen_Wall_Ros2`:

| Metric | FastDDS | CycloneDDS |
| --- | ---: | ---: |
| Full bag playback wall time | 10.272 s | 10.168 s |
| `/livox/lidar` CLI hz | 9.984 Hz | 5.876 Hz |
| `/livox/imu` CLI hz | 83.182 Hz | 95.614 Hz |
| FAST-LIVO image callbacks | 101/101 | 101/101 |
| FAST-LIVO LiDAR callbacks | 99/101 | 86/101, then 95/101 in repeats |

Recommendation for this workspace:

```text
Use FastDDS by default.
```

`ros2 topic hz /left_camera/image` is not a reliable final metric for large raw
images because the CLI subscriber itself can become the bottleneck.

## Troubleshooting

### Other terminals cannot see bag topics

Make sure every terminal uses the same environment:

```bash
echo RMW=$RMW_IMPLEMENTATION
echo DOMAIN=$ROS_DOMAIN_ID
echo LOCALHOST=$ROS_LOCALHOST_ONLY
```

All should match, for example:

```text
RMW=rmw_fastrtps_cpp
DOMAIN=33
LOCALHOST=1
```

Also source the workspace:

```bash
source /home/alienware/livo_workspace/install/setup.bash
```

Without it, `ros2 bag play` may ignore `/livox/lidar` because
`livox_ros_driver2` is not found.

### Converted image count is zero

Older converter versions only handled `/left_camera/image/compressed`.
`CBD_Building_02.bag` uses raw `/left_camera/image`. The current converter
supports both raw and compressed image topics.

### LIVO is worse than LIO

Use the calibration-specific launch file. For CBD02/CBD03, use:

```bash
ros2 launch fast_livo mapping_avia_cbd02.launch.py rviz:=true
```

Do not use:

```bash
ros2 launch fast_livo mapping_avia.launch.py rviz:=true
```

for CBD02/CBD03.

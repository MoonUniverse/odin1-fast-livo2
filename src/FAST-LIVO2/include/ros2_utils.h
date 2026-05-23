#ifndef FAST_LIVO_ROS2_UTILS_H_
#define FAST_LIVO_ROS2_UTILS_H_

#include <cassert>
#include <cmath>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace sensor_msgs
{
using Image = msg::Image;
using Imu = msg::Imu;
using PointCloud2 = msg::PointCloud2;
using ImageConstPtr = msg::Image::ConstSharedPtr;
} // namespace sensor_msgs

namespace nav_msgs
{
using Odometry = msg::Odometry;
using Path = msg::Path;
} // namespace nav_msgs

namespace geometry_msgs
{
using PoseStamped = msg::PoseStamped;
using Quaternion = msg::Quaternion;
using TransformStamped = msg::TransformStamped;
} // namespace geometry_msgs

namespace visualization_msgs
{
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
} // namespace visualization_msgs

namespace std_msgs
{
using Header = msg::Header;
} // namespace std_msgs

namespace livox_ros_driver = livox_ros_driver2::msg;

namespace fast_livo
{

inline double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
  return rclcpp::Time(stamp).seconds();
}

inline builtin_interfaces::msg::Time stampFromSec(double seconds)
{
  const auto nanoseconds = static_cast<int64_t>(std::llround(seconds * 1e9));
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000);
  return stamp;
}

inline builtin_interfaces::msg::Time timeToMsg(const rclcpp::Time &time)
{
  return stampFromSec(time.seconds());
}

inline rclcpp::Logger logger()
{
  return rclcpp::get_logger("fast_livo");
}

} // namespace fast_livo

#ifndef ROS_INFO
#define ROS_INFO(...) RCLCPP_INFO(::fast_livo::logger(), __VA_ARGS__)
#endif
#ifndef ROS_WARN
#define ROS_WARN(...) RCLCPP_WARN(::fast_livo::logger(), __VA_ARGS__)
#endif
#ifndef ROS_ERROR
#define ROS_ERROR(...) RCLCPP_ERROR(::fast_livo::logger(), __VA_ARGS__)
#endif
#ifndef ROS_ASSERT
#define ROS_ASSERT(condition) assert(condition)
#endif

#endif // FAST_LIVO_ROS2_UTILS_H_

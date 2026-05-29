#include "odin_direct_sdk.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <yaml-cpp/yaml.h>

#include "polynomial_camera.hpp"
#include "yaml_parser.h"

namespace odin_ros_driver
{
namespace
{
constexpr size_t kRawImuMax = 1000;
constexpr size_t kRawCloudMax = 20;
constexpr size_t kRawImageMax = 15;
constexpr size_t kOutImuMax = 2000;
constexpr size_t kOutCloudMax = 30;
constexpr size_t kOutImageMax = 30;
constexpr int kDtofRowsPerGroup = 6;

std::mutex g_active_mutex;
OdinDirectSdk *g_active = nullptr;

template <typename QueueT>
void dropOldestIfFull(QueueT &queue, size_t max_size, std::atomic<uint64_t> &drop_counter)
{
  if (queue.size() >= max_size)
  {
    queue.pop();
    drop_counter.fetch_add(1, std::memory_order_relaxed);
  }
}

template <typename T>
void appendPod(std::vector<uint8_t> &blob, const T &value)
{
  const auto *p = reinterpret_cast<const uint8_t *>(&value);
  blob.insert(blob.end(), p, p + sizeof(T));
}

std::string defaultConfigFile()
{
  return (std::filesystem::current_path() / "src" / "odin_ros_driver" / "config" / "control_command_fast_livo.yaml").string();
}

std::filesystem::path defaultRecorddataDir()
{
  const char *home = std::getenv("HOME");
  if (home && *home)
  {
    return std::filesystem::path(home) / "OdinData";
  }
  return std::filesystem::current_path() / "OdinData";
}

std::string formatDouble(double value)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << value;
  return ss.str();
}

} // namespace

OdinDirectSdk::OdinDirectSdk(rclcpp::Node *node) : node_(node) {}

OdinDirectSdk::~OdinDirectSdk()
{
  stop();
}

bool OdinDirectSdk::start(const OdinDirectOptions &options, OdinDirectCallbacks callbacks)
{
  if (started_.exchange(true, std::memory_order_acq_rel))
  {
    return true;
  }

  options_ = options;
  callbacks_ = std::move(callbacks);
  if (options_.config_file.empty())
  {
    options_.config_file = defaultConfigFile();
  }
  if (options_.recorddata_dir.empty())
  {
    options_.recorddata_dir = defaultRecorddataDir().string();
  }

  if (!loadConfig())
  {
    started_.store(false, std::memory_order_release);
    return false;
  }

  initPublishers();
  initRecorddata();

  {
    std::lock_guard<std::mutex> lock(g_active_mutex);
    g_active = this;
  }

  if (lidar_system_init(&OdinDirectSdk::deviceCallbackStatic) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Odin direct SDK: lidar_system_init failed");
    std::lock_guard<std::mutex> lock(g_active_mutex);
    if (g_active == this)
    {
      g_active = nullptr;
    }
    started_.store(false, std::memory_order_release);
    return false;
  }

  if (enable_imu_smooth_)
  {
    lidar_enable_imu_smooth_sending(1);
    lidar_set_imu_smooth_frequency(static_cast<uint32_t>(imu_smooth_frequency_));
  }
  else
  {
    lidar_enable_imu_smooth_sending(0);
  }

  startWorkers();
  RCLCPP_INFO(node_->get_logger(), "Odin direct SDK started with config: %s", options_.config_file.c_str());
  return true;
}

void OdinDirectSdk::stop()
{
  if (!started_.exchange(false, std::memory_order_acq_rel))
  {
    return;
  }

  stopWorkers();

  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    if (device_)
    {
      lidar_unregister_stream_callback(device_);
      lidar_stop_stream(device_, LIDAR_MODE_SLAM);
      lidar_close_device(device_);
      lidar_destory_device(device_);
      device_ = nullptr;
    }
  }
  device_connected_.store(false, std::memory_order_release);
  lidar_system_deinit();

  if (data_logger_)
  {
    writeCamInEx();
    RCLCPP_INFO(node_->get_logger(), "Odin direct SDK flushing recorddata at %s", recorddata_root_.c_str());
    data_logger_.reset();
  }

  {
    std::lock_guard<std::mutex> lock(g_active_mutex);
    if (g_active == this)
    {
      g_active = nullptr;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Odin direct SDK stopped");
}

void OdinDirectSdk::spinSome(size_t max_imu, size_t max_cloud, size_t max_image)
{
  for (size_t i = 0; i < max_cloud; ++i)
  {
    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (cloud_queue_.empty())
      {
        break;
      }
      msg = cloud_queue_.front();
      cloud_queue_.pop();
    }
    if (callbacks_.cloud)
    {
      callbacks_.cloud(msg);
    }
    if (options_.publish_debug_topics && cloud_pub_)
    {
      cloud_pub_->publish(*msg);
    }
    delivered_cloud_.fetch_add(1, std::memory_order_relaxed);
  }

  for (size_t i = 0; i < max_image; ++i)
  {
    sensor_msgs::msg::Image::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      if (image_queue_.empty())
      {
        break;
      }
      msg = image_queue_.front();
      image_queue_.pop();
    }
    if (callbacks_.image)
    {
      callbacks_.image(msg);
    }
    if (options_.publish_debug_topics && image_pub_)
    {
      image_pub_->publish(*msg);
    }
    delivered_image_.fetch_add(1, std::memory_order_relaxed);
  }

  for (size_t i = 0; i < max_imu; ++i)
  {
    sensor_msgs::msg::Imu::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      if (imu_queue_.empty())
      {
        break;
      }
      msg = imu_queue_.front();
      imu_queue_.pop();
    }
    if (callbacks_.imu)
    {
      callbacks_.imu(msg);
    }
    if (options_.publish_debug_topics && imu_pub_)
    {
      imu_pub_->publish(*msg);
    }
    delivered_imu_.fetch_add(1, std::memory_order_relaxed);
  }
}

OdinDirectStats OdinDirectSdk::stats() const
{
  OdinDirectStats stats;
  stats.sdk_imu = sdk_imu_.load(std::memory_order_relaxed);
  stats.sdk_cloud = sdk_cloud_.load(std::memory_order_relaxed);
  stats.sdk_image = sdk_image_.load(std::memory_order_relaxed);
  stats.delivered_imu = delivered_imu_.load(std::memory_order_relaxed);
  stats.delivered_cloud = delivered_cloud_.load(std::memory_order_relaxed);
  stats.delivered_image = delivered_image_.load(std::memory_order_relaxed);
  stats.dropped_imu = dropped_imu_.load(std::memory_order_relaxed);
  stats.dropped_cloud = dropped_cloud_.load(std::memory_order_relaxed);
  stats.dropped_image = dropped_image_.load(std::memory_order_relaxed);
  return stats;
}

void OdinDirectSdk::deviceCallbackStatic(const lidar_device_info_t *device, bool attach)
{
  std::lock_guard<std::mutex> lock(g_active_mutex);
  if (g_active)
  {
    g_active->deviceCallback(device, attach);
  }
}

void OdinDirectSdk::dataCallbackStatic(const lidar_data_t *data, void *user_data)
{
  auto *self = static_cast<OdinDirectSdk *>(user_data);
  if (self)
  {
    self->dataCallback(data);
  }
}

void OdinDirectSdk::deviceCallback(const lidar_device_info_t *device, bool attach)
{
  if (!attach)
  {
    RCLCPP_WARN(node_->get_logger(), "Odin direct SDK: device detached");
    device_connected_.store(false, std::memory_order_release);
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Odin direct SDK: device attached");
  if (!configureDevice(device))
  {
    RCLCPP_ERROR(node_->get_logger(), "Odin direct SDK: device configuration failed");
    device_connected_.store(false, std::memory_order_release);
    return;
  }
  device_connected_.store(true, std::memory_order_release);
}

void OdinDirectSdk::dataCallback(const lidar_data_t *data)
{
  if (!data || !device_connected_.load(std::memory_order_relaxed))
  {
    return;
  }

  switch (data->type)
  {
  case LIDAR_DT_RAW_IMU:
  {
    if (!send_imu_)
    {
      return;
    }
    const auto *imu_data = reinterpret_cast<const imu_convert_data_t *>(data->stream.imageList[0].pAddr);
    if (!imu_data)
    {
      return;
    }
    sdk_imu_.fetch_add(1, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(raw_imu_mutex_);
      dropOldestIfFull(raw_imu_queue_, kRawImuMax, dropped_imu_);
      raw_imu_queue_.push(*imu_data);
    }
    raw_imu_cv_.notify_one();
    break;
  }
  case LIDAR_DT_RAW_DTOF:
  {
    if (!send_dtof_)
    {
      return;
    }
    const auto &stream = data->stream;
    if (stream.imageCount < 2)
    {
      return;
    }
    const auto &cloud_buf = stream.imageList[1];
    if (!cloud_buf.pAddr || cloud_buf.width == 0 || cloud_buf.height == 0)
    {
      return;
    }

    CloudFrame frame;
    frame.width = cloud_buf.width;
    frame.height = cloud_buf.height;
    frame.timestamp = cloud_buf.timestamp;
    frame.image_count = stream.imageCount;

    const size_t point_count = static_cast<size_t>(cloud_buf.width) * static_cast<size_t>(cloud_buf.height);
    const size_t xyz_stride = stream.imageCount == 4 ? 3 : 4;
    frame.xyz_data.assign(
        static_cast<const uint8_t *>(cloud_buf.pAddr),
        static_cast<const uint8_t *>(cloud_buf.pAddr) + point_count * xyz_stride * sizeof(float));

    if (stream.imageCount >= 3 && stream.imageList[2].pAddr)
    {
      const size_t intensity_size = point_count * (stream.imageCount == 4 ? sizeof(uint8_t) : sizeof(uint16_t));
      frame.intensity_data.assign(
          static_cast<const uint8_t *>(stream.imageList[2].pAddr),
          static_cast<const uint8_t *>(stream.imageList[2].pAddr) + intensity_size);
    }
    if (stream.imageCount >= 4 && stream.imageList[3].pAddr)
    {
      frame.confidence_data.assign(
          static_cast<const uint8_t *>(stream.imageList[3].pAddr),
          static_cast<const uint8_t *>(stream.imageList[3].pAddr) + point_count * sizeof(uint16_t));
    }

    sdk_cloud_.fetch_add(1, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(raw_cloud_mutex_);
      dropOldestIfFull(raw_cloud_queue_, kRawCloudMax, dropped_cloud_);
      raw_cloud_queue_.push(std::move(frame));
    }
    raw_cloud_cv_.notify_one();
    break;
  }
  case LIDAR_DT_RAW_RGB:
  {
    if (!send_rgb_)
    {
      return;
    }
    const auto &stream = data->stream;
    const auto &image_buf = stream.imageList[0];
    if (!image_buf.pAddr || image_buf.length == 0)
    {
      return;
    }
    ImageFrame frame;
    frame.width = image_buf.width;
    frame.height = image_buf.height;
    frame.timestamp = image_buf.timestamp;
    frame.length = image_buf.length;
    frame.jpeg_data.assign(
        static_cast<const uint8_t *>(image_buf.pAddr),
        static_cast<const uint8_t *>(image_buf.pAddr) + image_buf.length);

    sdk_image_.fetch_add(1, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(raw_image_mutex_);
      dropOldestIfFull(raw_image_queue_, kRawImageMax, dropped_image_);
      raw_image_queue_.push(std::move(frame));
    }
    raw_image_cv_.notify_one();
    break;
  }
  case LIDAR_DT_SLAM_CLOUD:
    if (data_logger_)
    {
      recordSlamCloud(data->stream, 0);
    }
    break;
  case LIDAR_DT_SLAM_ODOMETRY:
    if (data_logger_)
    {
      recordPose(data->stream);
    }
    break;
  default:
    break;
  }
}

void OdinDirectSdk::startWorkers()
{
  workers_running_.store(true, std::memory_order_release);
  imu_thread_ = std::thread(&OdinDirectSdk::imuWorker, this);
  cloud_thread_ = std::thread(&OdinDirectSdk::cloudWorker, this);
  image_thread_ = std::thread(&OdinDirectSdk::imageWorker, this);
}

void OdinDirectSdk::stopWorkers()
{
  workers_running_.store(false, std::memory_order_release);
  raw_imu_cv_.notify_all();
  raw_cloud_cv_.notify_all();
  raw_image_cv_.notify_all();
  if (imu_thread_.joinable())
  {
    imu_thread_.join();
  }
  if (cloud_thread_.joinable())
  {
    cloud_thread_.join();
  }
  if (image_thread_.joinable())
  {
    image_thread_.join();
  }
}

void OdinDirectSdk::imuWorker()
{
  while (workers_running_.load(std::memory_order_acquire))
  {
    imu_convert_data_t raw{};
    {
      std::unique_lock<std::mutex> lock(raw_imu_mutex_);
      raw_imu_cv_.wait(lock, [&] {
        return !raw_imu_queue_.empty() || !workers_running_.load(std::memory_order_acquire);
      });
      if (!workers_running_.load(std::memory_order_acquire))
      {
        break;
      }
      raw = raw_imu_queue_.front();
      raw_imu_queue_.pop();
    }
    auto msg = makeImuMsg(raw);
    pushOutput(imu_queue_, imu_mutex_, dropped_imu_, msg, kOutImuMax);
  }
}

void OdinDirectSdk::cloudWorker()
{
  while (workers_running_.load(std::memory_order_acquire))
  {
    CloudFrame raw;
    {
      std::unique_lock<std::mutex> lock(raw_cloud_mutex_);
      raw_cloud_cv_.wait(lock, [&] {
        return !raw_cloud_queue_.empty() || !workers_running_.load(std::memory_order_acquire);
      });
      if (!workers_running_.load(std::memory_order_acquire))
      {
        break;
      }
      raw = std::move(raw_cloud_queue_.front());
      raw_cloud_queue_.pop();
    }
    auto msg = makeCloudMsg(raw);
    if (msg)
    {
      pushOutput(cloud_queue_, cloud_mutex_, dropped_cloud_, msg, kOutCloudMax);
    }
  }
}

void OdinDirectSdk::imageWorker()
{
  while (workers_running_.load(std::memory_order_acquire))
  {
    ImageFrame raw;
    {
      std::unique_lock<std::mutex> lock(raw_image_mutex_);
      raw_image_cv_.wait(lock, [&] {
        return !raw_image_queue_.empty() || !workers_running_.load(std::memory_order_acquire);
      });
      if (!workers_running_.load(std::memory_order_acquire))
      {
        break;
      }
      raw = std::move(raw_image_queue_.front());
      raw_image_queue_.pop();
    }
    auto msg = makeImageMsg(raw);
    if (msg)
    {
      pushOutput(image_queue_, image_mutex_, dropped_image_, msg, kOutImageMax);
    }
  }
}

bool OdinDirectSdk::configureDevice(const lidar_device_info_t *device)
{
  if (!device)
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(device_mutex_);
  if (device_)
  {
    lidar_unregister_stream_callback(device_);
    lidar_stop_stream(device_, LIDAR_MODE_SLAM);
    lidar_close_device(device_);
    lidar_destory_device(device_);
    device_ = nullptr;
  }

  if (lidar_create_device(const_cast<lidar_device_info_t *>(device), &device_) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Odin direct SDK: lidar_create_device failed");
    return false;
  }

  const bool need_open =
      device->initial_state != LIDAR_DEVICE_INITIALIZED &&
      device->initial_state != LIDAR_DEVICE_STREAM_STOPPED;
  if (need_open && lidar_open_device(device_) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Odin direct SDK: lidar_open_device failed");
    lidar_destory_device(device_);
    device_ = nullptr;
    return false;
  }

  lidar_fireware_version_t version{};
  if (lidar_get_version(device_, &version) == 0)
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "Odin direct SDK firmware kernel=%d.%d.%d soc=%d.%d.%d slam=%d.%d.%d",
        version.kernel_version.major, version.kernel_version.minor, version.kernel_version.patch,
        version.soc_version.major, version.soc_version.minor, version.soc_version.patch,
        version.slam_version.major, version.slam_version.minor, version.slam_version.patch);
  }

  std::filesystem::path config_dir = std::filesystem::path(options_.config_file).parent_path();
  if (config_dir.empty())
  {
    config_dir = std::filesystem::current_path();
  }
  calib_file_ = (config_dir / "calib.yaml").string();
  if (device->initial_state != LIDAR_DEVICE_STREAM_STOPPED)
  {
    if (lidar_get_calib_file(device_, config_dir.c_str()) != 0)
    {
      RCLCPP_WARN(node_->get_logger(), "Odin direct SDK: failed to retrieve calib.yaml into %s", config_dir.c_str());
    }
  }
  if (std::filesystem::exists(calib_file_) && loadCameraParams(calib_file_))
  {
    buildUndistortMap();
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Odin direct SDK: undistort map unavailable; raw decoded images will be used");
  }

  lidar_depth_para_t dtof_param{};
  dtof_param.odr = dtof_fps_ == 100 ? LIDAR_DEPTH_ODR_10HZ : LIDAR_DEPTH_ODR_14_5HZ;
  if (lidar_set_depth_parameter(device_, &dtof_param) != 0)
  {
    RCLCPP_WARN(node_->get_logger(), "Odin direct SDK: lidar_set_depth_parameter failed");
  }

  if (lidar_set_mode(device_, LIDAR_MODE_SLAM) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Odin direct SDK: lidar_set_mode failed");
    lidar_close_device(device_);
    lidar_destory_device(device_);
    device_ = nullptr;
    return false;
  }

  if (parser_ && !parser_->applyCustomParameters(device_))
  {
    RCLCPP_WARN(node_->get_logger(), "Odin direct SDK: some custom parameters failed to apply");
  }

  lidar_data_callback_info_t callback_info{};
  callback_info.data_callback = &OdinDirectSdk::dataCallbackStatic;
  callback_info.user_data = this;
  if (lidar_register_stream_callback(device_, callback_info) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Odin direct SDK: lidar_register_stream_callback failed");
    lidar_close_device(device_);
    lidar_destory_device(device_);
    device_ = nullptr;
    return false;
  }

  uint32_t dtof_subframe_odr = 0;
  if (lidar_start_stream(device_, LIDAR_MODE_SLAM, dtof_subframe_odr) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Odin direct SDK: lidar_start_stream failed");
    lidar_unregister_stream_callback(device_);
    lidar_close_device(device_);
    lidar_destory_device(device_);
    device_ = nullptr;
    return false;
  }
  dtof_subframe_odr_ = static_cast<int>(dtof_subframe_odr);

  send_rgb_ ? lidar_activate_stream_type(device_, LIDAR_DT_RAW_RGB) : lidar_deactivate_stream_type(device_, LIDAR_DT_RAW_RGB);
  send_imu_ ? lidar_activate_stream_type(device_, LIDAR_DT_RAW_IMU) : lidar_deactivate_stream_type(device_, LIDAR_DT_RAW_IMU);
  send_dtof_ ? lidar_activate_stream_type(device_, LIDAR_DT_RAW_DTOF) : lidar_deactivate_stream_type(device_, LIDAR_DT_RAW_DTOF);
  if (data_logger_)
  {
    lidar_activate_stream_type(device_, LIDAR_DT_SLAM_CLOUD);
    lidar_activate_stream_type(device_, LIDAR_DT_SLAM_ODOMETRY);
  }
  else
  {
    lidar_deactivate_stream_type(device_, LIDAR_DT_SLAM_CLOUD);
    lidar_deactivate_stream_type(device_, LIDAR_DT_SLAM_ODOMETRY);
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "Odin direct SDK streams active: rgb=%d imu=%d dtof=%d debug_topics=%d recorddata=%d dtof_subframe_odr=%d",
      send_rgb_, send_imu_, send_dtof_, options_.publish_debug_topics, static_cast<bool>(data_logger_), dtof_subframe_odr_);
  return true;
}

bool OdinDirectSdk::loadConfig()
{
  parser_ = std::make_shared<YamlParser>(options_.config_file);
  if (!parser_->loadConfig())
  {
    RCLCPP_ERROR(node_->get_logger(), "Odin direct SDK: failed to load config: %s", options_.config_file.c_str());
    return false;
  }
  const auto &keys = parser_->getRegisterKeys();
  auto get = [&](const std::string &key, int default_value) {
    const auto it = keys.find(key);
    return it == keys.end() ? default_value : it->second;
  };

  send_rgb_ = get("sendrgb", 1);
  send_imu_ = get("sendimu", 1);
  send_dtof_ = get("senddtof", 1);
  send_rgb_undistort_ = get("sendrgbundistort", 1);
  cloud_confidence_threshold_ = get("cloud_raw_confidence_threshold", 35);
  dtof_fps_ = get("dtof_fps", 100);
  use_host_ros_time_ = get("use_host_ros_time", 0);
  enable_imu_smooth_ = get("enable_imu_smooth", 0);
  imu_smooth_frequency_ = get("imu_smooth_frequency", 400);
  return true;
}

bool OdinDirectSdk::loadCameraParams(const std::string &yaml_file)
{
  try
  {
    YAML::Node config = YAML::LoadFile(yaml_file);
    YAML::Node cam_node = config["cam_0"];
    if (!cam_node)
    {
      return false;
    }
    camera_params_.width = cam_node["image_width"].as<int>();
    camera_params_.height = cam_node["image_height"].as<int>();
    camera_params_.fx = cam_node["A11"].as<double>();
    camera_params_.fy = cam_node["A22"].as<double>();
    camera_params_.cx = cam_node["u0"].as<double>();
    camera_params_.cy = cam_node["v0"].as<double>();
    camera_params_.skew = cam_node["A12"].as<double>();
    camera_params_.k2 = cam_node["k2"].as<double>();
    camera_params_.k3 = cam_node["k3"].as<double>();
    camera_params_.k4 = cam_node["k4"].as<double>();
    camera_params_.k5 = cam_node["k5"].as<double>();
    camera_params_.k6 = cam_node["k6"].as<double>();
    camera_params_.k7 = cam_node["k7"].as<double>();
    camera_params_.p1 = cam_node["p1"].as<double>();
    camera_params_.p2 = cam_node["p2"].as<double>();
    return camera_params_.width > 0 && camera_params_.height > 0 && camera_params_.fx != 0.0 && camera_params_.fy != 0.0;
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(node_->get_logger(), "Odin direct SDK: failed to load camera params: %s", e.what());
    return false;
  }
}

void OdinDirectSdk::buildUndistortMap()
{
  mini_vikit::PolynomialCamera camera(
      camera_params_.width,
      camera_params_.height,
      camera_params_.fx,
      camera_params_.fy,
      camera_params_.cx,
      camera_params_.cy,
      camera_params_.skew,
      camera_params_.k2,
      camera_params_.k3,
      camera_params_.k4,
      camera_params_.k5,
      camera_params_.k6,
      camera_params_.k7);

  undistort_map_x_.create(camera_params_.height, camera_params_.width, CV_32F);
  undistort_map_y_.create(camera_params_.height, camera_params_.width, CV_32F);

  for (int v = 0; v < camera_params_.height; ++v)
  {
    for (int u = 0; u < camera_params_.width; ++u)
    {
      double x_norm = (static_cast<double>(u) - camera.cx()) / camera.fx();
      double y_norm = (static_cast<double>(v) - camera.cy()) / camera.fy();
      x_norm = x_norm - y_norm * camera.skew() / camera.fx();
      const Eigen::Vector2d distorted = camera.world2cam(Eigen::Vector2d(x_norm, y_norm));
      undistort_map_x_.at<float>(v, u) = static_cast<float>(distorted[0]);
      undistort_map_y_.at<float>(v, u) = static_cast<float>(distorted[1]);
    }
  }
  undistort_map_ready_ = true;
}

void OdinDirectSdk::initPublishers()
{
  imu_pub_.reset();
  cloud_pub_.reset();
  image_pub_.reset();
  if (!options_.publish_debug_topics)
  {
    return;
  }
  auto qos_small = rclcpp::QoS(10).reliable().durability_volatile();
  auto qos_sensor = rclcpp::QoS(10).reliable().durability_volatile();
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/odin1/imu", qos_small);
  cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/odin1/cloud_raw", qos_sensor);
  image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/odin1/image/undistorted", qos_sensor);
}

void OdinDirectSdk::initRecorddata()
{
  data_logger_.reset();
  if (!options_.recorddata)
  {
    return;
  }

  try
  {
    BinaryDataLogger::Options opts;
    opts.batch_size = 100;
    opts.base_dir = options_.recorddata_dir;
    data_logger_ = std::make_shared<BinaryDataLogger>(opts);
    recorddata_root_ = data_logger_->root_dir();
    RCLCPP_INFO(node_->get_logger(), "Odin direct SDK recorddata enabled at %s", recorddata_root_.c_str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(node_->get_logger(), "Odin direct SDK: failed to initialize recorddata: %s", e.what());
    data_logger_.reset();
  }
}

void OdinDirectSdk::recordPose(const capture_Image_List_t &stream)
{
  if (!data_logger_ || stream.imageList[0].length != sizeof(ros_odom_convert_complete_t) || !stream.imageList[0].pAddr)
  {
    return;
  }
  const auto *odom = reinterpret_cast<const ros_odom_convert_complete_t *>(stream.imageList[0].pAddr);
  const uint32_t idx = record_pose_index_.fetch_add(1, std::memory_order_relaxed);
  const double ts = static_cast<double>(odom->timestamp_ns) / 1e9;
  const float pose[7] = {
      static_cast<float>(static_cast<double>(odom->pos[0]) / 1e6),
      static_cast<float>(static_cast<double>(odom->pos[1]) / 1e6),
      static_cast<float>(static_cast<double>(odom->pos[2]) / 1e6),
      static_cast<float>(static_cast<double>(odom->orient[0]) / 1e6),
      static_cast<float>(static_cast<double>(odom->orient[1]) / 1e6),
      static_cast<float>(static_cast<double>(odom->orient[2]) / 1e6),
      static_cast<float>(static_cast<double>(odom->orient[3]) / 1e6)};

  std::vector<uint8_t> blob;
  blob.reserve(sizeof(uint32_t) + sizeof(double) + sizeof(float) * 7);
  appendPod(blob, idx);
  appendPod(blob, ts);
  for (float value : pose)
  {
    appendPod(blob, value);
  }
  data_logger_->enqueuePoseFrame(std::move(blob));
}

void OdinDirectSdk::recordSlamCloud(const capture_Image_List_t &stream, int idx)
{
  if (!data_logger_ || idx < 0 || idx >= DEVICE_MAX_CH_NUMBER || !stream.imageList[idx].pAddr)
  {
    return;
  }
  const auto &buf = stream.imageList[idx];
  const size_t point_size = sizeof(int32_t) * 7;
  const uint32_t points = static_cast<uint32_t>(buf.length / point_size);
  if (points == 0)
  {
    return;
  }

  const double ts = static_cast<double>(stream.imageList[0].timestamp) / 1e9;
  const uint32_t cloud_idx = record_cloud_index_.fetch_add(1, std::memory_order_relaxed);
  const auto *xyz_data = static_cast<const int32_t *>(buf.pAddr);

  std::vector<uint8_t> blob;
  blob.reserve(sizeof(uint32_t) + sizeof(double) + sizeof(uint32_t) + static_cast<size_t>(points) * (sizeof(float) * 3 + sizeof(uint8_t) * 4));
  appendPod(blob, cloud_idx);
  appendPod(blob, ts);
  appendPod(blob, points);
  for (uint32_t i = 0; i < points; ++i)
  {
    const int32_t *ptr = xyz_data + 7 * i;
    const float x = static_cast<float>(ptr[0]) / 10000.0f;
    const float y = static_cast<float>(ptr[1]) / 10000.0f;
    const float z = static_cast<float>(ptr[2]) / 10000.0f;
    const uint8_t r = static_cast<uint8_t>(ptr[3] & 0xff);
    const uint8_t g = static_cast<uint8_t>(ptr[4] & 0xff);
    const uint8_t b = static_cast<uint8_t>(ptr[5] & 0xff);
    const uint8_t a = static_cast<uint8_t>(ptr[6] & 0xff);
    appendPod(blob, x);
    appendPod(blob, y);
    appendPod(blob, z);
    blob.push_back(r);
    blob.push_back(g);
    blob.push_back(b);
    blob.push_back(a);
  }
  data_logger_->enqueuePointCloudFrame(std::move(blob));
}

void OdinDirectSdk::writeCamInEx()
{
  if (!data_logger_ || calib_file_.empty() || recorddata_root_.empty() || !std::filesystem::exists(calib_file_))
  {
    return;
  }

  try
  {
    YAML::Node root = YAML::LoadFile(calib_file_);
    YAML::Node cam0 = root["cam_0"];
    YAML::Node tcl = root["Tcl_0"];
    const std::filesystem::path out_path = recorddata_root_ / "image" / "cam_in_ex.txt";
    std::ofstream out(out_path, std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
      return;
    }
    out << "Tcl_0: [";
    for (int i = 0; i < 16; ++i)
    {
      if (i > 0)
      {
        out << ", ";
        if (i % 4 == 0)
        {
          out << "\n        ";
        }
      }
      const double value = (tcl && tcl.IsSequence() && i < static_cast<int>(tcl.size())) ? tcl[i].as<double>() : (i == 15 ? 1.0 : 0.0);
      out << formatDouble(value);
    }
    out << "]\n";
    out << "cam_0: \n";
    auto write_key = [&](const char *key, const std::string &default_value) {
      out << "   " << key << ": ";
      if (cam0 && cam0[key])
      {
        out << cam0[key].as<std::string>();
      }
      else
      {
        out << default_value;
      }
      out << "\n";
    };
    write_key("image_width", "0");
    write_key("image_height", "0");
    write_key("k2", "0.000000");
    write_key("k3", "0.000000");
    write_key("k4", "0.000000");
    write_key("k5", "0.000000");
    write_key("k6", "0.000000");
    write_key("k7", "0.000000");
    write_key("p1", "0.000000");
    write_key("p2", "0.000000");
    write_key("A11", "0.000000");
    write_key("A12", "0.000000");
    write_key("A22", "0.000000");
    write_key("u0", "0.000000");
    write_key("v0", "0.000000");
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(node_->get_logger(), "Odin direct SDK: failed to write cam_in_ex.txt: %s", e.what());
  }
}

sensor_msgs::msg::Imu::SharedPtr OdinDirectSdk::makeImuMsg(const imu_convert_data_t &stream) const
{
  auto msg = std::make_shared<sensor_msgs::msg::Imu>();
  msg->header.stamp = makeStamp(stream.stamp);
  msg->header.frame_id = "imu_link";
  msg->linear_acceleration.y = -1.0 * stream.accel_x;
  msg->linear_acceleration.x = stream.accel_y;
  msg->linear_acceleration.z = stream.accel_z;
  msg->angular_velocity.y = -1.0 * stream.gyro_x;
  msg->angular_velocity.x = stream.gyro_y;
  msg->angular_velocity.z = stream.gyro_z;
  msg->orientation.x = 0.0;
  msg->orientation.y = 0.0;
  msg->orientation.z = 0.0;
  msg->orientation.w = 1.0;

  if (data_logger_)
  {
    const double ts = static_cast<double>(stream.stamp) / 1e9;
    const float ax = static_cast<float>(msg->linear_acceleration.x);
    const float ay = static_cast<float>(msg->linear_acceleration.y);
    const float az = static_cast<float>(msg->linear_acceleration.z);
    const float wx = static_cast<float>(msg->angular_velocity.x);
    const float wy = static_cast<float>(msg->angular_velocity.y);
    const float wz = static_cast<float>(msg->angular_velocity.z);
    std::vector<uint8_t> blob;
    blob.reserve(sizeof(double) + sizeof(float) * 6);
    appendPod(blob, ts);
    appendPod(blob, ax);
    appendPod(blob, ay);
    appendPod(blob, az);
    appendPod(blob, wx);
    appendPod(blob, wy);
    appendPod(blob, wz);
    data_logger_->enqueueIMUFrame(std::move(blob));
  }
  return msg;
}

sensor_msgs::msg::PointCloud2::SharedPtr OdinDirectSdk::makeCloudMsg(const CloudFrame &frame) const
{
  const size_t point_count = static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height);
  if (point_count == 0 || frame.xyz_data.empty())
  {
    return nullptr;
  }

  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp = makeStamp(frame.timestamp);
  msg->header.frame_id = "odin1_base_link";
  msg->height = frame.height;
  msg->width = frame.width;
  msg->is_dense = false;
  msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::UINT8,
      "confidence", 1, sensor_msgs::msg::PointField::UINT16,
      "offset_time", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(point_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(*msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_confidence(*msg, "confidence");
  sensor_msgs::PointCloud2Iterator<float> iter_offset(*msg, "offset_time");

  const auto *xyz = reinterpret_cast<const float *>(frame.xyz_data.data());
  const float *xyz_end = reinterpret_cast<const float *>(frame.xyz_data.data() + frame.xyz_data.size());

  if (frame.image_count == 4)
  {
    const auto *intensity = frame.intensity_data.empty() ? nullptr : frame.intensity_data.data();
    const auto *confidence = frame.confidence_data.empty() ? nullptr : reinterpret_cast<const uint16_t *>(frame.confidence_data.data());
    const float dtof_subframe_odr = static_cast<float>(dtof_subframe_odr_) / 1000.0f;
    for (size_t i = 0; i < point_count; ++i)
    {
      const bool has_xyz = xyz + 2 < xyz_end;
      const uint16_t conf = confidence ? confidence[i] : 1;
      if (!has_xyz || conf < static_cast<uint16_t>(cloud_confidence_threshold_))
      {
        *iter_x = 0.0f;
        *iter_y = 0.0f;
        *iter_z = 0.0f;
        *iter_intensity = 0;
        *iter_confidence = 0;
        *iter_offset = 0.0f;
      }
      else
      {
        *iter_x = xyz[i * 3 + 2] / 1000.0f;
        *iter_y = -xyz[i * 3 + 0] / 1000.0f;
        *iter_z = xyz[i * 3 + 1] / 1000.0f;
        *iter_intensity = intensity ? intensity[i] : 0;
        *iter_confidence = conf;
        if (dtof_subframe_odr > 0.0f)
        {
          const int line_num = static_cast<int>(i / 256);
          const int group = line_num / kDtofRowsPerGroup;
          *iter_offset = static_cast<float>(group) / dtof_subframe_odr;
        }
        else
        {
          *iter_offset = 0.0f;
        }
      }
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
      ++iter_confidence;
      ++iter_offset;
    }
  }
  else
  {
    const auto *intensity = frame.intensity_data.empty() ? nullptr : reinterpret_cast<const uint16_t *>(frame.intensity_data.data());
    for (size_t i = 0; i < point_count; ++i)
    {
      const bool has_xyz = xyz + i * 4 + 2 < xyz_end;
      if (has_xyz)
      {
        *iter_x = xyz[i * 4 + 2] / 1000.0f;
        *iter_y = -xyz[i * 4 + 0] / 1000.0f;
        *iter_z = xyz[i * 4 + 1] / 1000.0f;
      }
      else
      {
        *iter_x = 0.0f;
        *iter_y = 0.0f;
        *iter_z = 0.0f;
      }
      float value = 0.0f;
      if (intensity)
      {
        value = (static_cast<float>(intensity[i]) - 10.0f) * 255.0f / (12500.0f - 10.0f);
      }
      *iter_intensity = static_cast<uint8_t>(std::clamp(value, 0.0f, 255.0f));
      *iter_confidence = 0;
      *iter_offset = 0.0f;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
      ++iter_confidence;
      ++iter_offset;
    }
  }
  return msg;
}

sensor_msgs::msg::Image::SharedPtr OdinDirectSdk::makeImageMsg(const ImageFrame &frame)
{
  cv::Mat encoded(1, static_cast<int>(frame.jpeg_data.size()), CV_8UC1, const_cast<uint8_t *>(frame.jpeg_data.data()));
  cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
  if (decoded.empty())
  {
    return nullptr;
  }
  cv::Mat output = decoded;
  if (send_rgb_undistort_ && undistort_map_ready_)
  {
    cv::Mat undistorted;
    cv::remap(decoded, undistorted, undistort_map_x_, undistort_map_y_, cv::INTER_LINEAR);
    output = std::move(undistorted);
  }

  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->header.stamp = makeStamp(frame.timestamp);
  msg->header.frame_id = "odin1_camera";
  msg->height = static_cast<uint32_t>(output.rows);
  msg->width = static_cast<uint32_t>(output.cols);
  msg->encoding = "bgr8";
  msg->is_bigendian = false;
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(output.cols * output.elemSize());
  msg->data.assign(output.datastart, output.dataend);

  if (data_logger_)
  {
    const uint32_t idx = record_image_index_.fetch_add(1, std::memory_order_relaxed);
    const double ts = static_cast<double>(frame.timestamp) / 1e9;
    const uint32_t jpeg_size = static_cast<uint32_t>(frame.jpeg_data.size());
    std::vector<uint8_t> blob;
    blob.reserve(sizeof(uint32_t) + sizeof(double) + sizeof(uint32_t) + frame.jpeg_data.size());
    appendPod(blob, idx);
    appendPod(blob, ts);
    appendPod(blob, jpeg_size);
    blob.insert(blob.end(), frame.jpeg_data.begin(), frame.jpeg_data.end());
    data_logger_->enqueueImageFrame(std::move(blob));
  }
  return msg;
}

builtin_interfaces::msg::Time OdinDirectSdk::makeStamp(uint64_t sensor_timestamp_ns) const
{
  if (use_host_ros_time_ == 1 && node_)
  {
    return node_->now();
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(sensor_timestamp_ns / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(sensor_timestamp_ns % 1000000000ULL);
  return stamp;
}

template <typename T>
void OdinDirectSdk::pushOutput(std::queue<std::shared_ptr<T>> &queue,
                               std::mutex &mutex,
                               std::atomic<uint64_t> &drop_counter,
                               std::shared_ptr<T> msg,
                               size_t max_size)
{
  std::lock_guard<std::mutex> lock(mutex);
  dropOldestIfFull(queue, max_size, drop_counter);
  queue.push(std::move(msg));
}

} // namespace odin_ros_driver

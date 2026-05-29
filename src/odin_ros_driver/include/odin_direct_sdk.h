/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
*/
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "data_logger.h"
#include "lidar_api.h"

namespace odin_ros_driver
{

class YamlParser;

struct OdinDirectOptions
{
  std::string config_file;
  bool recorddata = false;
  bool publish_debug_topics = false;
  std::string recorddata_dir;
};

struct OdinDirectCallbacks
{
  std::function<void(sensor_msgs::msg::Imu::ConstSharedPtr)> imu;
  std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr)> cloud;
  std::function<void(sensor_msgs::msg::Image::ConstSharedPtr)> image;
};

struct OdinDirectStats
{
  uint64_t sdk_imu = 0;
  uint64_t sdk_cloud = 0;
  uint64_t sdk_image = 0;
  uint64_t delivered_imu = 0;
  uint64_t delivered_cloud = 0;
  uint64_t delivered_image = 0;
  uint64_t dropped_imu = 0;
  uint64_t dropped_cloud = 0;
  uint64_t dropped_image = 0;
};

class OdinDirectSdk
{
public:
  explicit OdinDirectSdk(rclcpp::Node *node);
  ~OdinDirectSdk();

  bool start(const OdinDirectOptions &options, OdinDirectCallbacks callbacks);
  void stop();
  void spinSome(size_t max_imu = 200, size_t max_cloud = 5, size_t max_image = 5);

  bool connected() const { return device_connected_.load(std::memory_order_relaxed); }
  OdinDirectStats stats() const;

private:
  struct CloudFrame
  {
    std::vector<uint8_t> xyz_data;
    std::vector<uint8_t> intensity_data;
    std::vector<uint8_t> confidence_data;
    uint32_t width = 0;
    uint32_t height = 0;
    uint64_t timestamp = 0;
    uint32_t image_count = 0;
  };

  struct ImageFrame
  {
    std::vector<uint8_t> jpeg_data;
    uint32_t width = 0;
    uint32_t height = 0;
    uint64_t timestamp = 0;
    uint32_t length = 0;
  };

  static void deviceCallbackStatic(const lidar_device_info_t *device, bool attach);
  static void dataCallbackStatic(const lidar_data_t *data, void *user_data);

  void deviceCallback(const lidar_device_info_t *device, bool attach);
  void dataCallback(const lidar_data_t *data);

  void startWorkers();
  void stopWorkers();
  void imuWorker();
  void cloudWorker();
  void imageWorker();

  bool configureDevice(const lidar_device_info_t *device);
  bool loadConfig();
  bool loadCameraParams(const std::string &yaml_file);
  void buildUndistortMap();
  void initPublishers();
  void initRecorddata();
  void recordPose(const capture_Image_List_t &stream);
  void recordSlamCloud(const capture_Image_List_t &stream, int idx);
  void writeCamInEx();

  sensor_msgs::msg::Imu::SharedPtr makeImuMsg(const imu_convert_data_t &stream) const;
  sensor_msgs::msg::PointCloud2::SharedPtr makeCloudMsg(const CloudFrame &frame) const;
  sensor_msgs::msg::Image::SharedPtr makeImageMsg(const ImageFrame &frame);
  builtin_interfaces::msg::Time makeStamp(uint64_t sensor_timestamp_ns) const;

  template <typename T>
  void pushOutput(std::queue<std::shared_ptr<T>> &queue,
                  std::mutex &mutex,
                  std::atomic<uint64_t> &drop_counter,
                  std::shared_ptr<T> msg,
                  size_t max_size);

  rclcpp::Node *node_ = nullptr;
  OdinDirectOptions options_;
  OdinDirectCallbacks callbacks_;

  std::atomic<bool> started_{false};
  std::atomic<bool> device_connected_{false};
  std::atomic<bool> workers_running_{false};
  device_handle device_ = nullptr;
  mutable std::mutex device_mutex_;

  std::thread imu_thread_;
  std::thread cloud_thread_;
  std::thread image_thread_;

  std::queue<imu_convert_data_t> raw_imu_queue_;
  std::queue<CloudFrame> raw_cloud_queue_;
  std::queue<ImageFrame> raw_image_queue_;
  mutable std::mutex raw_imu_mutex_;
  mutable std::mutex raw_cloud_mutex_;
  mutable std::mutex raw_image_mutex_;
  std::condition_variable raw_imu_cv_;
  std::condition_variable raw_cloud_cv_;
  std::condition_variable raw_image_cv_;

  std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_queue_;
  std::queue<sensor_msgs::msg::Image::SharedPtr> image_queue_;
  mutable std::mutex imu_mutex_;
  mutable std::mutex cloud_mutex_;
  mutable std::mutex image_mutex_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  std::shared_ptr<BinaryDataLogger> data_logger_;
  std::shared_ptr<YamlParser> parser_;
  std::filesystem::path recorddata_root_;
  std::string calib_file_;

  int send_rgb_ = 1;
  int send_imu_ = 1;
  int send_dtof_ = 1;
  int send_rgb_undistort_ = 1;
  int cloud_confidence_threshold_ = 35;
  int dtof_fps_ = 100;
  int use_host_ros_time_ = 0;
  int enable_imu_smooth_ = 0;
  int imu_smooth_frequency_ = 400;
  int dtof_subframe_odr_ = 0;

  struct CameraParams
  {
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double skew = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
    double k5 = 0.0;
    double k6 = 0.0;
    double k7 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
  };
  CameraParams camera_params_;
  bool undistort_map_ready_ = false;
  cv::Mat undistort_map_x_;
  cv::Mat undistort_map_y_;

  std::atomic<uint64_t> sdk_imu_{0};
  std::atomic<uint64_t> sdk_cloud_{0};
  std::atomic<uint64_t> sdk_image_{0};
  std::atomic<uint64_t> delivered_imu_{0};
  std::atomic<uint64_t> delivered_cloud_{0};
  std::atomic<uint64_t> delivered_image_{0};
  std::atomic<uint64_t> dropped_imu_{0};
  std::atomic<uint64_t> dropped_cloud_{0};
  std::atomic<uint64_t> dropped_image_{0};
  std::atomic<uint32_t> record_pose_index_{0};
  std::atomic<uint32_t> record_cloud_index_{0};
  std::atomic<uint32_t> record_image_index_{0};
};

} // namespace odin_ros_driver

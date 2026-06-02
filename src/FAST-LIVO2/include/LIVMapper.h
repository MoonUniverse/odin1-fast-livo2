/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <odin_direct_sdk.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vikit/pinhole_camera.h>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <thread>

class LIVMapper : public rclcpp::Node
{
public:
  struct TopicDiagStats
  {
    string name;
    double expected_hz = 0.0;
    vector<double> arrival_times;
    vector<double> header_stamps;
  };

  explicit LIVMapper(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~LIVMapper();
  void initializeSubscribersAndPublishers();
  void initializeDirectOdinInput();
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void processImu();
  bool shouldSaveForPose(bool has_last_pose, const V3D &last_pos, const M3D &last_rot, double last_time, double current_time) const;
  void markPcdSaved(double save_time);
  void markImageSaved(double save_time);
  void saveFinalMap();
  void startAsyncSaveWorker();
  void stopAsyncSaveWorker();
  bool enqueueAsyncPcdRgb(const std::string &path, const PointCloudXYZRGB::Ptr &cloud, const std::string &pose_line);
  bool enqueueAsyncPcdIntensity(const std::string &path, const PointCloudXYZI::Ptr &cloud, const std::string &pose_line);
  bool enqueueAsyncImage(const std::string &path, const cv::Mat &image, const std::string &pose_line);
  void asyncSaveWorker();
  void writeAsyncSaveStats();
  std::string outputPath(const std::string &relative_path) const;
  void recordInternalTopicSample(TopicDiagStats &stats, double header_stamp);
  void writeInternalTopicReport();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback();
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstSharedPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstSharedPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstSharedPtr &msg_in);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &pubSubVisualMap);
  void publish_effect_world(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr &pubOdomAftMapped);
  void publish_mavros(const rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr &mavros_pose_publisher);
  void publish_path(const rclcpp::Publisher<nav_msgs::Path>::SharedPtr &pubPath);
  void readParameters();
  bool loadCameraFromParameters();
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir, output_run_dir, internal_topic_report_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  string input_source = "ros_topic", odin_direct_config_file, odin_direct_recorddata_dir;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double b_gyr_cov = 0, b_acc_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  bool final_map_save_en = false;
  bool verbose_log_en = false;
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
  bool pcd_async_save_en = true, image_async_save_en = true;
  int pcd_async_queue_size = 8, image_async_queue_size = 8;
  string pcd_save_trigger_mode = "interval", img_save_trigger_mode = "interval";
  double save_pose_translation_m = 0.2, save_pose_rotation_deg = 10.0, save_pose_min_interval_s = 0.0;
  bool last_pcd_save_pose_valid = false, last_image_save_pose_valid = false;
  V3D last_pcd_save_pos = V3D::Zero(), last_image_save_pos = V3D::Zero();
  M3D last_pcd_save_rot = M3D::Identity(), last_image_save_rot = M3D::Identity();
  double last_pcd_save_time = -1.0, last_image_save_time = -1.0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::Odometry imu_prop_odom;
  rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  bool odin_direct_recorddata = false;
  bool odin_direct_publish_debug_topics = false;
  bool lidar_qos_reliable = false;
  bool img_qos_reliable = true;
  int img_en = 1, imu_int_frame = 3;
  int lidar_queue_size = 200000;
  int img_queue_size = 200;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstSharedPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;
  PointCloudXYZRGB::Ptr pcl_final_map_rgb;
  PointCloudXYZI::Ptr pcl_final_map_intensity;

  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;

  rclcpp::Publisher<visualization_msgs::Marker>::SharedPtr plane_pub;
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr voxel_pub;
  rclcpp::Subscription<sensor_msgs::PointCloud2>::SharedPtr sub_pcl;
  rclcpp::Subscription<livox_ros_driver::CustomMsg>::SharedPtr sub_livox_pcl;
  rclcpp::Subscription<sensor_msgs::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::Image>::SharedPtr sub_img;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudFullRes;
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr pubNormal;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubSubVisualMap;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudEffect;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudMap;
  rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<nav_msgs::Path>::SharedPtr pubPath;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudDyn;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudDynRmed;
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr mavros_pose_publisher;
  rclcpp::TimerBase::SharedPtr imu_prop_timer;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<odin_ros_driver::OdinDirectSdk> odin_direct_sdk_;
  std::unique_ptr<vk::AbstractCamera> camera_;

  struct AsyncSaveJob
  {
    enum class Type
    {
      PcdRgb,
      PcdIntensity,
      Image
    };
    Type type;
    std::string path;
    std::string pose_line;
    PointCloudXYZRGB::Ptr rgb_cloud;
    PointCloudXYZI::Ptr intensity_cloud;
    cv::Mat image;
  };

  std::mutex async_save_mutex_;
  std::condition_variable async_save_cv_;
  std::deque<AsyncSaveJob> async_pcd_jobs_;
  std::deque<AsyncSaveJob> async_image_jobs_;
  std::thread async_save_thread_;
  bool async_save_running_ = false;
  uint64_t async_pcd_enqueued_ = 0, async_pcd_written_ = 0, async_pcd_dropped_ = 0, async_pcd_failed_ = 0;
  uint64_t async_image_enqueued_ = 0, async_image_written_ = 0, async_image_dropped_ = 0, async_image_failed_ = 0;
  std::chrono::steady_clock::time_point diag_start_time;
  TopicDiagStats diag_imu, diag_cloud, diag_image;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif

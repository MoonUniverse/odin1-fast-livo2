/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <sstream>

namespace
{

template <typename T>
void readParam(rclcpp::Node &node, const std::string &name, T &value, const T &default_value)
{
  value = node.declare_parameter<T>(name, default_value);
}

void readParam(rclcpp::Node &node, const std::string &name, double &value, const double &default_value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  node.declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
  const auto parameter = node.get_parameter(name);
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    value = static_cast<double>(parameter.as_int());
  }
  else
  {
    value = parameter.as_double();
  }
}

void readParam(rclcpp::Node &node, const std::string &name, std::vector<double> &value, const std::vector<double> &default_value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  node.declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
  const auto parameter = node.get_parameter(name);
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
  {
    const auto integers = parameter.as_integer_array();
    value.assign(integers.begin(), integers.end());
  }
  else
  {
    value = parameter.as_double_array();
  }
}

} // namespace

LIVMapper::LIVMapper(const rclcpp::NodeOptions &options)
    : rclcpp::Node("laserMapping", options),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  diag_start_time = std::chrono::steady_clock::now();
  diag_imu.name = "/odin1/imu";
  diag_imu.expected_hz = 400.0;
  diag_cloud.name = "/odin1/cloud_raw";
  diag_cloud.expected_hz = 10.0;
  diag_image.name = "/odin1/image/undistorted";
  diag_image.expected_hz = 10.0;

  readParameters();
  VoxelMapConfig voxel_config;
  loadVoxelConfig(*this, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  pcl_final_map_rgb.reset(new PointCloudXYZRGB());
  pcl_final_map_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = fast_livo::timeToMsg(this->now());
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters()
{
  readParam(*this, "common.lid_topic", lid_topic, std::string("/livox/lidar"));
  readParam(*this, "common.imu_topic", imu_topic, std::string("/livox/imu"));
  readParam(*this, "common.ros_driver_bug_fix", ros_driver_fix_en, false);
  readParam(*this, "common.img_en", img_en, 1);
  readParam(*this, "common.lidar_en", lidar_en, 1);
  readParam(*this, "common.lidar_qos_reliable", lidar_qos_reliable, false);
  readParam(*this, "common.lidar_queue_size", lidar_queue_size, 200000);
  readParam(*this, "common.img_topic", img_topic, std::string("/left_camera/image"));
  readParam(*this, "common.img_qos_reliable", img_qos_reliable, true);
  readParam(*this, "common.img_queue_size", img_queue_size, 200);

  readParam(*this, "vio.normal_en", normal_en, true);
  readParam(*this, "vio.inverse_composition_en", inverse_composition_en, false);
  readParam(*this, "vio.max_iterations", max_iterations, 5);
  readParam(*this, "vio.img_point_cov", IMG_POINT_COV, 100.0);
  readParam(*this, "vio.raycast_en", raycast_en, false);
  readParam(*this, "vio.exposure_estimate_en", exposure_estimate_en, true);
  readParam(*this, "vio.inv_expo_cov", inv_expo_cov, 0.2);
  readParam(*this, "vio.grid_size", grid_size, 5);
  readParam(*this, "vio.grid_n_height", grid_n_height, 17);
  readParam(*this, "vio.patch_pyrimid_level", patch_pyrimid_level, 3);
  readParam(*this, "vio.patch_size", patch_size, 8);
  readParam(*this, "vio.outlier_threshold", outlier_threshold, 1000.0);

  readParam(*this, "time_offset.exposure_time_init", exposure_time_init, 0.0);
  readParam(*this, "time_offset.img_time_offset", img_time_offset, 0.0);
  readParam(*this, "time_offset.imu_time_offset", imu_time_offset, 0.0);
  readParam(*this, "time_offset.lidar_time_offset", lidar_time_offset, 0.0);
  readParam(*this, "uav.imu_rate_odom", imu_prop_enable, false);
  readParam(*this, "uav.gravity_align_en", gravity_align_en, false);

  readParam(*this, "evo.seq_name", seq_name, std::string("01"));
  readParam(*this, "evo.pose_output_en", pose_output_en, false);
  readParam(*this, "imu.gyr_cov", gyr_cov, 1.0);
  readParam(*this, "imu.acc_cov", acc_cov, 1.0);
  readParam(*this, "imu.b_gyr_cov", b_gyr_cov, 0.0001);
  readParam(*this, "imu.b_acc_cov", b_acc_cov, 0.0001);
  readParam(*this, "imu.imu_int_frame", imu_int_frame, 3);
  readParam(*this, "imu.imu_en", imu_en, false);
  readParam(*this, "imu.gravity_est_en", gravity_est_en, true);
  readParam(*this, "imu.ba_bg_est_en", ba_bg_est_en, true);

  readParam(*this, "preprocess.blind", p_pre->blind, 0.01);
  readParam(*this, "preprocess.filter_size_surf", filter_size_surf_min, 0.5);
  readParam(*this, "preprocess.hilti_en", hilti_en, false);
  readParam(*this, "preprocess.lidar_type", p_pre->lidar_type, static_cast<int>(AVIA));
  readParam(*this, "preprocess.scan_line", p_pre->N_SCANS, 6);
  readParam(*this, "preprocess.point_filter_num", p_pre->point_filter_num, 3);
  readParam(*this, "preprocess.feature_extract_enabled", p_pre->feature_enabled, false);

  readParam(*this, "pcd_save.interval", pcd_save_interval, -1);
  readParam(*this, "pcd_save.pcd_save_en", pcd_save_en, false);
  readParam(*this, "pcd_save.type", pcd_save_type, 0);
  readParam(*this, "pcd_save.trigger_mode", pcd_save_trigger_mode, std::string("interval"));
  readParam(*this, "pcd_save.final_map_save_en", final_map_save_en, false);
  readParam(*this, "image_save.img_save_en", img_save_en, false);
  readParam(*this, "image_save.interval", img_save_interval, 1);
  readParam(*this, "image_save.trigger_mode", img_save_trigger_mode, std::string("interval"));
  readParam(*this, "save_pose_gate.translation_m", save_pose_translation_m, 0.2);
  readParam(*this, "save_pose_gate.rotation_deg", save_pose_rotation_deg, 10.0);
  readParam(*this, "save_pose_gate.min_interval_s", save_pose_min_interval_s, 0.0);

  readParam(*this, "pcd_save.colmap_output_en", colmap_output_en, false);
  readParam(*this, "pcd_save.filter_size_pcd", filter_size_pcd, 0.5);
  readParam(*this, "extrin_calib.extrinsic_T", extrinT, extrinT);
  readParam(*this, "extrin_calib.extrinsic_R", extrinR, extrinR);
  readParam(*this, "extrin_calib.Pcl", cameraextrinT, cameraextrinT);
  readParam(*this, "extrin_calib.Rcl", cameraextrinR, cameraextrinR);
  readParam(*this, "debug.plot_time", plot_time, -10.0);
  readParam(*this, "debug.frame_cnt", frame_cnt, 6);

  readParam(*this, "publish.blind_rgb_points", blind_rgb_points, 0.01);
  readParam(*this, "publish.pub_scan_num", pub_scan_num, 1);
  readParam(*this, "publish.pub_effect_point_en", pub_effect_point_en, false);
  readParam(*this, "publish.dense_map_en", dense_map_en, false);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

bool LIVMapper::loadCameraFromParameters()
{
  const std::string cam_model = this->declare_parameter<std::string>("cam_model", "Pinhole");
  const int cam_width = this->declare_parameter<int>("cam_width", 0);
  const int cam_height = this->declare_parameter<int>("cam_height", 0);
  const double scale = this->declare_parameter<double>("scale", 1.0);
  const double fx = this->declare_parameter<double>("cam_fx", 0.0);
  const double fy = this->declare_parameter<double>("cam_fy", 0.0);
  const double cx = this->declare_parameter<double>("cam_cx", 0.0);
  const double cy = this->declare_parameter<double>("cam_cy", 0.0);
  const double d0 = this->declare_parameter<double>("cam_d0", 0.0);
  const double d1 = this->declare_parameter<double>("cam_d1", 0.0);
  const double d2 = this->declare_parameter<double>("cam_d2", 0.0);
  const double d3 = this->declare_parameter<double>("cam_d3", 0.0);

  if (cam_width <= 0 || cam_height <= 0 || fx == 0.0 || fy == 0.0) { return false; }

  if (cam_model == "EquidistantCamera")
  {
    camera_ = std::make_unique<vk::EquidistantCamera>(cam_width, cam_height, fx, fy, cx, cy, scale, d0, d1, d2, d3);
  }
  else
  {
    camera_ = std::make_unique<vk::PinholeCamera>(cam_width, cam_height, fx, fy, cx, cy, scale, d0, d1, d2, d3);
  }
  vio_manager->cam = camera_.get();
  return true;
}

void LIVMapper::initializeComponents() 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!loadCameraFromParameters()) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/pcd");
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/image");
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/result");

  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers()
{
  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(200000);
  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(std::max(1, lidar_queue_size)));
  if (lidar_qos_reliable)
  {
    lidar_qos.reliable();
  }
  else
  {
    lidar_qos.best_effort();
  }
  if (p_pre->lidar_type == AVIA)
  {
    sub_livox_pcl = create_subscription<livox_ros_driver::CustomMsg>(
        lid_topic, lidar_qos, std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
  }
  else
  {
    sub_pcl = create_subscription<sensor_msgs::PointCloud2>(
        lid_topic, lidar_qos, std::bind(&LIVMapper::standard_pcl_cbk, this, std::placeholders::_1));
  }
  sub_imu = create_subscription<sensor_msgs::Imu>(
      imu_topic, sensor_qos, std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1));
  auto image_qos = rclcpp::QoS(rclcpp::KeepLast(std::max(1, img_queue_size)));
  if (img_qos_reliable)
  {
    image_qos.reliable();
  }
  else
  {
    image_qos.best_effort();
  }
  sub_img = create_subscription<sensor_msgs::Image>(
      img_topic, image_qos, std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));

  pubLaserCloudFullRes = create_publisher<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  pubNormal = create_publisher<visualization_msgs::MarkerArray>("visualization_marker", 100);
  pubSubVisualMap = create_publisher<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = create_publisher<sensor_msgs::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = create_publisher<sensor_msgs::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = create_publisher<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  pubPath = create_publisher<nav_msgs::Path>("/path", 10);
  plane_pub = create_publisher<visualization_msgs::Marker>("/planner_normal", 1);
  voxel_pub = create_publisher<visualization_msgs::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = create_publisher<sensor_msgs::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = create_publisher<sensor_msgs::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = create_publisher<sensor_msgs::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = create_publisher<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = image_transport::create_publisher(this, "/rgb_img");
  pubImuPropOdom = create_publisher<nav_msgs::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = create_wall_timer(std::chrono::milliseconds(4), std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_ = create_publisher<visualization_msgs::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleVIO() 
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{    
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  Eigen::Quaterniond quat_cur(_state.rot_end);
  geoQuat.x = quat_cur.x();
  geoQuat.y = quat_cur.y();
  geoQuat.z = quat_cur.z();
  geoQuat.w = quat_cur.w();
  publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

bool LIVMapper::shouldSaveForPose(bool has_last_pose, const V3D &last_pos, const M3D &last_rot, double last_time, double current_time) const
{
  if (!has_last_pose) return true;
  if (save_pose_min_interval_s > 0.0 && last_time > 0.0 && current_time - last_time < save_pose_min_interval_s) return false;

  const double trans_delta = (_state.pos_end - last_pos).norm();
  const M3D rot_delta = last_rot.transpose() * _state.rot_end;
  const double cos_angle = std::clamp((rot_delta.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double rot_delta_deg = std::acos(cos_angle) * 180.0 / M_PI;
  return trans_delta >= save_pose_translation_m || rot_delta_deg >= save_pose_rotation_deg;
}

void LIVMapper::markPcdSaved(double save_time)
{
  last_pcd_save_pos = _state.pos_end;
  last_pcd_save_rot = _state.rot_end;
  last_pcd_save_time = save_time;
  last_pcd_save_pose_valid = true;
}

void LIVMapper::markImageSaved(double save_time)
{
  last_image_save_pos = _state.pos_end;
  last_image_save_rot = _state.rot_end;
  last_image_save_time = save_time;
  last_image_save_pose_valid = true;
}

void LIVMapper::saveFinalMap()
{
  if (!final_map_save_en) return;

  const std::string final_map_dir = std::string(ROOT_DIR) + "Log/pcd/final_map.pcd";
  const std::string final_rgb_map_dir = std::string(ROOT_DIR) + "Log/pcd/final_map_rgb.pcd";
  pcl::PCDWriter pcd_writer;
  if (pcl_final_map_intensity && !pcl_final_map_intensity->empty())
  {
    pcd_writer.writeBinary(final_map_dir, *pcl_final_map_intensity);
    std::cout << GREEN << "Final intensity map saved to: " << final_map_dir
              << " with point count: " << pcl_final_map_intensity->points.size() << RESET << std::endl;
  }
  if (pcl_final_map_rgb && !pcl_final_map_rgb->empty())
  {
    const std::string rgb_path = (pcl_final_map_intensity && !pcl_final_map_intensity->empty()) ? final_rgb_map_dir : final_map_dir;
    pcd_writer.writeBinary(rgb_path, *pcl_final_map_rgb);
    std::cout << GREEN << "Final RGB map saved to: " << rgb_path
              << " with point count: " << pcl_final_map_rgb->points.size() << RESET << std::endl;
  }
}

void LIVMapper::recordInternalTopicSample(TopicDiagStats &stats, double header_stamp)
{
  const auto now = std::chrono::steady_clock::now();
  const double arrival_s = std::chrono::duration<double>(now - diag_start_time).count();
  stats.arrival_times.push_back(arrival_s);
  stats.header_stamps.push_back(header_stamp);
}

namespace
{
double percentileMs(std::vector<double> values_s, double pct)
{
  if (values_s.empty()) return -1.0;
  std::sort(values_s.begin(), values_s.end());
  const size_t idx = static_cast<size_t>(std::round((values_s.size() - 1) * pct / 100.0));
  return values_s[std::min(idx, values_s.size() - 1)] * 1000.0;
}

std::vector<double> makeIntervals(const std::vector<double> &values)
{
  std::vector<double> result;
  if (values.size() < 2) return result;
  result.reserve(values.size() - 1);
  for (size_t i = 1; i < values.size(); ++i) result.push_back(values[i] - values[i - 1]);
  return result;
}

double hzFromSeries(const std::vector<double> &values)
{
  if (values.size() < 2) return 0.0;
  const double duration = values.back() - values.front();
  return duration > 0.0 ? static_cast<double>(values.size() - 1) / duration : 0.0;
}

void writeJsonTopic(std::ofstream &out, const LIVMapper::TopicDiagStats &stats, bool trailing_comma)
{
  const auto arrival_intervals = makeIntervals(stats.arrival_times);
  const auto header_intervals = makeIntervals(stats.header_stamps);
  const double hz = hzFromSeries(stats.arrival_times);
  const double header_hz = hzFromSeries(stats.header_stamps);
  const std::string status =
      stats.expected_hz > 0.0 && hz < stats.expected_hz * 0.8 ? "low_rate" : "ok";

  out << "    \"" << stats.name << "\": {\n"
      << "      \"expected_hz\": " << stats.expected_hz << ",\n"
      << "      \"status\": \"" << status << "\",\n"
      << "      \"count\": " << stats.arrival_times.size() << ",\n"
      << "      \"hz\": " << hz << ",\n"
      << "      \"header_hz\": " << header_hz << ",\n"
      << "      \"arrival_interval_ms\": {\n"
      << "        \"p50\": " << percentileMs(arrival_intervals, 50.0) << ",\n"
      << "        \"p95\": " << percentileMs(arrival_intervals, 95.0) << ",\n"
      << "        \"p99\": " << percentileMs(arrival_intervals, 99.0) << "\n"
      << "      },\n"
      << "      \"header_interval_ms\": {\n"
      << "        \"p50\": " << percentileMs(header_intervals, 50.0) << ",\n"
      << "        \"p95\": " << percentileMs(header_intervals, 95.0) << ",\n"
      << "        \"p99\": " << percentileMs(header_intervals, 99.0) << "\n"
      << "      }\n"
      << "    }" << (trailing_comma ? "," : "") << "\n";
}

void writeMdTopic(std::ofstream &out, const LIVMapper::TopicDiagStats &stats)
{
  const auto arrival_intervals = makeIntervals(stats.arrival_times);
  const auto header_intervals = makeIntervals(stats.header_stamps);
  const double hz = hzFromSeries(stats.arrival_times);
  const double header_hz = hzFromSeries(stats.header_stamps);
  out << "| `" << stats.name << "` | " << stats.arrival_times.size()
      << " | " << std::fixed << std::setprecision(3) << hz
      << " | " << header_hz
      << " | " << percentileMs(arrival_intervals, 95.0)
      << " | " << percentileMs(header_intervals, 95.0) << " |\n";
}
} // namespace

void LIVMapper::writeInternalTopicReport()
{
  const std::filesystem::path output_dir("/tmp/fast_livo_topic_reports");
  std::filesystem::create_directories(output_dir);

  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&t, &tm);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);

  const std::filesystem::path json_path = output_dir / ("fast_livo_internal_report_" + std::string(stamp) + ".json");
  const std::filesystem::path md_path = output_dir / ("fast_livo_internal_report_" + std::string(stamp) + ".md");
  const double duration_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - diag_start_time).count();

  std::ofstream json(json_path);
  json << "{\n"
       << "  \"source\": \"fast_livo_internal_callbacks\",\n"
       << "  \"duration_s\": " << duration_s << ",\n"
       << "  \"topics\": {\n";
  writeJsonTopic(json, diag_imu, true);
  writeJsonTopic(json, diag_cloud, true);
  writeJsonTopic(json, diag_image, false);
  json << "  }\n"
       << "}\n";

  std::ofstream md(md_path);
  md << "# FAST-LIVO2 Internal Topic Diagnostics\n\n"
     << "- Source: FAST-LIVO2 callbacks, no external image/cloud subscription\n"
     << "- Duration: `" << std::fixed << std::setprecision(1) << duration_s << "s`\n\n"
     << "| Topic | Count | Hz | Header Hz | Arrival p95 ms | Header p95 ms |\n"
     << "|---|---:|---:|---:|---:|---:|\n";
  writeMdTopic(md, diag_imu);
  writeMdTopic(md, diag_cloud);
  writeMdTopic(md, diag_image);

  std::cout << GREEN << "Internal topic report written to: " << json_path << " and " << md_path << RESET << std::endl;
}

void LIVMapper::run() 
{
  rclcpp::Rate rate(5000);
  while (rclcpp::ok())
  {
    try
    {
      rclcpp::spin_some(this->get_node_base_interface());
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
      RCLCPP_WARN(this->get_logger(), "Stopping FAST-LIVO2 after ROS context error during shutdown: %s", e.what());
      break;
    }
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
  }
  writeInternalTopicReport();
  savePCD();
  saveFinalMap();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && fast_livo::stampToSec(prop_imu_buffer.front().header.stamp) < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = fast_livo::stampToSec(prop_imu_buffer[i].header.stamp) - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = fast_livo::stampToSec(newest_imu.header.stamp) - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstSharedPtr &msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = fast_livo::stampToSec(msg->header.stamp) + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;
  recordInternalTopicSample(diag_cloud, cur_head_time);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstSharedPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  auto msg = std::make_shared<livox_ros_driver::CustomMsg>(*msg_in);
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = fast_livo::stampFromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - fast_livo::stampToSec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - fast_livo::stampToSec(msg->header.stamp);
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = fast_livo::stampToSec(msg->header.stamp);
  ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    ROS_ERROR("Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;
  recordInternalTopicSample(diag_cloud, cur_head_time);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstSharedPtr &msg_in)
{
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
  auto msg = std::make_shared<sensor_msgs::Imu>(*msg_in);
  msg->header.stamp = fast_livo::stampFromSec(fast_livo::stampToSec(msg->header.stamp) - imu_time_offset);
  double timestamp = fast_livo::stampToSec(msg->header.stamp);

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = fast_livo::stampFromSec(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  recordInternalTopicSample(diag_imu, timestamp);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
  if (!img_en) return;
  auto msg = std::make_shared<sensor_msgs::Image>(*msg_in);
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = fast_livo::stampFromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  msg->header.stamp.toSec();
  double msg_header_time = fast_livo::stampToSec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  ROS_INFO("Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    ROS_ERROR("image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    ROS_WARN("Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);
  recordInternalTopicSample(diag_image, img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (fast_livo::stampToSec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = fast_livo::stampToSec(imu_buffer.back()->header.stamp);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        ROS_ERROR("[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // ROS_ERROR("lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (fast_livo::stampToSec(imu_buffer.front()->header.stamp) > m.lio_time) break;

        if (fast_livo::stampToSec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // fast_livo::stampToSec(imu_buffer.front()->header.stamp));
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = fast_livo::stampToSec(imu_buffer.front()->header.stamp);

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = fast_livo::stampToSec(imu_buffer.front()->header.stamp);
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   fast_livo::stampToSec(imu_buffer.front()->header.stamp));
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  ROS_ERROR("out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = fast_livo::timeToMsg(this->now());
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::PointCloud2 laserCloudmsg;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
  {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = fast_livo::timeToMsg(this->now());
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (final_map_save_en)
  {
    if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      if (!pcl_w_wait_pub->empty()) *pcl_final_map_intensity += *pcl_w_wait_pub;
    }
    if (slam_mode_ == LIVO)
    {
      if (LidarMeasures.lio_vio_flg == VIO && !laserCloudWorldRGB->empty()) *pcl_final_map_rgb += *laserCloudWorldRGB;
    }
  }

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;
    const bool pose_trigger = (pcd_save_trigger_mode == "pose_delta");
    bool pcd_saved = false;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (pose_trigger)
        {
          const bool has_rgb_cloud = slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO && !laserCloudWorldRGB->empty();
          const bool has_intensity_cloud = slam_mode_ != LIVO && !pcl_w_wait_pub->empty();
          if ((has_rgb_cloud || has_intensity_cloud) &&
              shouldSaveForPose(last_pcd_save_pose_valid, last_pcd_save_pos, last_pcd_save_rot, last_pcd_save_time, update_time))
          {
            string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "pose-gated scan saved to " << all_points_dir << endl;
            if (has_rgb_cloud)
            {
              pcd_writer.writeBinary(all_points_dir, *laserCloudWorldRGB);
            }
            else
            {
              pcd_writer.writeBinary(all_points_dir, *pcl_w_wait_pub);
            }
            pcd_saved = true;
          }
        }
        else
        {
          if (slam_mode_ == LIVO)
          {
            *pcl_wait_save += *laserCloudWorldRGB;
          }
          else
          {
            *pcl_wait_save_intensity += *pcl_w_wait_pub;
          }
          if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        }
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          if (pose_trigger)
          {
            if (!laserCloudBody->empty() &&
                shouldSaveForPose(last_pcd_save_pose_valid, last_pcd_save_pos, last_pcd_save_rot, last_pcd_save_time, update_time))
            {
              string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));
              pcl::PCDWriter pcd_writer;
              cout << "pose-gated body frame scan saved to " << all_points_dir << endl;
              pcd_writer.writeBinary(all_points_dir, *laserCloudBody);
              pcd_saved = true;
            }
          }
          else
          {
            *pcl_wait_save_intensity += *laserCloudBody;
            scan_wait_num++;
            cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
          }
        }
        if (!pose_trigger) pcd_save_interval = 1;
        
        break;

      default:
        if (!pose_trigger)
        {
          pcd_save_interval = 1;
          scan_wait_num++;
        }
        break;
    }
    if (!pose_trigger && (pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
      pcd_saved = true;
    }
    
    if(pcd_saved)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
      markPcdSaved(update_time);
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    const bool pose_trigger = (img_save_trigger_mode == "pose_delta");
    bool save_image = false;
    if (pose_trigger)
    {
      save_image = shouldSaveForPose(last_image_save_pose_valid, last_image_save_pos, last_image_save_rot, last_image_save_time, update_time);
    }
    else
    {
      img_wait_num++;
      save_image = img_save_interval > 0 && img_wait_num >= img_save_interval;
    }

    if (save_image)
    {
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() + string(".png"), vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
      markImageSaved(update_time);
    }
  }

  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = fast_livo::timeToMsg(this->now());
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = fast_livo::timeToMsg(this->now());
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = fast_livo::timeToMsg(this->now());
  set_posestamp(odomAftMapped.pose.pose);

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = odomAftMapped.header.stamp;
  transform.header.frame_id = "camera_init";
  transform.child_frame_id = "aft_mapped";
  transform.transform.translation.x = _state.pos_end(0);
  transform.transform.translation.y = _state.pos_end(1);
  transform.transform.translation.z = _state.pos_end(2);
  transform.transform.rotation = geoQuat;
  tf_broadcaster_->sendTransform(transform);
  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = fast_livo::timeToMsg(this->now());
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::Path>::SharedPtr &pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = fast_livo::timeToMsg(this->now());
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}

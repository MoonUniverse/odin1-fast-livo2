cmake_minimum_required(VERSION 3.8)
project(fast_livo)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-pthread -fexceptions)

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    add_compile_options("$<$<CONFIG:Release>:-O3;-mcpu=native;-mtune=native;-ffast-math>")
    message(STATUS "Using 64-bit ARM optimizations")
  else()
    add_compile_options("$<$<CONFIG:Release>:-O3;-mcpu=native;-mtune=native;-mfpu=neon;-ffast-math>")
    message(STATUS "Using 32-bit ARM optimizations")
  endif()
  add_compile_definitions(ARM_ARCH)
else()
  add_compile_options("$<$<CONFIG:Release>:-O3;-march=native;-mtune=native;-funroll-loops>")
  message(STATUS "Using x86 optimizations")
  add_compile_definitions(X86_ARCH)
endif()

add_compile_definitions(ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")
if(N GREATER 4)
  add_compile_definitions(MP_EN MP_PROC_NUM=4)
elseif(N GREATER 1)
  add_compile_definitions(MP_EN MP_PROC_NUM=${N})
else()
  add_compile_definitions(MP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(vikit_common REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenMP QUIET)
find_package(mimalloc QUIET)

set(FAST_LIVO_ROS_DEPS
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions
  cv_bridge
  image_transport
  livox_ros_driver2
  vikit_common
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)

foreach(target vio lio pre imu_proc laser_mapping)
  ament_target_dependencies(${target} ${FAST_LIVO_ROS_DEPS})
  target_link_libraries(${target}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  if(OpenMP_CXX_FOUND)
    target_link_libraries(${target} OpenMP::OpenMP_CXX)
  endif()
endforeach()

target_link_libraries(laser_mapping vio lio pre imu_proc)

add_executable(fastlivo_mapping src/main.cpp)
ament_target_dependencies(fastlivo_mapping ${FAST_LIVO_ROS_DEPS})
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlivo_mapping OpenMP::OpenMP_CXX)
endif()

if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

install(TARGETS
  fastlivo_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/record_imu_static.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz_cfg
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${FAST_LIVO_ROS_DEPS})
ament_package()

#ifndef VIKIT_PINHOLE_CAMERA_H_
#define VIKIT_PINHOLE_CAMERA_H_

#include <array>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vikit/abstract_camera.h>

namespace vk
{

class PinholeCamera : public AbstractCamera
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeCamera(
      int width, int height, double fx, double fy, double cx, double cy, double scale,
      double d0 = 0.0, double d1 = 0.0, double d2 = 0.0, double d3 = 0.0)
      : AbstractCamera(width, height, fx, fy, cx, cy, scale), distortion_{d0, d1, d2, d3}
  {
  }

  Eigen::Vector3d cam2world(double x, double y) const override
  {
    Eigen::Vector3d xyz((x - cx_) / fx_, (y - cy_) / fy_, 1.0);
    return xyz.normalized();
  }

  Eigen::Vector2d world2cam(const Eigen::Vector3d &xyz) const override
  {
    const double z = std::abs(xyz.z()) > 1e-12 ? xyz.z() : 1e-12;
    return Eigen::Vector2d(fx_ * xyz.x() / z + cx_, fy_ * xyz.y() / z + cy_);
  }

  void undistortImage(const cv::Mat &src, cv::Mat &dst) const
  {
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 4) << distortion_[0], distortion_[1], distortion_[2], distortion_[3]);
    cv::undistort(src, dst, camera_matrix, dist_coeffs);
  }

protected:
  std::array<double, 4> distortion_;
};

class EquidistantCamera : public PinholeCamera
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EquidistantCamera(
      int width, int height, double fx, double fy, double cx, double cy, double scale,
      double d0 = 0.0, double d1 = 0.0, double d2 = 0.0, double d3 = 0.0)
      : PinholeCamera(width, height, fx, fy, cx, cy, scale, d0, d1, d2, d3)
  {
  }

  void undistortImage(const cv::Mat &src, cv::Mat &dst) const
  {
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    cv::Mat dist_coeffs = (cv::Mat_<double>(4, 1) << distortion_[0], distortion_[1], distortion_[2], distortion_[3]);
    cv::fisheye::undistortImage(src, dst, camera_matrix, dist_coeffs, camera_matrix, src.size());
  }
};

} // namespace vk

#endif // VIKIT_PINHOLE_CAMERA_H_

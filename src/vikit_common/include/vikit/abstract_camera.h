#ifndef VIKIT_ABSTRACT_CAMERA_H_
#define VIKIT_ABSTRACT_CAMERA_H_

#include <Eigen/Core>

namespace vk
{

class AbstractCamera
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AbstractCamera(int width, int height, double fx, double fy, double cx, double cy, double scale)
      : width_(width), height_(height), fx_(fx), fy_(fy), cx_(cx), cy_(cy), scale_(scale)
  {
  }

  virtual ~AbstractCamera() = default;

  virtual Eigen::Vector3d cam2world(double x, double y) const = 0;
  virtual Eigen::Vector3d cam2world(const Eigen::Vector2d &px) const { return cam2world(px.x(), px.y()); }
  virtual Eigen::Vector2d world2cam(const Eigen::Vector3d &xyz) const = 0;

  bool isInFrame(const Eigen::Vector2i &px, int boundary = 0) const
  {
    return px.x() >= boundary && px.y() >= boundary &&
           px.x() < width_ - boundary && px.y() < height_ - boundary;
  }

  int width() const { return width_; }
  int height() const { return height_; }
  double fx() const { return fx_; }
  double fy() const { return fy_; }
  double cx() const { return cx_; }
  double cy() const { return cy_; }
  double scale() const { return scale_; }

protected:
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double scale_;
};

} // namespace vk

#endif // VIKIT_ABSTRACT_CAMERA_H_

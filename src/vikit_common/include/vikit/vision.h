#ifndef VIKIT_VISION_H_
#define VIKIT_VISION_H_

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace vk
{

inline void halfSample(const cv::Mat &in, cv::Mat &out)
{
  cv::resize(in, out, out.size(), 0.0, 0.0, cv::INTER_AREA);
}

inline float interpolateMat_8u(const cv::Mat &mat, float u, float v)
{
  const int x = static_cast<int>(std::floor(u));
  const int y = static_cast<int>(std::floor(v));
  if (x < 0 || y < 0 || x >= mat.cols - 1 || y >= mat.rows - 1) { return 0.0f; }

  const float dx = u - x;
  const float dy = v - y;
  const uint8_t *row0 = mat.ptr<uint8_t>(y);
  const uint8_t *row1 = mat.ptr<uint8_t>(y + 1);
  return (1.0f - dx) * (1.0f - dy) * row0[x] +
         dx * (1.0f - dy) * row0[x + 1] +
         (1.0f - dx) * dy * row1[x] +
         dx * dy * row1[x + 1];
}

inline float shiTomasiScore(const cv::Mat &img, int u, int v)
{
  constexpr int halfbox = 4;
  if (u < halfbox || v < halfbox || u >= img.cols - halfbox || v >= img.rows - halfbox) { return 0.0f; }

  float dxx = 0.0f;
  float dyy = 0.0f;
  float dxy = 0.0f;
  for (int y = v - halfbox; y <= v + halfbox; ++y)
  {
    const uint8_t *row = img.ptr<uint8_t>(y);
    for (int x = u - halfbox; x <= u + halfbox; ++x)
    {
      const float dx = static_cast<float>(row[x + 1]) - static_cast<float>(row[x - 1]);
      const float dy = static_cast<float>(img.ptr<uint8_t>(y + 1)[x]) - static_cast<float>(img.ptr<uint8_t>(y - 1)[x]);
      dxx += dx * dx;
      dyy += dy * dy;
      dxy += dx * dy;
    }
  }

  const float trace = dxx + dyy;
  const float det = dxx * dyy - dxy * dxy;
  const float discr = std::max(0.0f, trace * trace - 4.0f * det);
  return 0.5f * (trace - std::sqrt(discr));
}

} // namespace vk

#endif // VIKIT_VISION_H_

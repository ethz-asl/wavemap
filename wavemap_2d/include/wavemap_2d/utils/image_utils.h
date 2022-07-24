#ifndef WAVEMAP_2D_UTILS_IMAGE_UTILS_H_
#define WAVEMAP_2D_UTILS_IMAGE_UTILS_H_

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
template <typename T>
static cv::Mat MatrixToImage(const MatrixT<T>& matrix,
                             FloatingPoint lower_bound,
                             FloatingPoint upper_bound, bool use_color) {
  if (!matrix.size()) {
    return cv::Mat{};
  }

  cv::Mat image;
  cv::eigen2cv(matrix, image);
  const FloatingPoint scale_factor = 255.f / (upper_bound - lower_bound);
  image.convertTo(image, CV_8UC1, scale_factor, -lower_bound * scale_factor);
  cv::flip(image, image, -1);
  cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);

  if (use_color) {
    cv::applyColorMap(image, image, cv::ColormapTypes::COLORMAP_PARULA);
  }

  return image;
}

inline void ShowImage(const cv::Mat& image, int delay_ms = 1) {
  if (image.empty()) {
    return;
  }

  cv::namedWindow("Grid map", cv::WINDOW_NORMAL);
  cv::setWindowProperty("Grid map", cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  cv::imshow("Grid map", image);
  cv::waitKey(delay_ms);
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_UTILS_IMAGE_UTILS_H_

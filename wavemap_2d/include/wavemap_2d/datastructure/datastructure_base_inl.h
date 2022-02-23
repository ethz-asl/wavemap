#ifndef WAVEMAP_2D_DATASTRUCTURE_DATASTRUCTURE_BASE_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_DATASTRUCTURE_BASE_INL_H_

namespace wavemap_2d {
template <typename T>
cv::Mat DataStructureBase::MatrixToImage(const MatrixT<T>& matrix,
                                         FloatingPoint lower_bound,
                                         FloatingPoint upper_bound,
                                         bool use_color) {
  if (!matrix.size()) {
    return cv::Mat{};
  }

  cv::Mat image;
  cv::eigen2cv(matrix, image);
  image.convertTo(image, CV_8UC1, 255.f / (upper_bound - lower_bound),
                  -lower_bound);
  cv::flip(image, image, -1);
  cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);

  if (use_color) {
    cv::applyColorMap(image, image, cv::ColormapTypes::COLORMAP_PARULA);
  }

  return image;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_DATASTRUCTURE_BASE_INL_H_

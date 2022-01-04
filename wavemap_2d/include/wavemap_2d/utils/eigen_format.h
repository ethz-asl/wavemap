#ifndef WAVEMAP_2D_UTILS_EIGEN_FORMAT_H_
#define WAVEMAP_2D_UTILS_EIGEN_FORMAT_H_

#include <Eigen/Eigen>

namespace wavemap_2d {
struct EigenFormat {
  static const Eigen::IOFormat kOneLine;

  template <typename Derived>
  static Eigen::WithFormat<Derived> oneLine(const Derived& matrix) {
    return Eigen::WithFormat<Derived>(matrix, kOneLine);
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_UTILS_EIGEN_FORMAT_H_

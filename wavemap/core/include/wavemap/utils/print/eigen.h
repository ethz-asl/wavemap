#ifndef WAVEMAP_UTILS_PRINT_EIGEN_H_
#define WAVEMAP_UTILS_PRINT_EIGEN_H_

#include <Eigen/Eigen>

namespace wavemap::print::eigen {
template <typename Derived>
static Eigen::WithFormat<Derived> oneLine(const Derived& matrix) {
  static const Eigen::IOFormat kOneLine =
      Eigen::IOFormat(4, Eigen::DontAlignCols, ", ", ", ", "", "", " [", "]");
  return Eigen::WithFormat<Derived>(matrix, kOneLine);
}
}  // namespace wavemap::print::eigen

#endif  // WAVEMAP_UTILS_PRINT_EIGEN_H_

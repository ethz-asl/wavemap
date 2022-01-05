#ifndef WAVEMAP_2D_DISCRETE_WAVELET_TRANSFORM_H_
#define WAVEMAP_2D_DISCRETE_WAVELET_TRANSFORM_H_

#include <Eigen/Eigen>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
class DiscreteWaveletTransform {
 public:
  template <typename T>
  using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

  template <typename T>
  static void ForwardHaarNaive(Matrix<T>& matrix, int n_passes) {  // NOLINT
    static_assert(!matrix.IsRowMajor);

    for (int pass_idx = 0; pass_idx < n_passes; ++pass_idx) {
      Eigen::Index relevant_rows = matrix.rows() / std::exp2(pass_idx);
      Eigen::Index relevant_cols = matrix.cols() / std::exp2(pass_idx);
      matrix.block(0, 0, relevant_rows, relevant_cols) =
          ForwardHaarNaiveSingle<T>(
              matrix.block(0, 0, relevant_rows, relevant_cols));
    }
  }

 private:
  template <typename T>
  static Matrix<T> ForwardHaarNaiveSingle(Matrix<T> matrix) {
    const T kSqrt2Inv = 0.70710678118;

    // TODO(victorr): Handle odd numbers of rows and columns
    Eigen::Index rows = matrix.rows();
    Eigen::Index cols = matrix.cols();
    // CHECK(rows % 2 == 0);
    // CHECK(cols % 2 == 0);

    using ColStride = Eigen::OuterStride<>;
    Matrix<T> even_cols = Eigen::Map<Matrix<T>, 0, ColStride>(
        matrix.data(), rows, cols / 2, ColStride(2 * rows));
    Matrix<T> odd_cols = Eigen::Map<Matrix<T>, 0, ColStride>(
        matrix.data() + rows, rows, cols / 2, ColStride(2 * rows));
    matrix.block(0, 0, rows, cols / 2) = (even_cols + odd_cols) * kSqrt2Inv;
    matrix.block(0, cols / 2, rows, cols / 2) =
        (even_cols - odd_cols) * kSqrt2Inv;

    using RowStride = Eigen::Stride<Eigen::Dynamic, 2>;
    Matrix<T> even_rows = Eigen::Map<Matrix<T>, 0, RowStride>(
        matrix.data(), rows / 2, cols, RowStride(rows, 2));
    Matrix<T> odd_rows = Eigen::Map<Matrix<T>, 0, RowStride>(
        matrix.data() + 1, rows / 2, cols, RowStride(rows, 2));
    matrix.block(0, 0, rows / 2, cols) = (even_rows + odd_rows) * kSqrt2Inv;
    matrix.block(rows / 2, 0, rows / 2, cols) =
        (even_rows - odd_rows) * kSqrt2Inv;

    return matrix;
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DISCRETE_WAVELET_TRANSFORM_H_

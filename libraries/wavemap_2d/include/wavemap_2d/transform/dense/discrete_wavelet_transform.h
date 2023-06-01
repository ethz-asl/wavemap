#ifndef WAVEMAP_2D_TRANSFORM_DENSE_DISCRETE_WAVELET_TRANSFORM_H_
#define WAVEMAP_2D_TRANSFORM_DENSE_DISCRETE_WAVELET_TRANSFORM_H_

#include <utility>

#include <Eigen/Eigen>
#include <wavemap_common/common.h>

namespace wavemap {
template <typename T>
class DiscreteWaveletTransform {
 public:
  using ValueType = T;

  virtual ~DiscreteWaveletTransform() = default;

  void forward(MatrixT<T>& matrix, int n_passes) const {
    CHECK(!matrix.IsRowMajor);
    CHECK_GT(n_passes, 0) << "The number of wavelet transform passes must be a "
                             "positive integer, but received "
                          << n_passes << " instead.";
    const auto min_divisor = static_cast<Eigen::Index>(std::exp2(n_passes - 1));
    CHECK_EQ(matrix.rows() % min_divisor, 0)
        << "The number of rows (" << matrix.rows()
        << ") must be divisible by 2^(n_passes-1) (" << min_divisor << ")";
    CHECK_EQ(matrix.cols() % min_divisor, 0)
        << "The number of columns (" << matrix.cols()
        << ") must be divisible by 2^(n_passes-1) (" << min_divisor << ")";

    for (int pass_idx = 0; pass_idx < n_passes; ++pass_idx) {
      Eigen::Index relevant_rows = matrix.rows() / std::exp2(pass_idx);
      Eigen::Index relevant_cols = matrix.cols() / std::exp2(pass_idx);
      matrix.block(0, 0, relevant_rows, relevant_cols) =
          singleForwardPass(matrix.block(0, 0, relevant_rows, relevant_cols));
    }
  }

  void backward(MatrixT<T>& matrix, int n_passes) const {
    CHECK(!matrix.IsRowMajor);
    CHECK_GT(n_passes, 0) << "The number of wavelet transform passes must be a "
                             "positive integer, but received "
                          << n_passes << " instead.";
    const auto min_divisor = static_cast<Eigen::Index>(std::exp2(n_passes - 1));
    CHECK_EQ(matrix.rows() % min_divisor, 0)
        << "The number of rows (" << matrix.rows()
        << ") must be divisible by 2^(n_passes-1) (" << min_divisor << ")";
    CHECK_EQ(matrix.cols() % min_divisor, 0)
        << "The number of columns (" << matrix.cols()
        << ") must be divisible by 2^(n_passes-1) (" << min_divisor << ")";

    for (int pass_idx = n_passes - 1; 0 <= pass_idx; --pass_idx) {
      Eigen::Index relevant_rows = matrix.rows() / std::exp2(pass_idx);
      Eigen::Index relevant_cols = matrix.cols() / std::exp2(pass_idx);
      matrix.block(0, 0, relevant_rows, relevant_cols) =
          singleBackwardPass(matrix.block(0, 0, relevant_rows, relevant_cols));
    }
  }

 private:
  virtual MatrixT<T> singleForwardPass(MatrixT<T> matrix) const = 0;
  virtual MatrixT<T> singleBackwardPass(MatrixT<T> matrix) const = 0;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_TRANSFORM_DENSE_DISCRETE_WAVELET_TRANSFORM_H_

#ifndef WAVEMAP_2D_TRANSFORM_DISCRETE_WAVELET_TRANSFORM_H_
#define WAVEMAP_2D_TRANSFORM_DISCRETE_WAVELET_TRANSFORM_H_

#include <utility>

#include <Eigen/Eigen>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
template <typename T>
class DiscreteWaveletTransform {
 public:
  // TODO(victorr): Check that num rows and columns are divisible by 2^n_pass
  void forward(Matrix<T>& matrix, int n_passes) const {
    CHECK(!matrix.IsRowMajor);

    for (int pass_idx = 0; pass_idx < n_passes; ++pass_idx) {
      Eigen::Index relevant_rows = matrix.rows() / std::exp2(pass_idx);
      Eigen::Index relevant_cols = matrix.cols() / std::exp2(pass_idx);
      matrix.block(0, 0, relevant_rows, relevant_cols) =
          singleForwardPass(matrix.block(0, 0, relevant_rows, relevant_cols));
    }
  }

  // TODO(victorr): Check that num rows and columns are divisible by 2^n_pass
  void backward(Matrix<T>& matrix, int n_passes) const {
    CHECK(!matrix.IsRowMajor);

    for (int pass_idx = n_passes - 1; 0 <= pass_idx; --pass_idx) {
      Eigen::Index relevant_rows = matrix.rows() / std::exp2(pass_idx);
      Eigen::Index relevant_cols = matrix.cols() / std::exp2(pass_idx);
      matrix.block(0, 0, relevant_rows, relevant_cols) =
          singleBackwardPass(matrix.block(0, 0, relevant_rows, relevant_cols));
    }
  }

 protected:
  virtual Matrix<T> singleForwardPass(Matrix<T> matrix) const = 0;
  virtual Matrix<T> singleBackwardPass(Matrix<T> matrix) const = 0;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_TRANSFORM_DISCRETE_WAVELET_TRANSFORM_H_

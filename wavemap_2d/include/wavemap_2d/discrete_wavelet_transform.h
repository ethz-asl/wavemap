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
    CHECK(!matrix.IsRowMajor);

    for (int pass_idx = 0; pass_idx < n_passes; ++pass_idx) {
      Eigen::Index relevant_rows = matrix.rows() / std::exp2(pass_idx);
      Eigen::Index relevant_cols = matrix.cols() / std::exp2(pass_idx);
      matrix.block(0, 0, relevant_rows, relevant_cols) =
          ForwardHaarNaiveSingle<T>(
              matrix.block(0, 0, relevant_rows, relevant_cols));
    }
  }

  template <typename T>
  static void BackwardHaarNaive(Matrix<T>& matrix, int n_passes) {  // NOLINT
    CHECK(!matrix.IsRowMajor);

    for (int pass_idx = n_passes - 1; 0 <= pass_idx; --pass_idx) {
      Eigen::Index relevant_rows = matrix.rows() / std::exp2(pass_idx);
      Eigen::Index relevant_cols = matrix.cols() / std::exp2(pass_idx);
      matrix.block(0, 0, relevant_rows, relevant_cols) =
          BackwardHaarNaiveSingle<T>(
              matrix.block(0, 0, relevant_rows, relevant_cols));
    }
  }

 private:
  template <typename T>
  static constexpr T kSqrt2Inv = 0.707106781186547524400844362104;

  // TODO(victorr): Handle odd numbers of rows and columns
  template <typename T>
  static Matrix<T> ForwardHaarNaiveSingle(Matrix<T> matrix) {
    Eigen::Index rows = matrix.rows();
    Eigen::Index cols = matrix.cols();

    using ColStride = Eigen::OuterStride<>;
    Matrix<T> even_cols = Eigen::Map<Matrix<T>, 0, ColStride>(
        matrix.data(), rows, cols / 2, ColStride(2 * rows));
    Matrix<T> odd_cols = Eigen::Map<Matrix<T>, 0, ColStride>(
        matrix.data() + rows, rows, cols / 2, ColStride(2 * rows));
    typename Matrix<T>::BlockXpr V1 = matrix.block(0, 0, rows, cols / 2);
    typename Matrix<T>::BlockXpr V2 = matrix.block(0, cols / 2, rows, cols / 2);
    V1 = (even_cols + odd_cols) * kSqrt2Inv<T>;
    V2 = (even_cols - odd_cols) * kSqrt2Inv<T>;

    using RowStride = Eigen::Stride<Eigen::Dynamic, 2>;
    Matrix<T> even_rows = Eigen::Map<Matrix<T>, 0, RowStride>(
        matrix.data(), rows / 2, cols, RowStride(rows, 2));
    Matrix<T> odd_rows = Eigen::Map<Matrix<T>, 0, RowStride>(
        matrix.data() + 1, rows / 2, cols, RowStride(rows, 2));
    typename Matrix<T>::BlockXpr H1 = matrix.block(0, 0, rows / 2, cols);
    typename Matrix<T>::BlockXpr H2 = matrix.block(rows / 2, 0, rows / 2, cols);
    H1 = (even_rows + odd_rows) * kSqrt2Inv<T>;
    H2 = (even_rows - odd_rows) * kSqrt2Inv<T>;

    return matrix;
  }

  // TODO(victorr): Handle odd numbers of rows and columns
  template <typename T>
  static Matrix<T> BackwardHaarNaiveSingle(Matrix<T> matrix) {
    Eigen::Index rows = matrix.rows();
    Eigen::Index cols = matrix.cols();

    using ColStride = Eigen::OuterStride<>;
    Eigen::Map<Matrix<T>, 0, ColStride> even_cols(matrix.data(), rows, cols / 2,
                                                  ColStride(2 * rows));
    Eigen::Map<Matrix<T>, 0, ColStride> odd_cols(matrix.data() + rows, rows,
                                                 cols / 2, ColStride(2 * rows));
    Matrix<T> V1 = matrix.block(0, 0, rows, cols / 2);
    Matrix<T> V2 = matrix.block(0, cols / 2, rows, cols / 2);
    even_cols = (V1 + V2) * kSqrt2Inv<T>;
    odd_cols = (V1 - V2) * kSqrt2Inv<T>;

    using RowStride = Eigen::Stride<Eigen::Dynamic, 2>;
    Eigen::Map<Matrix<T>, 0, RowStride> even_rows(matrix.data(), rows / 2, cols,
                                                  RowStride(rows, 2));
    Eigen::Map<Matrix<T>, 0, RowStride> odd_rows(matrix.data() + 1, rows / 2,
                                                 cols, RowStride(rows, 2));
    Matrix<T> H1 = matrix.block(0, 0, rows / 2, cols);
    Matrix<T> H2 = matrix.block(rows / 2, 0, rows / 2, cols);
    even_rows = (H1 + H2) * kSqrt2Inv<T>;
    odd_rows = (H1 - H2) * kSqrt2Inv<T>;

    return matrix;
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DISCRETE_WAVELET_TRANSFORM_H_

#ifndef WAVEMAP_2D_TRANSFORM_NAIVE_HAAR_H_
#define WAVEMAP_2D_TRANSFORM_NAIVE_HAAR_H_

#include "wavemap_2d/transform/discrete_wavelet_transform.h"

namespace wavemap_2d {
template <typename T>
class NaiveHaar : public DiscreteWaveletTransform<T> {
 protected:
  static constexpr T kSqrt2Inv = 0.707106781186547524400844362104;

  Matrix<T> singleForwardPass(Matrix<T> matrix) const override {
    const Eigen::Index rows = matrix.rows();
    const Eigen::Index cols = matrix.cols();

    using ColStride = Eigen::OuterStride<>;
    Matrix<T> even_cols = Eigen::Map<Matrix<T>, 0, ColStride>(
        matrix.data(), rows, cols / 2, ColStride(2 * rows));
    Matrix<T> odd_cols = Eigen::Map<Matrix<T>, 0, ColStride>(
        matrix.data() + rows, rows, cols / 2, ColStride(2 * rows));
    typename Matrix<T>::BlockXpr V1 = matrix.block(0, 0, rows, cols / 2);
    typename Matrix<T>::BlockXpr V2 = matrix.block(0, cols / 2, rows, cols / 2);
    V1 = (even_cols + odd_cols) * kSqrt2Inv;
    V2 = (even_cols - odd_cols) * kSqrt2Inv;

    using RowStride = Eigen::Stride<Eigen::Dynamic, 2>;
    Matrix<T> even_rows = Eigen::Map<Matrix<T>, 0, RowStride>(
        matrix.data(), rows / 2, cols, RowStride(rows, 2));
    Matrix<T> odd_rows = Eigen::Map<Matrix<T>, 0, RowStride>(
        matrix.data() + 1, rows / 2, cols, RowStride(rows, 2));
    typename Matrix<T>::BlockXpr H1 = matrix.block(0, 0, rows / 2, cols);
    typename Matrix<T>::BlockXpr H2 = matrix.block(rows / 2, 0, rows / 2, cols);
    H1 = (even_rows + odd_rows) * kSqrt2Inv;
    H2 = (even_rows - odd_rows) * kSqrt2Inv;

    return matrix;
  }

  Matrix<T> singleBackwardPass(Matrix<T> matrix) const override {
    const Eigen::Index rows = matrix.rows();
    const Eigen::Index cols = matrix.cols();

    using ColStride = Eigen::OuterStride<>;
    Eigen::Map<Matrix<T>, 0, ColStride> even_cols(matrix.data(), rows, cols / 2,
                                                  ColStride(2 * rows));
    Eigen::Map<Matrix<T>, 0, ColStride> odd_cols(matrix.data() + rows, rows,
                                                 cols / 2, ColStride(2 * rows));
    Matrix<T> V1 = matrix.block(0, 0, rows, cols / 2);
    Matrix<T> V2 = matrix.block(0, cols / 2, rows, cols / 2);
    even_cols = (V1 + V2) * kSqrt2Inv;
    odd_cols = (V1 - V2) * kSqrt2Inv;

    using RowStride = Eigen::Stride<Eigen::Dynamic, 2>;
    Eigen::Map<Matrix<T>, 0, RowStride> even_rows(matrix.data(), rows / 2, cols,
                                                  RowStride(rows, 2));
    Eigen::Map<Matrix<T>, 0, RowStride> odd_rows(matrix.data() + 1, rows / 2,
                                                 cols, RowStride(rows, 2));
    Matrix<T> H1 = matrix.block(0, 0, rows / 2, cols);
    Matrix<T> H2 = matrix.block(rows / 2, 0, rows / 2, cols);
    even_rows = (H1 + H2) * kSqrt2Inv;
    odd_rows = (H1 - H2) * kSqrt2Inv;

    return matrix;
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_TRANSFORM_NAIVE_HAAR_H_

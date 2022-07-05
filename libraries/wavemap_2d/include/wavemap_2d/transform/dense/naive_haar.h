#ifndef WAVEMAP_2D_TRANSFORM_DENSE_NAIVE_HAAR_H_
#define WAVEMAP_2D_TRANSFORM_DENSE_NAIVE_HAAR_H_

#include "wavemap_2d/transform/dense/discrete_wavelet_transform.h"

namespace wavemap {
template <typename T>
class NaiveHaar : public DiscreteWaveletTransform<T> {
 private:
  MatrixT<T> singleForwardPass(MatrixT<T> matrix) const override {
    const Eigen::Index rows = matrix.rows();
    const Eigen::Index cols = matrix.cols();

    using ColStride = Eigen::OuterStride<>;
    MatrixT<T> even_cols = Eigen::Map<MatrixT<T>, 0, ColStride>(
        matrix.data(), rows, cols / 2, ColStride(2 * rows));
    MatrixT<T> odd_cols = Eigen::Map<MatrixT<T>, 0, ColStride>(
        matrix.data() + rows, rows, cols / 2, ColStride(2 * rows));
    typename MatrixT<T>::BlockXpr V1 = matrix.block(0, 0, rows, cols / 2);
    typename MatrixT<T>::BlockXpr V2 =
        matrix.block(0, cols / 2, rows, cols / 2);
    V1 = (even_cols + odd_cols) * constants<T>::kSqrt2Inv;
    V2 = (even_cols - odd_cols) * constants<T>::kSqrt2Inv;

    using RowStride = Eigen::Stride<Eigen::Dynamic, 2>;
    MatrixT<T> even_rows = Eigen::Map<MatrixT<T>, 0, RowStride>(
        matrix.data(), rows / 2, cols, RowStride(rows, 2));
    MatrixT<T> odd_rows = Eigen::Map<MatrixT<T>, 0, RowStride>(
        matrix.data() + 1, rows / 2, cols, RowStride(rows, 2));
    typename MatrixT<T>::BlockXpr H1 = matrix.block(0, 0, rows / 2, cols);
    typename MatrixT<T>::BlockXpr H2 =
        matrix.block(rows / 2, 0, rows / 2, cols);
    H1 = (even_rows + odd_rows) * constants<T>::kSqrt2Inv;
    H2 = (even_rows - odd_rows) * constants<T>::kSqrt2Inv;

    return matrix;
  }

  MatrixT<T> singleBackwardPass(MatrixT<T> matrix) const override {
    const Eigen::Index rows = matrix.rows();
    const Eigen::Index cols = matrix.cols();

    using ColStride = Eigen::OuterStride<>;
    Eigen::Map<MatrixT<T>, 0, ColStride> even_cols(
        matrix.data(), rows, cols / 2, ColStride(2 * rows));
    Eigen::Map<MatrixT<T>, 0, ColStride> odd_cols(
        matrix.data() + rows, rows, cols / 2, ColStride(2 * rows));
    MatrixT<T> V1 = matrix.block(0, 0, rows, cols / 2);
    MatrixT<T> V2 = matrix.block(0, cols / 2, rows, cols / 2);
    even_cols = (V1 + V2) * constants<T>::kSqrt2Inv;
    odd_cols = (V1 - V2) * constants<T>::kSqrt2Inv;

    using RowStride = Eigen::Stride<Eigen::Dynamic, 2>;
    Eigen::Map<MatrixT<T>, 0, RowStride> even_rows(matrix.data(), rows / 2,
                                                   cols, RowStride(rows, 2));
    Eigen::Map<MatrixT<T>, 0, RowStride> odd_rows(matrix.data() + 1, rows / 2,
                                                  cols, RowStride(rows, 2));
    MatrixT<T> H1 = matrix.block(0, 0, rows / 2, cols);
    MatrixT<T> H2 = matrix.block(rows / 2, 0, rows / 2, cols);
    even_rows = (H1 + H2) * constants<T>::kSqrt2Inv;
    odd_rows = (H1 - H2) * constants<T>::kSqrt2Inv;

    return matrix;
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_TRANSFORM_DENSE_NAIVE_HAAR_H_

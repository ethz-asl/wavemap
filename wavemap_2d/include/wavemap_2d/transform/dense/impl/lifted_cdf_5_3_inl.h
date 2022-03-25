#ifndef WAVEMAP_2D_TRANSFORM_DENSE_IMPL_LIFTED_CDF_5_3_INL_H_
#define WAVEMAP_2D_TRANSFORM_DENSE_IMPL_LIFTED_CDF_5_3_INL_H_

#include <vector>

namespace wavemap_2d {
template <typename T>
MatrixT<T> LiftedCDF53<T>::singleForwardPass(MatrixT<T> matrix) const {
  const Eigen::Index cols = matrix.cols();
  for (int col_idx = 0; col_idx < cols; ++col_idx) {
    singleForwardPass1D(matrix.col(col_idx));
  }

  const Eigen::Index rows = matrix.rows();
  for (int row_idx = 0; row_idx < rows; ++row_idx) {
    singleForwardPass1D(matrix.row(row_idx));
  }

  return matrix;
}

template <typename T>
MatrixT<T> LiftedCDF53<T>::singleBackwardPass(MatrixT<T> matrix) const {
  const Eigen::Index cols = matrix.cols();
  for (int col_idx = 0; col_idx < cols; ++col_idx) {
    singleBackwardPass1D(matrix.col(col_idx));
  }

  const Eigen::Index rows = matrix.rows();
  for (int row_idx = 0; row_idx < rows; ++row_idx) {
    singleBackwardPass1D(matrix.row(row_idx));
  }

  return matrix;
}

template <typename T>
template <typename XprT>
void LiftedCDF53<T>::singleForwardPass1D(XprT x) const {
  const int n = static_cast<int>(x.size());

  // Prediction pass 1
  for (int i = 1; i < n - 2; i += 2) {
    x[i] += kPredict1 * (x[i - 1] + x[i + 1]);
  }
  x[n - 1] += 2 * kPredict1 * x[n - 2];

  // Update pass 1
  for (int i = 2; i < n; i += 2) {
    x[i] += kUpdate1 * (x[i - 1] + x[i + 1]);
  }
  x[0] += 2 * kUpdate1 * x[1];

  // Correct the scale
  for (int i = 0; i < n; ++i) {
    if (i % 2) {
      x[i] *= kScale;
    } else {
      x[i] *= kScaleInv;
    }
  }

  // Pack the averages into the first half and the details into the second half
  // of the vector
  std::vector<T> tempbank(n);
  for (int i = 0; i < n; ++i) {
    if (i % 2 == 0) {
      tempbank[i / 2] = x[i];
    } else {
      tempbank[n / 2 + i / 2] = x[i];
    }
  }
  for (int i = 0; i < n; ++i) {
    x[i] = tempbank[i];
  }
}

template <typename T>
template <typename XprT>
void LiftedCDF53<T>::singleBackwardPass1D(XprT x) const {
  const int n = static_cast<int>(x.size());

  // Unpack
  std::vector<T> tempbank(n);
  for (int i = 0; i < n / 2; ++i) {
    tempbank[i * 2] = x[i];
    tempbank[i * 2 + 1] = x[i + n / 2];
  }
  for (int i = 0; i < n; ++i) {
    x[i] = tempbank[i];
  }

  // Undo the scaling pass
  for (int i = 0; i < n; ++i) {
    if (i % 2) {
      x[i] *= kScaleInv;
    } else {
      x[i] *= kScale;
    }
  }

  // Undo update pass 1
  for (int i = 2; i < n; i += 2) {
    x[i] += kUpdate1Inv * (x[i - 1] + x[i + 1]);
  }
  x[0] += 2 * kUpdate1Inv * x[1];

  // Undo prediction pass 1
  for (int i = 1; i < n - 2; i += 2) {
    x[i] += kPredict1Inv * (x[i - 1] + x[i + 1]);
  }
  x[n - 1] += 2 * kPredict1Inv * x[n - 2];
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_TRANSFORM_DENSE_IMPL_LIFTED_CDF_5_3_INL_H_

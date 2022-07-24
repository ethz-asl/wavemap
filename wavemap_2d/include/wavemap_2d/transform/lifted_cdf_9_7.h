#ifndef WAVEMAP_2D_TRANSFORM_LIFTED_CDF_9_7_H_
#define WAVEMAP_2D_TRANSFORM_LIFTED_CDF_9_7_H_

#include <vector>

#include "wavemap_2d/transform/discrete_wavelet_transform.h"

namespace wavemap_2d {
template <typename T>
class LiftedCDF97 : public DiscreteWaveletTransform<T> {
 protected:
  // NOTE: These values are from the JPEG200 Standard (ISO/IEC 15444-1).
  static constexpr T kScaleInv = 1.230174104914001;
  static constexpr T kScale = 1 / kScaleInv;
  static constexpr T kPredict1 = -1.586134342059924;
  static constexpr T kPredict1Inv = -kPredict1;
  static constexpr T kUpdate1 = -0.052980118572961;
  static constexpr T kUpdate1Inv = -kUpdate1;
  static constexpr T kPredict2 = 0.882911075530934;
  static constexpr T kPredict2Inv = -kPredict2;
  static constexpr T kUpdate2 = 0.443506852043971;
  static constexpr T kUpdate2Inv = -kUpdate2;

  MatrixT<T> singleForwardPass(MatrixT<T> matrix) const override;
  MatrixT<T> singleBackwardPass(MatrixT<T> matrix) const override;

  template <typename XprT>
  void singleForwardPass1D(XprT x) const;
  template <typename XprType>
  void singleBackwardPass1D(XprType x) const;
};
}  // namespace wavemap_2d

#include "wavemap_2d/transform/lifted_cdf_9_7_inl.h"

#endif  // WAVEMAP_2D_TRANSFORM_LIFTED_CDF_9_7_H_

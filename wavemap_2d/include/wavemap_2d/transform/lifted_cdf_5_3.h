#ifndef WAVEMAP_2D_TRANSFORM_LIFTED_CDF_5_3_H_
#define WAVEMAP_2D_TRANSFORM_LIFTED_CDF_5_3_H_

#include <vector>

#include "wavemap_2d/transform/discrete_wavelet_transform.h"

namespace wavemap_2d {
template <typename T>
class LiftedCDF53 : public DiscreteWaveletTransform<T> {
 protected:
  static constexpr T kScaleInv = M_SQRT2;
  static constexpr T kScale = 1.f / kScaleInv;
  static constexpr T kPredict1 = -1.f / 2.f;
  static constexpr T kPredict1Inv = -kPredict1;
  static constexpr T kUpdate1 = 1.f / 4.f;
  static constexpr T kUpdate1Inv = -kUpdate1;

  MatrixT<T> singleForwardPass(MatrixT<T> matrix) const override;
  MatrixT<T> singleBackwardPass(MatrixT<T> matrix) const override;

  template <typename XprT>
  void singleForwardPass1D(XprT x) const;
  template <typename XprT>
  void singleBackwardPass1D(XprT x) const;
};
}  // namespace wavemap_2d

#include "wavemap_2d/transform/lifted_cdf_5_3_inl.h"

#endif  // WAVEMAP_2D_TRANSFORM_LIFTED_CDF_5_3_H_

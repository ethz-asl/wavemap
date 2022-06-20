#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_HAAR_WAVELET_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_HAAR_WAVELET_H_

#include <string>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/volumetric/cell_types/wavelet_coefficients.h"
#include "wavemap_2d/indexing/ndtree_index.h"

namespace wavemap_2d {
template <typename ValueT = FloatingPoint>
class HaarWavelet {
 public:
  using Coefficients = WaveletCoefficients<ValueT>;
  using ParentCoefficients = typename Coefficients::Parent;
  using ChildScaleCoefficients = typename Coefficients::ChildScales;

  // Transform 4 scale space coefficients into 1 coarse scale and 3 detail
  // coefficients using the DWT with the orthonormal Haar basis:
  // fn_scale = 1 / Sqrt2 * [1, 1] and fn_details = 1 / Sqrt2 * [-1, 1]
  static ParentCoefficients forward(
      const ChildScaleCoefficients& child_scale_coefficients) {
    // NOTE: We use the lifted implementation because it's as fast as the naive
    //       version but uses less operations, leaving more room for pipelining
    //       of surrounding code, and possibly hyper-threads.
    return forwardLifted(child_scale_coefficients);
  }

  // Transform 1 coarse scale and 3 detail coefficients into 4 scale space
  // coefficients using Inverse DWT with the orthonormal Haar basis:
  // fn_scale = 1 / Sqrt2 * [1, 1] and fn_details = 1 / Sqrt2 * [-1, 1]
  static ChildScaleCoefficients backward(
      const ParentCoefficients& parent_coefficients) {
    // NOTE: We use the naive implementation because it's significantly faster.
    //       Even though the lifted implementation uses less operations, most of
    //       its operations depend on each other's results. This generates long
    //       dependency chains and reduces the potential for pipelining.
    return backwardNaive(parent_coefficients);
  }

  static ParentCoefficients forwardSingleChild(
      typename Coefficients::Scale child_scale_coefficient,
      QuadtreeIndex::RelativeChild relative_child_idx);

  static typename Coefficients::Scale backwardSingleChild(
      const ParentCoefficients& parent_coefficients,
      QuadtreeIndex::RelativeChild relative_child_idx);

 private:
  static constexpr auto kOneHalf = static_cast<ValueT>(0.5);
  static constexpr auto kOneQuarter = static_cast<ValueT>(0.25);

  static ParentCoefficients forwardNaive(
      const ChildScaleCoefficients& child_scale_coefficients);

  static ChildScaleCoefficients backwardNaive(
      const ParentCoefficients& parent_coefficients);

  static ParentCoefficients forwardLifted(
      const ChildScaleCoefficients& child_scale_coefficients);

  static ChildScaleCoefficients backwardLifted(
      const ParentCoefficients& parent_coefficients);

  friend class HaarCellTest_LiftedImplementationEquivalence_Test;
};
}  // namespace wavemap_2d

#include "wavemap_2d/data_structure/volumetric/cell_types/impl/haar_wavelet_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_HAAR_WAVELET_H_

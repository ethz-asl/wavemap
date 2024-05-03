#ifndef WAVEMAP_CORE_MAP_CELL_TYPES_HAAR_TRANSFORM_H_
#define WAVEMAP_CORE_MAP_CELL_TYPES_HAAR_TRANSFORM_H_

#include <string>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/map/cell_types/haar_coefficients.h"

namespace wavemap {
// Transform 2^d scale space coefficients into 1 coarse scale and 2^d - 1
// detail coefficients using the DWT with the orthogonal Haar basis:
// fn_scale = [1, 1] and fn_details = 0.5 * [-1, 1] along each dimension
template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::Parent ForwardLifted(
    const typename HaarCoefficients<ValueT, dim>::CoefficientsArray&
        child_scales);

template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::Parent ForwardParallel(
    const typename HaarCoefficients<ValueT, dim>::CoefficientsArray&
        child_scales);

template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::Parent ForwardSingleChild(
    const typename HaarCoefficients<ValueT, dim>::Scale& child_scale,
    NdtreeIndexRelativeChild child_idx);

// Transform 1 coarse scale and 2^d - 1 detail coefficients into 2^d scale
// space coefficients using Inverse DWT with the orthogonal Haar basis:
// fn_scale = [1, 1] and fn_details = 0.5 * [-1, 1] along each dimension
template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::CoefficientsArray BackwardLifted(
    const typename HaarCoefficients<ValueT, dim>::Parent& parent);

template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::CoefficientsArray BackwardParallel(
    const typename HaarCoefficients<ValueT, dim>::Parent& parent);

template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::Scale BackwardSingleChild(
    const typename HaarCoefficients<ValueT, dim>::Parent& parent,
    NdtreeIndexRelativeChild child_idx);

// NOTE: The parallel and lifted transform implementations trade off data
//       parallelism (i.e. short instruction dependency chains) and efficiency
//       (i.e. less required operations in total). Benchmarks show the lifting
//       implementation outperforming its parallel counterpart from 3D onwards
//       on Intel Skylake. This might differ on other architectures.
template <typename ValueT, int dim>
struct HaarTransform {
  static constexpr int kDim = dim;

  static typename HaarCoefficients<ValueT, kDim>::Parent forward(
      const typename HaarCoefficients<ValueT, kDim>::CoefficientsArray&
          child_scale_coefficients) {
    return ForwardLifted<ValueT, kDim>(child_scale_coefficients);
  }

  static typename HaarCoefficients<ValueT, kDim>::CoefficientsArray backward(
      const typename HaarCoefficients<ValueT, kDim>::Parent&
          parent_coefficients) {
    return BackwardLifted<ValueT, kDim>(parent_coefficients);
  }

  static typename HaarCoefficients<ValueT, kDim>::Parent forwardSingleChild(
      typename HaarCoefficients<ValueT, kDim>::Scale child_scale,
      NdtreeIndexRelativeChild child_idx) {
    return ForwardSingleChild<ValueT, kDim>(child_scale, child_idx);
  }

  static typename HaarCoefficients<ValueT, kDim>::Scale backwardSingleChild(
      typename HaarCoefficients<ValueT, kDim>::Parent parent,
      NdtreeIndexRelativeChild child_idx) {
    return BackwardSingleChild<ValueT, kDim>(parent, child_idx);
  }
};

template <typename ValueT>
struct HaarTransform<ValueT, 1> {
  static constexpr int kDim = 1;

  static typename HaarCoefficients<ValueT, kDim>::Parent forward(
      const typename HaarCoefficients<ValueT, kDim>::CoefficientsArray&
          child_scale_coefficients) {
    return ForwardParallel<ValueT, kDim>(child_scale_coefficients);
  }

  static typename HaarCoefficients<ValueT, kDim>::CoefficientsArray backward(
      const typename HaarCoefficients<ValueT, kDim>::Parent&
          parent_coefficients) {
    return BackwardParallel<ValueT, kDim>(parent_coefficients);
  }

  static typename HaarCoefficients<ValueT, kDim>::Parent forwardSingleChild(
      typename HaarCoefficients<ValueT, kDim>::Scale child_scale,
      NdtreeIndexRelativeChild child_idx) {
    return ForwardSingleChild<ValueT, kDim>(child_scale, child_idx);
  }

  static typename HaarCoefficients<ValueT, kDim>::Scale backwardSingleChild(
      typename HaarCoefficients<ValueT, kDim>::Parent parent,
      NdtreeIndexRelativeChild child_idx) {
    return BackwardSingleChild<ValueT, kDim>(parent, child_idx);
  }
};

template <typename ValueT>
struct HaarTransform<ValueT, 2> {
  static constexpr int kDim = 2;

  static typename HaarCoefficients<ValueT, kDim>::Parent forward(
      const typename HaarCoefficients<ValueT, kDim>::CoefficientsArray&
          child_scale_coefficients) {
    return ForwardParallel<ValueT, kDim>(child_scale_coefficients);
  }

  static typename HaarCoefficients<ValueT, kDim>::CoefficientsArray backward(
      const typename HaarCoefficients<ValueT, kDim>::Parent&
          parent_coefficients) {
    return BackwardParallel<ValueT, kDim>(parent_coefficients);
  }

  static typename HaarCoefficients<ValueT, kDim>::Parent forwardSingleChild(
      typename HaarCoefficients<ValueT, kDim>::Scale child_scale,
      NdtreeIndexRelativeChild child_idx) {
    return ForwardSingleChild<ValueT, kDim>(child_scale, child_idx);
  }

  static typename HaarCoefficients<ValueT, kDim>::Scale backwardSingleChild(
      typename HaarCoefficients<ValueT, kDim>::Parent parent,
      NdtreeIndexRelativeChild child_idx) {
    return BackwardSingleChild<ValueT, kDim>(parent, child_idx);
  }
};
}  // namespace wavemap

#include "wavemap/core/map/cell_types/impl/haar_transform_inl.h"

#endif  // WAVEMAP_CORE_MAP_CELL_TYPES_HAAR_TRANSFORM_H_

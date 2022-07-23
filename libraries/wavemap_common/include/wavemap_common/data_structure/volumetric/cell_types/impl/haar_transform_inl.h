#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_IMPL_HAAR_TRANSFORM_INL_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_IMPL_HAAR_TRANSFORM_INL_H_

namespace wavemap {
template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::Parent ForwardLifted(
    const typename HaarCoefficients<ValueT, dim>::CoefficientsArray&
        child_scales) {
  typename HaarCoefficients<ValueT, dim>::CoefficientsArray
      parent_coefficients = child_scales;

  // Apply the forward transform one beam (row/col) at a time using lifting
  constexpr NdtreeIndexElement kNumBeams = int_math::exp2(dim - 1);
  for (NdtreeIndexElement dim_idx = 0; dim_idx < dim; ++dim_idx) {
    for (NdtreeIndexElement beam_idx = 0; beam_idx < kNumBeams; ++beam_idx) {
      const NdtreeIndexElement scale_idx =
          bit_manip::rotate_left<NdtreeIndexElement, dim>(beam_idx << 1,
                                                          dim_idx);
      const NdtreeIndexElement detail_idx =
          bit_manip::rotate_left<NdtreeIndexElement, dim>(beam_idx << 1 | 1,
                                                          dim_idx);
      parent_coefficients[detail_idx] -= parent_coefficients[scale_idx];
      parent_coefficients[scale_idx] +=
          static_cast<ValueT>(0.5) * parent_coefficients[detail_idx];
    }
  }

  return parent_coefficients;
}

template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::Parent ForwardParallel(
    const typename HaarCoefficients<ValueT, dim>::CoefficientsArray&
        child_scales) {
  typename HaarCoefficients<ValueT, dim>::CoefficientsArray
      parent_coefficients{};

  // TODO(victorr): Check assembly to see if GCC optimizes out the constants and
  //                manages to vectorize the loops nicely
  constexpr auto kNumCoefficients =
      HaarCoefficients<ValueT, dim>::kNumCoefficients;
  for (NdtreeIndexElement parent_idx = 0; parent_idx < kNumCoefficients;
       ++parent_idx) {
    for (NdtreeIndexElement child_idx = 0; child_idx < kNumCoefficients;
         ++child_idx) {
      parent_coefficients[parent_idx] +=
          bit_manip::parity(parent_idx & ~child_idx) ? -child_scales[child_idx]
                                                     : +child_scales[child_idx];
    }
    parent_coefficients[parent_idx] /= static_cast<ValueT>(
        int_math::exp2(dim - bit_manip::popcount(parent_idx)));
  }

  return parent_coefficients;
}

template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::Parent ForwardSingleChild(
    const typename HaarCoefficients<ValueT, dim>::Scale& child_scale,
    NdtreeIndexRelativeChild child_idx) {
  typename HaarCoefficients<ValueT, dim>::CoefficientsArray parent_coefficients;

  constexpr auto kNumCoefficients =
      HaarCoefficients<ValueT, dim>::kNumCoefficients;
  for (NdtreeIndexElement parent_idx = 0; parent_idx < kNumCoefficients;
       ++parent_idx) {
    parent_coefficients[parent_idx] = bit_manip::parity(parent_idx & ~child_idx)
                                          ? -child_scale
                                          : +child_scale;
    parent_coefficients[parent_idx] /= static_cast<ValueT>(
        int_math::exp2(dim - bit_manip::popcount(parent_idx)));
  }

  return parent_coefficients;
}

template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::CoefficientsArray BackwardLifted(
    const typename HaarCoefficients<ValueT, dim>::Parent& parent) {
  typename HaarCoefficients<ValueT, dim>::CoefficientsArray child_scales =
      parent;

  // Apply the inverse transform one beam (row/col) at a time using lifting
  constexpr NdtreeIndexElement kNumBeams = int_math::exp2(dim - 1);
  for (NdtreeIndexElement dim_idx = 0; dim_idx < dim; ++dim_idx) {
    for (NdtreeIndexElement beam_idx = 0; beam_idx < kNumBeams; ++beam_idx) {
      const NdtreeIndexElement scale_idx =
          bit_manip::rotate_left<NdtreeIndexElement, dim>(beam_idx << 1,
                                                          dim_idx);
      const NdtreeIndexElement detail_idx =
          bit_manip::rotate_left<NdtreeIndexElement, dim>(beam_idx << 1 | 1,
                                                          dim_idx);
      child_scales[scale_idx] -=
          static_cast<ValueT>(0.5) * child_scales[detail_idx];
      child_scales[detail_idx] += child_scales[scale_idx];
    }
  }

  return child_scales;
}

template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::CoefficientsArray BackwardParallel(
    const typename HaarCoefficients<ValueT, dim>::Parent& parent) {
  typename HaarCoefficients<ValueT, dim>::CoefficientsArray child_scales{};

  constexpr auto kNumCoefficients =
      HaarCoefficients<ValueT, dim>::kNumCoefficients;
  for (NdtreeIndexElement child_idx = 0; child_idx < kNumCoefficients;
       ++child_idx) {
    child_scales[child_idx] += parent.scale;
    for (NdtreeIndexElement parent_idx = 1; parent_idx < kNumCoefficients;
         ++parent_idx) {
      const ValueT contribution =
          parent.details[parent_idx - 1] /
          static_cast<ValueT>(int_math::exp2(bit_manip::popcount(parent_idx)));
      child_scales[child_idx] += bit_manip::parity(~child_idx & parent_idx)
                                     ? -contribution
                                     : contribution;
    }
  }

  return child_scales;
}

template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::Scale BackwardSingleChild(
    const typename HaarCoefficients<ValueT, dim>::Parent& parent,
    NdtreeIndexRelativeChild child_idx) {
  ValueT scale = parent.scale;

  for (NdtreeIndexElement parent_idx = 1;
       parent_idx < HaarCoefficients<ValueT, dim>::kNumCoefficients;
       ++parent_idx) {
    const ValueT contribution =
        parent.details[parent_idx - 1] /
        static_cast<ValueT>(int_math::exp2(bit_manip::popcount(parent_idx)));
    scale += bit_manip::parity(~child_idx & parent_idx) ? -contribution
                                                        : contribution;
  }

  return scale;
}
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_IMPL_HAAR_TRANSFORM_INL_H_

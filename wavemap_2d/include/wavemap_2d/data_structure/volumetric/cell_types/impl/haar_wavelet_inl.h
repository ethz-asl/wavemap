#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_IMPL_HAAR_WAVELET_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_IMPL_HAAR_WAVELET_INL_H_

namespace wavemap_2d {
template <typename ValueT>
typename HaarWavelet<ValueT>::ParentCoefficients
HaarWavelet<ValueT>::forwardNaive(
    const HaarWavelet::ChildScaleCoefficients& child_scale_coefficients) {
  ParentCoefficients parent_coefficients;
  parent_coefficients.scale =
      child_scale_coefficients[0] + child_scale_coefficients[2] +
      child_scale_coefficients[1] + child_scale_coefficients[3];
  parent_coefficients.details.xx =
      -child_scale_coefficients[0] - child_scale_coefficients[2] +
      child_scale_coefficients[1] + child_scale_coefficients[3];
  parent_coefficients.details.yy =
      -child_scale_coefficients[0] + child_scale_coefficients[2] -
      child_scale_coefficients[1] + child_scale_coefficients[3];
  parent_coefficients.details.xy =
      child_scale_coefficients[0] - child_scale_coefficients[2] -
      child_scale_coefficients[1] + child_scale_coefficients[3];
  return static_cast<ValueT>(0.5) * parent_coefficients;
}

template <typename ValueT>
typename HaarWavelet<ValueT>::ChildScaleCoefficients
HaarWavelet<ValueT>::backwardNaive(
    const HaarWavelet::ParentCoefficients& parent_coefficients) {
  ChildScaleCoefficients child_scale_coeffs;
  child_scale_coeffs[0] =
      parent_coefficients.scale - parent_coefficients.details.xx -
      parent_coefficients.details.yy + parent_coefficients.details.xy;
  child_scale_coeffs[1] =
      parent_coefficients.scale + parent_coefficients.details.xx -
      parent_coefficients.details.yy - parent_coefficients.details.xy;
  child_scale_coeffs[2] =
      parent_coefficients.scale - parent_coefficients.details.xx +
      parent_coefficients.details.yy - parent_coefficients.details.xy;
  child_scale_coeffs[3] =
      parent_coefficients.scale + parent_coefficients.details.xx +
      parent_coefficients.details.yy + parent_coefficients.details.xy;
  return static_cast<ValueT>(0.5) * child_scale_coeffs;
}

template <typename ValueT>
typename HaarWavelet<ValueT>::ParentCoefficients
HaarWavelet<ValueT>::forwardLifted(
    const HaarWavelet::ChildScaleCoefficients& child_scale_coefficients) {
  typename Coefficients::Array coefficient_array = child_scale_coefficients;

  // Apply the wavelet transform one row/col at a time using lifting
  // NOTE: This lifts into basis s=[0.5, 0.5]; d=[-1, 1].
  for (QuadtreeIndex::Element dim = 0; dim < QuadtreeIndex::kDim; ++dim) {
    for (QuadtreeIndex::Element pass = 0; pass < QuadtreeIndex::kDim; ++pass) {
      const QuadtreeIndex::Element scale_idx = pass << dim;
      const QuadtreeIndex::Element detail_idx =
          (QuadtreeIndex::kDim >> dim) | (pass << dim);
      coefficient_array[detail_idx] =
          coefficient_array[detail_idx] - coefficient_array[scale_idx];
      coefficient_array[scale_idx] =
          coefficient_array[scale_idx] +
          static_cast<ValueT>(0.5) * coefficient_array[detail_idx];
    }
  }
  // Convert to the orthonormal Haar basis
  // NOTE: This step could be avoided by directly lifting with steps
  //       b = 1 / Sqrt2 * (b - a); a = 1 / Sqrt2 * a + b, but that requires
  //       more compute.
  coefficient_array[0b00] *= static_cast<ValueT>(2.0);
  coefficient_array[0b11] *= static_cast<ValueT>(0.5);

  return static_cast<ParentCoefficients>(coefficient_array);
}

template <typename ValueT>
typename HaarWavelet<ValueT>::ChildScaleCoefficients
HaarWavelet<ValueT>::backwardLifted(
    const HaarWavelet::ParentCoefficients& parent_coefficients) {
  ChildScaleCoefficients child_scale_coefficients{
      parent_coefficients.scale, parent_coefficients.details.xx,
      parent_coefficients.details.yy, parent_coefficients.details.xy};

  child_scale_coefficients[0b00] *= static_cast<ValueT>(0.5);
  child_scale_coefficients[0b11] *= static_cast<ValueT>(2.0);
  for (QuadtreeIndex::Element dim = 0; dim < QuadtreeIndex::kDim; ++dim) {
    for (QuadtreeIndex::Element pass = 0; pass < QuadtreeIndex::kDim; ++pass) {
      const QuadtreeIndex::Element scale_idx = pass << dim;
      const QuadtreeIndex::Element detail_idx =
          (QuadtreeIndex::kDim >> dim) | (pass << dim);
      child_scale_coefficients[scale_idx] =
          child_scale_coefficients[scale_idx] -
          static_cast<ValueT>(0.5) * child_scale_coefficients[detail_idx];
      child_scale_coefficients[detail_idx] =
          child_scale_coefficients[detail_idx] +
          child_scale_coefficients[scale_idx];
    }
  }

  return child_scale_coefficients;
}

template <typename ValueT>
typename HaarWavelet<ValueT>::ParentCoefficients
HaarWavelet<ValueT>::forwardSingleChild(
    typename HaarWavelet<ValueT>::Coefficients::Scale child_scale_coefficient,
    QuadtreeIndex::RelativeChild relative_child_idx) {
  switch (relative_child_idx) {
    case 0:
      return static_cast<ValueT>(0.5) *
             ParentCoefficients{
                 child_scale_coefficient,
                 {-child_scale_coefficient, -child_scale_coefficient,
                  child_scale_coefficient}};
    case 1:
      return static_cast<ValueT>(0.5) *
             ParentCoefficients{
                 child_scale_coefficient,
                 {child_scale_coefficient, -child_scale_coefficient,
                  -child_scale_coefficient}};
    case 2:
      return static_cast<ValueT>(0.5) *
             ParentCoefficients{
                 child_scale_coefficient,
                 {-child_scale_coefficient, child_scale_coefficient,
                  -child_scale_coefficient}};
    case 3:
      return static_cast<ValueT>(0.5) *
             ParentCoefficients{
                 child_scale_coefficient,
                 {child_scale_coefficient, child_scale_coefficient,
                  child_scale_coefficient}};
    default:
      LOG(FATAL) << "Requested Discrete Wavelet Transform for "
                    "out-of-bounds child_idx: "
                 << relative_child_idx;
      return ParentCoefficients{};
  }
}

template <typename ValueT>
typename HaarWavelet<ValueT>::Coefficients::Scale
HaarWavelet<ValueT>::backwardSingleChild(
    const HaarWavelet::ParentCoefficients& parent_coefficients,
    QuadtreeIndex::RelativeChild relative_child_idx) {
  switch (relative_child_idx) {
    case 0:
      return static_cast<ValueT>(0.5) *
             (parent_coefficients.scale - parent_coefficients.details.xx -
              parent_coefficients.details.yy + parent_coefficients.details.xy);
    case 1:
      return static_cast<ValueT>(0.5) *
             (parent_coefficients.scale + parent_coefficients.details.xx -
              parent_coefficients.details.yy - parent_coefficients.details.xy);
    case 2:
      return static_cast<ValueT>(0.5) *
             (parent_coefficients.scale - parent_coefficients.details.xx +
              parent_coefficients.details.yy - parent_coefficients.details.xy);
    case 3:
      return static_cast<ValueT>(0.5) *
             (parent_coefficients.scale + parent_coefficients.details.xx +
              parent_coefficients.details.yy + parent_coefficients.details.xy);
    default:
      LOG(FATAL) << "Requested Inverse Discrete Wavelet Transform for "
                    "out-of-bounds child_idx: "
                 << relative_child_idx;
      return {};
  }
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_IMPL_HAAR_WAVELET_INL_H_

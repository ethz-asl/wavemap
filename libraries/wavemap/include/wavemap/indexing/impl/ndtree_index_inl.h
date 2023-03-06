#ifndef WAVEMAP_INDEXING_IMPL_NDTREE_INDEX_INL_H_
#define WAVEMAP_INDEXING_IMPL_NDTREE_INDEX_INL_H_

#include <string>
#include <vector>

#include "wavemap/utils/bit_manipulation.h"

namespace wavemap {
namespace convert {
template <int dim>
MortonCode indexToMorton(const Index<dim>& index) {
  uint64_t morton = 0u;
  constexpr auto pattern = bit_manip::repeat_block<uint64_t>(dim, 0b1);
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    DCHECK_GE(index[dim_idx], 0);
    DCHECK_LE(index[dim_idx], kMortonCoordinateMax<dim>);
    morton |= bit_manip::expand<uint64_t>(index[dim_idx], pattern << dim_idx);
  }
  return morton;
}

template <int dim>
Index<dim> mortonToIndex(MortonCode morton) {
  Index<dim> index;
  constexpr auto pattern = bit_manip::repeat_block<uint64_t>(dim, 0b1);
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    index[dim_idx] = bit_manip::compress<uint64_t>(morton, pattern << dim_idx);
  }
  return index;
}
}  // namespace convert

template <int dim>
NdtreeIndex<dim> NdtreeIndex<dim>::computeParentIndex() const {
  return {height + 1, int_math::div_exp2_floor(position, 1)};
}

template <int dim>
NdtreeIndex<dim> NdtreeIndex<dim>::computeParentIndex(
    Element parent_height) const {
  DCHECK_GE(parent_height, height);
  const Element height_difference = parent_height - height;
  return {parent_height, int_math::div_exp2_floor(position, height_difference)};
}

template <int dim>
NdtreeIndex<dim> NdtreeIndex<dim>::computeChildIndex(
    RelativeChild relative_child_index) const {
  NdtreeIndex child_index = *this;

  // Compute index of first child
  child_index.position *= 2;
  child_index.height -= 1;

  // Add offset to current child
  for (int i = 0; i < dim; ++i) {
    child_index.position[i] += (relative_child_index >> i) & 0b1;
  }

  return child_index;
}

template <int dim>
typename NdtreeIndex<dim>::ChildArray NdtreeIndex<dim>::computeChildIndices()
    const {
  ChildArray child_indices;
  for (RelativeChild relative_child_idx = 0; relative_child_idx < kNumChildren;
       ++relative_child_idx) {
    child_indices[relative_child_idx] = computeChildIndex(relative_child_idx);
  }
  return child_indices;
}

template <int dim>
NdtreeIndexRelativeChild NdtreeIndex<dim>::computeRelativeChildIndex() const {
  RelativeChild child_index = 0;
  for (int i = 0; i < dim; ++i) {
    child_index += (position[i] & 0b1) << i;
  }
  return child_index;
}

template <int dim>
NdtreeIndexRelativeChild NdtreeIndex<dim>::computeRelativeChildIndex(
    MortonCode morton, NdtreeIndex::Element parent_height) {
  return (morton >> ((parent_height - 1) * dim)) & kRelativeChildIndexMask;
}

template <int dim>
MortonCode NdtreeIndex<dim>::computeMortonCode() const {
  return convert::indexToMorton(position) << (height * dim);
}

template <int dim>
std::string NdtreeIndex<dim>::toString() const {
  std::stringstream ss;
  ss << "[height=";
  ss << height << ", position=[";
  for (int i = 0; i < dim; ++i) {
    if (i) {
      ss << ", ";
    }
    ss << position[i];
  }
  ss << "]]";
  return ss.str();
}
}  // namespace wavemap

#endif  // WAVEMAP_INDEXING_IMPL_NDTREE_INDEX_INL_H_

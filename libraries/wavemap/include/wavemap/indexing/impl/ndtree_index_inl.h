#ifndef WAVEMAP_INDEXING_IMPL_NDTREE_INDEX_INL_H_
#define WAVEMAP_INDEXING_IMPL_NDTREE_INDEX_INL_H_

#include <string>
#include <vector>

#include "wavemap/utils/bit_manipulation.h"
#include "wavemap/utils/tree_math.h"

namespace wavemap {
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
  const Element child_height = parent_height - 1;
  static constexpr MortonCode kRelativeChildIndexMask = (1 << dim) - 1;
  return (morton >> (child_height * dim)) & kRelativeChildIndexMask;
}

template <int dim>
LinearIndex NdtreeIndex<dim>::computeLinearOffset(
    MortonCode morton, NdtreeIndex::Element parent_height,
    NdtreeIndex::Element child_height) {
  const Element height_difference = parent_height - child_height;
  const LinearIndex parent_to_first_child_offset =
      tree_math::perfect_tree::num_total_nodes_fast<dim>(height_difference);

  const MortonCode relative_child_index_mask =
      (static_cast<MortonCode>(1) << (height_difference * dim)) - 1;
  const LinearIndex first_child_to_child_offset =
      (morton >> (child_height * dim)) & relative_child_index_mask;

  return parent_to_first_child_offset + first_child_to_child_offset;
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

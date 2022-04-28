#ifndef WAVEMAP_2D_INDEXING_IMPL_NDTREE_INDEX_INL_H_
#define WAVEMAP_2D_INDEXING_IMPL_NDTREE_INDEX_INL_H_

#include <string>
#include <vector>

namespace wavemap_2d {
template <int dim>
NdtreeIndex<dim> NdtreeIndex<dim>::computeParentIndex() const {
  DCHECK_GT(depth, 0);

  NdtreeIndex parent_index = *this;
  for (int i = 0; i < dim; ++i) {
    parent_index.position[i] >>= 1;
  }
  parent_index.depth = depth - 1;

  return parent_index;
}

template <int dim>
NdtreeIndex<dim> NdtreeIndex<dim>::computeParentIndex(
    Element parent_depth) const {
  DCHECK_GE(parent_depth, 0);
  DCHECK_LT(parent_depth, depth);
  const Element depth_difference = depth - parent_depth;

  NdtreeIndex parent_index = *this;
  for (int i = 0; i < dim; ++i) {
    parent_index.position[i] >>= depth_difference;
  }
  parent_index.depth = parent_depth;

  return parent_index;
}

template <int dim>
std::vector<NdtreeIndex<dim>> NdtreeIndex<dim>::computeParentIndices() const {
  if (depth == 0) {
    return {};
  }

  std::vector<NdtreeIndex> parent_indices(depth);
  parent_indices[depth - 1] = computeParentIndex();
  for (Element depth_idx = depth - 2; 0 <= depth_idx; --depth_idx) {
    parent_indices[depth_idx] =
        parent_indices[depth_idx + 1].computeParentIndex();
  }

  return parent_indices;
}

template <int dim>
NdtreeIndex<dim> NdtreeIndex<dim>::computeChildIndex(
    RelativeChild relative_child_index) const {
  NdtreeIndex child_index = *this;

  // Compute index of first child
  child_index.position *= 2;
  child_index.depth += 1;

  // Add offset to current child
  for (int i = 0; i < dim; ++i) {
    child_index.position[i] += (relative_child_index >> i) & 0b1;
  }

  return child_index;
}

template <int dim>
std::vector<NdtreeIndex<dim>> NdtreeIndex<dim>::computeChildIndices() const {
  std::vector<NdtreeIndex> child_indices(kNumChildren);
  for (RelativeChild relative_child_idx = 0; relative_child_idx < kNumChildren;
       ++relative_child_idx) {
    child_indices[relative_child_idx] = computeChildIndex(relative_child_idx);
  }
  return child_indices;
}

template <int dim>
typename NdtreeIndex<dim>::RelativeChild
NdtreeIndex<dim>::computeRelativeChildIndex() const {
  RelativeChild child_index = 0;
  for (int i = 0; i < dim; ++i) {
    child_index += (position[i] & 0b1) << i;
  }
  return child_index;
}

template <int dim>
std::vector<typename NdtreeIndex<dim>::RelativeChild>
NdtreeIndex<dim>::computeRelativeChildIndices() const {
  std::vector<RelativeChild> child_indices(depth);
  NdtreeIndex node_index = *this;
  for (Element depth_idx = depth - 1; 0 <= depth_idx; --depth_idx) {
    child_indices[depth_idx] = node_index.computeRelativeChildIndex();
    node_index = node_index.computeParentIndex();
  }
  return child_indices;
}

template <int dim>
std::string NdtreeIndex<dim>::toString() const {
  std::stringstream ss;
  ss << "[depth=";
  ss << depth << ", position=[";
  for (int i = 0; i < dim; ++i) {
    if (i) {
      ss << ", ";
    }
    ss << position[i];
  }
  ss << "]]";
  return ss.str();
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INDEXING_IMPL_NDTREE_INDEX_INL_H_

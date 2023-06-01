#ifndef WAVEMAP_COMMON_INDEXING_IMPL_NDTREE_INDEX_INL_H_
#define WAVEMAP_COMMON_INDEXING_IMPL_NDTREE_INDEX_INL_H_

#include <string>
#include <vector>

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
template <typename NdtreeIndex<dim>::Element max_height>
std::vector<NdtreeIndex<dim>> NdtreeIndex<dim>::computeParentIndices() const {
  DCHECK_LE(height, max_height);
  const int height_difference = max_height - height;
  if (height_difference == 0) {
    return {};
  }

  std::vector<NdtreeIndex> parent_indices(height_difference);
  parent_indices[height_difference - 1] = computeParentIndex();
  for (Element ancestor_idx = height_difference - 2; 0 <= ancestor_idx;
       --ancestor_idx) {
    parent_indices[ancestor_idx] =
        parent_indices[ancestor_idx + 1].computeParentIndex();
  }
  return parent_indices;
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
typename NdtreeIndex<dim>::RelativeChild
NdtreeIndex<dim>::computeRelativeChildIndex() const {
  RelativeChild child_index = 0;
  for (int i = 0; i < dim; ++i) {
    child_index += (position[i] & 0b1) << i;
  }
  return child_index;
}

template <int dim>
template <typename NdtreeIndex<dim>::Element max_height>
std::vector<typename NdtreeIndex<dim>::RelativeChild>
NdtreeIndex<dim>::computeRelativeChildIndices() const {
  DCHECK_LE(height, max_height);

  const int depth = max_height - height;
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

#endif  // WAVEMAP_COMMON_INDEXING_IMPL_NDTREE_INDEX_INL_H_

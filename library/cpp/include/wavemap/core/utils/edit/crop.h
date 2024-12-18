#ifndef WAVEMAP_CORE_UTILS_EDIT_CROP_H_
#define WAVEMAP_CORE_UTILS_EDIT_CROP_H_

#include <memory>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap::edit {
namespace detail {
template <typename MapT, typename ShapeT>
void cropLeavesBatch(typename MapT::Block::OctreeType::NodeRefType node,
                     const OctreeIndex& node_index, FloatingPoint& node_value,
                     ShapeT&& shape, FloatingPoint min_cell_width);

template <typename MapT, typename ShapeT>
void cropNodeRecursive(typename MapT::Block::OctreeType::NodeRefType node,
                       const OctreeIndex& node_index, FloatingPoint& node_value,
                       ShapeT&& shape, FloatingPoint min_cell_width,
                       IndexElement termination_height = 0);
}  // namespace detail

template <typename MapT, typename ShapeT>
void crop(MapT& map, ShapeT shape, IndexElement termination_height = 0,
          const std::shared_ptr<ThreadPool>& thread_pool = nullptr);
}  // namespace wavemap::edit

#include "wavemap/core/utils/edit/impl/crop_inl.h"

#endif  // WAVEMAP_CORE_UTILS_EDIT_CROP_H_

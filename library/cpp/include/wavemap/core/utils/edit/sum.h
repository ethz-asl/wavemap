#ifndef WAVEMAP_CORE_UTILS_EDIT_SUM_H_
#define WAVEMAP_CORE_UTILS_EDIT_SUM_H_

#include <memory>
#include <unordered_set>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/index_hashes.h"
#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap::edit {
namespace detail {
template <typename MapT, typename SamplingFn>
void sumLeavesBatch(typename MapT::Block::OctreeType::NodeRefType node,
                    const OctreeIndex& node_index, FloatingPoint& node_value,
                    SamplingFn&& sampling_function,
                    FloatingPoint min_cell_width);

template <typename MapT, typename SamplingFn>
void sumNodeRecursive(typename MapT::Block::OctreeType::NodeRefType node,
                      const OctreeIndex& node_index, FloatingPoint& node_value,
                      SamplingFn&& sampling_function,
                      FloatingPoint min_cell_width,
                      IndexElement termination_height = 0);

template <typename MapT>
void sumNodeRecursive(
    typename MapT::Block::OctreeType::NodeRefType node_A,
    typename MapT::Block::OctreeType::NodeConstRefType node_B);
}  // namespace detail

template <typename MapT, typename SamplingFn>
void sum(MapT& map, SamplingFn sampling_function,
         const std::unordered_set<Index3D, IndexHash<3>>& block_indices,
         IndexElement termination_height = 0,
         const std::shared_ptr<ThreadPool>& thread_pool = nullptr);

template <typename MapT>
void sum(MapT& map_A, const MapT& map_B,
         const std::shared_ptr<ThreadPool>& thread_pool = nullptr);
}  // namespace wavemap::edit

#include "wavemap/core/utils/edit/impl/sum_inl.h"

#endif  // WAVEMAP_CORE_UTILS_EDIT_SUM_H_

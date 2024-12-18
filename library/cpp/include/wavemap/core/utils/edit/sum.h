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
// Recursively sum two maps together
template <typename MapT>
void sumNodeRecursive(
    typename MapT::Block::OctreeType::NodeRefType node_A,
    typename MapT::Block::OctreeType::NodeConstRefType node_B);

// Recursively add a sampled value
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

// Recursively add a scalar value to all cells within a given shape
template <typename MapT, typename ShapeT>
void sumLeavesBatch(typename MapT::Block::OctreeType::NodeRefType node,
                    const OctreeIndex& node_index, FloatingPoint& node_value,
                    ShapeT&& mask, FloatingPoint summand,
                    FloatingPoint min_cell_width);
template <typename MapT, typename ShapeT>
void sumNodeRecursive(typename MapT::Block::OctreeType::NodeRefType node,
                      const OctreeIndex& node_index, FloatingPoint& node_value,
                      ShapeT&& mask, FloatingPoint summand,
                      FloatingPoint min_cell_width,
                      IndexElement termination_height = 0);
}  // namespace detail

// Sum two maps together
template <typename MapT>
void sum(MapT& map_A, const MapT& map_B,
         const std::shared_ptr<ThreadPool>& thread_pool = nullptr);

// Add a sampled value to all cells within a given list of blocks
template <typename MapT, typename SamplingFn>
void sum(MapT& map, SamplingFn sampling_function,
         const std::unordered_set<Index3D, IndexHash<3>>& block_indices,
         IndexElement termination_height = 0,
         const std::shared_ptr<ThreadPool>& thread_pool = nullptr);

// Add a scalar value to all cells within a given shape
template <typename MapT, typename ShapeT>
void sum(MapT& map, ShapeT mask, FloatingPoint summand,
         const std::shared_ptr<ThreadPool>& thread_pool = nullptr);
}  // namespace wavemap::edit

#include "wavemap/core/utils/edit/impl/sum_inl.h"

#endif  // WAVEMAP_CORE_UTILS_EDIT_SUM_H_

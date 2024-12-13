#ifndef WAVEMAP_CORE_UTILS_EDIT_SUM_H_
#define WAVEMAP_CORE_UTILS_EDIT_SUM_H_

#include <memory>

#include "wavemap/core/common.h"
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
}  // namespace detail

template <typename MapT, typename SamplingFn>
void sum(MapT& map, SamplingFn sampling_function,
         IndexElement termination_height = 0,
         const std::shared_ptr<ThreadPool>& thread_pool = nullptr);
}  // namespace wavemap::edit

#include "wavemap/core/utils/edit/impl/sum_inl.h"

#endif  // WAVEMAP_CORE_UTILS_EDIT_SUM_H_

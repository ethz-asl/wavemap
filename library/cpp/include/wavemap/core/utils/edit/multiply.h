#ifndef WAVEMAP_CORE_UTILS_EDIT_MULTIPLY_H_
#define WAVEMAP_CORE_UTILS_EDIT_MULTIPLY_H_

#include <memory>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap::edit {
namespace detail {
template <typename MapT>
void multiplyNodeRecursive(typename MapT::Block::OctreeType::NodeRefType node,
                           FloatingPoint multiplier);
}  // namespace detail

template <typename MapT>
void multiply(MapT& map, FloatingPoint multiplier,
              const std::shared_ptr<ThreadPool>& thread_pool = nullptr);
}  // namespace wavemap::edit

#include "wavemap/core/utils/edit/impl/multiply_inl.h"

#endif  // WAVEMAP_CORE_UTILS_EDIT_MULTIPLY_H_

#ifndef WAVEMAP_CORE_UTILS_EDIT_TRANSFORM_H_
#define WAVEMAP_CORE_UTILS_EDIT_TRANSFORM_H_

#include <memory>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap::edit {
template <typename MapT>
std::unique_ptr<MapT> transform(
    const MapT& B_map, const Transformation3D& T_AB,
    const std::shared_ptr<ThreadPool>& thread_pool = nullptr);
}  // namespace wavemap::edit

#include "wavemap/core/utils/edit/impl/transform_inl.h"

#endif  // WAVEMAP_CORE_UTILS_EDIT_TRANSFORM_H_

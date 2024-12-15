#ifndef WAVEMAP_CORE_UTILS_EDIT_IMPL_TRANSFORM_INL_H_
#define WAVEMAP_CORE_UTILS_EDIT_IMPL_TRANSFORM_INL_H_

#include <algorithm>
#include <memory>
#include <unordered_set>
#include <utility>

#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/indexing/index_hashes.h"
#include "wavemap/core/utils/edit/sum.h"
#include "wavemap/core/utils/geometry/aabb.h"
#include "wavemap/core/utils/iterate/grid_iterator.h"
#include "wavemap/core/utils/query/map_interpolator.h"

namespace wavemap::edit {
template <typename MapT>
std::unique_ptr<MapT> transform(
    const MapT& map_B, const Transformation3D& T_AB,
    const std::shared_ptr<ThreadPool>& thread_pool) {
  const IndexElement tree_height = map_B.getTreeHeight();
  const FloatingPoint min_cell_width = map_B.getMinCellWidth();
  const FloatingPoint block_width =
      convert::heightToCellWidth(min_cell_width, tree_height);
  const FloatingPoint block_width_inv = 1.f / block_width;

  // Find all blocks that need to be updated
  std::unordered_set<Index3D, IndexHash<3>> block_indices_A;
  map_B.forEachBlock([&block_indices_A, &T_AB, tree_height, min_cell_width,
                      block_width_inv](const Index3D& block_index,
                                       const auto& /*block*/) {
    AABB<Point3D> A_aabb{};
    const auto B_block_aabb =
        convert::nodeIndexToAABB<3>({tree_height, block_index}, min_cell_width);
    for (int idx = 0; idx < B_block_aabb.kNumCorners; ++idx) {
      const Point3D& B_corner = B_block_aabb.corner_point(idx);
      const Point3D A_corner = T_AB * B_corner;
      A_aabb.insert(A_corner);
    }
    const Index3D A_block_index_min =
        convert::pointToFloorIndex(A_aabb.min, block_width_inv);
    const Index3D A_block_index_max =
        convert::pointToCeilIndex(A_aabb.max, block_width_inv);
    for (const auto& A_block_index :
         Grid(A_block_index_min, A_block_index_max)) {
      block_indices_A.emplace(A_block_index);
    }
  });

  // Populate map A by interpolating map B
  auto map_A = std::make_unique<MapT>(map_B.getConfig());
  const Transformation3D T_BA = T_AB.inverse();
  QueryAccelerator accelerator_B{map_B};
  auto interpolator_B = [&T_BA, accelerator_B](const Point3D& A_point) mutable {
    const auto B_point = T_BA * A_point;
    return interpolate::trilinear(accelerator_B, B_point);
  };
  sum(*map_A, interpolator_B, block_indices_A, 0, thread_pool);

  return map_A;
}
}  // namespace wavemap::edit

#endif  // WAVEMAP_CORE_UTILS_EDIT_IMPL_TRANSFORM_INL_H_

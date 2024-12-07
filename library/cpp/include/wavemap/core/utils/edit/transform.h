#ifndef WAVEMAP_CORE_UTILS_EDIT_TRANSFORM_H_
#define WAVEMAP_CORE_UTILS_EDIT_TRANSFORM_H_

#include <algorithm>
#include <memory>
#include <utility>

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/aabb.h"
#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"
#include "wavemap/core/utils/edit/sample.h"
#include "wavemap/core/utils/iterate/grid_iterator.h"
#include "wavemap/core/utils/query/map_interpolator.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap::edit {
template <typename MapT = HashedWaveletOctree>
MapT transform(const MapT& B_map, const Transformation3D& T_AB,
               const std::shared_ptr<ThreadPool>& thread_pool = nullptr) {
  const IndexElement tree_height = B_map.getTreeHeight();
  const FloatingPoint min_cell_width = B_map.getMinCellWidth();
  const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;

  // Allocate blocks in the result map
  MapT A_map{B_map.getConfig()};
  B_map.forEachBlock([&A_map, &T_AB, tree_height, min_cell_width,
                      min_cell_width_inv](const Index3D& block_index,
                                          const auto& /*block*/) {
    AABB<Point3D> A_aabb{};
    const auto B_block_aabb =
        convert::nodeIndexToAABB<3>({tree_height, block_index}, min_cell_width);
    for (int idx = 0; idx < B_block_aabb.kNumCorners; ++idx) {
      const Point3D& B_corner = B_block_aabb.corner_point(idx);
      const Point3D A_corner = T_AB * B_corner;
      A_aabb.includePoint(A_corner);
    }
    const Index3D A_block_index_min =
        convert::pointToFloorIndex(A_aabb.min, min_cell_width_inv);
    const Index3D A_block_index_max =
        convert::pointToCeilIndex(A_aabb.max, min_cell_width_inv);
    for (const auto& A_block_index :
         Grid<3>(A_block_index_min, A_block_index_max)) {
      A_map.getOrAllocateBlock(A_block_index);
    }
  });

  // Populate map A by interpolating map B
  sample(
      A_map,
      [&B_map, &T_AB](const Point3D& A_point) {
        const auto B_point = T_AB * A_point;
        return interpolate::trilinear(B_map, B_point);
      },
      std::move(thread_pool));

  return A_map;
}
}  // namespace wavemap::edit

#endif  // WAVEMAP_CORE_UTILS_EDIT_TRANSFORM_H_

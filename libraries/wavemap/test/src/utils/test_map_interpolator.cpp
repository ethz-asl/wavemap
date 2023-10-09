#include <gtest/gtest.h>

#include "wavemap/data_structure/volumetric/hashed_blocks.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/utils/iterate/grid_iterator.h"
#include "wavemap/utils/query/map_interpolator.h"

namespace wavemap {
TEST(InterpolationUtilsTest, Trilinear) {
  VolumetricDataStructureConfig config{1.f, -10.f, 10.f};
  HashedBlocks map(config);
  const FloatingPoint cell_width = map.getMinCellWidth();
  // Fill with known pattern
  for (int neighbor_idx = 0; neighbor_idx < 8; ++neighbor_idx) {
    const Index3D index{(neighbor_idx >> 2) & 1, (neighbor_idx >> 1) & 1,
                        neighbor_idx & 1};
    const FloatingPoint value = static_cast<FloatingPoint>(neighbor_idx) + 1.f;
    map.setCellValue(index, value);
  }
  // Test that cell centers match exactly
  for (int neighbor_idx = 0; neighbor_idx < 8; ++neighbor_idx) {
    const Index3D index{(neighbor_idx >> 2) & 1, (neighbor_idx >> 1) & 1,
                        neighbor_idx & 1};
    const Point3D cell_center =
        convert::indexToCenterPoint<3>(index, cell_width);
    const FloatingPoint value = static_cast<FloatingPoint>(neighbor_idx) + 1.f;
    EXPECT_NEAR(interpolate::trilinear(map, cell_center), value, kEpsilon);
  }
  // Test values outside set range
  for (const auto& index : Grid<3>(-Index3D::Ones(), Index3D::Constant(2))) {
    if ((index.array() < 0 || 1 < index.array()).any()) {
      const Point3D cell_center =
          convert::indexToCenterPoint<3>(index, cell_width);
      EXPECT_NEAR(interpolate::trilinear(map, cell_center), 0.f, kEpsilon);
    }
  }
  // Test interpolation
  for (int dim_idx = 0; dim_idx < 3; ++dim_idx) {
    // Clear and fill the map with 2 constant partitions
    map.clear();
    constexpr FloatingPoint kFirstPartitionValue = 1.f;
    constexpr FloatingPoint kSecondPartitionValue = 2.f;
    for (int neighbor_idx = 0; neighbor_idx < 8; ++neighbor_idx) {
      const Index3D index{(neighbor_idx >> 2) & 1, (neighbor_idx >> 1) & 1,
                          neighbor_idx & 1};
      const FloatingPoint value =
          index[dim_idx] == 0 ? kFirstPartitionValue : kSecondPartitionValue;
      map.setCellValue(index, value);
    }
    // Create a line segment from the first to the second partition
    const Point3D min_cell_center =
        convert::indexToCenterPoint<3>(Index3D::Zero(), cell_width);
    const Point3D max_cell_center =
        convert::indexToCenterPoint<3>(Index3D::Ones(), cell_width);
    const Point3D midpoint = 0.5f * (max_cell_center + min_cell_center);
    Point3D segment_start = midpoint;
    segment_start[dim_idx] = min_cell_center[dim_idx];
    Point3D segment_end = midpoint;
    segment_end[dim_idx] = max_cell_center[dim_idx];
    // Test values along the line segment
    constexpr int kNumTestPoints = 20;
    for (int point_idx = 0; point_idx <= kNumTestPoints; ++point_idx) {
      const FloatingPoint a = static_cast<FloatingPoint>(point_idx) /
                              static_cast<FloatingPoint>(kNumTestPoints);
      const Point3D test_point = (1.f - a) * segment_start + a * segment_end;
      const FloatingPoint expected_value =
          (1.f - a) * kFirstPartitionValue + a * kSecondPartitionValue;
      EXPECT_NEAR(interpolate::trilinear(map, test_point), expected_value,
                  kEpsilon);
    }
  }
}
}  // namespace wavemap

#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/data_structure/volumetric/hashed_blocks.h"
#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"
#include "wavemap/test/config_generator.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"
#include "wavemap/utils/sdf/sdf_generator.h"

namespace wavemap {
class SdfGenerationTest : public FixtureBase,
                          public GeometryGenerator,
                          public ConfigGenerator {};

TEST_F(SdfGenerationTest, QuasiEuclideanSDFGenerator) {
  // Params
  const Index3D min_index = Index3D::Constant(-10);
  const Index3D max_index = Index3D::Constant(100);
  constexpr FloatingPoint kMaxSdfDistance = 2.f;

  // Create a map and sample random obstacles
  const auto config =
      ConfigGenerator::getRandomConfig<HashedWaveletOctree::Config>();
  HashedWaveletOctree map{config};
  const auto obstacle_cells =
      GeometryGenerator::getRandomIndexVector<3>(10, 20, min_index, max_index);
  const FloatingPoint min_cell_width = map.getMinCellWidth();

  // Set default occupancy to free
  const IndexElement padding = std::ceil(kMaxSdfDistance / min_cell_width);
  const OctreeIndex min_block_index = convert::indexAndHeightToNodeIndex<3>(
      min_index.array() - padding, map.getTreeHeight());
  const OctreeIndex max_block_index = convert::indexAndHeightToNodeIndex<3>(
      max_index.array() + padding, map.getTreeHeight());
  for (const auto& block_index :
       Grid(min_block_index.position, max_block_index.position)) {
    map.getOrAllocateBlock(block_index).getRootScale() = config.min_log_odds;
  }

  // Set obstacles to occupied
  for (const Index3D& index : obstacle_cells) {
    map.setCellValue(index, config.max_log_odds);
  }

  // Generate the SDF
  QuasiEuclideanSDFGenerator sdf_generator{kMaxSdfDistance};
  const auto sdf = sdf_generator.generate(map);

  // Compare the SDF distances to the brute force min distance
  sdf.forEachLeaf([&map, &sdf_generator, &sdf, &obstacle_cells, min_cell_width](
                      const OctreeIndex& node_index, FloatingPoint sdf_value) {
    // In unobserved space, the SDF should be uninitialized
    const FloatingPoint occupancy_value = map.getCellValue(node_index);
    if (QuasiEuclideanSDFGenerator::isUnobserved(occupancy_value)) {
      // In unknown space the SDF should be uninitialized
      EXPECT_NEAR(sdf_value, sdf.getDefaultCellValue(), kEpsilon);
      return;
    }

    // In occupied space, the SDF should be negative and vice versa
    if (sdf_generator.isOccupied(occupancy_value)) {
      // In occupied space, the SDF should be negative
      EXPECT_LE(sdf_value, 0.f);
      return;
    }
    if (sdf_value < 0.f) {
      EXPECT_GT(occupancy_value, 0.f);
      return;
    }

    // We're in free space
    // Compute the distance to the closest obstacle using brute force
    FloatingPoint min_distance_brute_force = sdf.getDefaultCellValue();
    const Point3D node_center =
        convert::nodeIndexToCenterPoint(node_index, min_cell_width);
    for (const auto& obstacle_cell : obstacle_cells) {
      const auto obstacle_aabb = convert::nodeIndexToAABB(
          OctreeIndex{0, obstacle_cell}, min_cell_width);
      min_distance_brute_force = std::min(
          obstacle_aabb.minDistanceTo(node_center), min_distance_brute_force);
    }

    // Check that the SDF accurately approximates the min obstacle distance
    constexpr FloatingPoint kMaxMultiplicativeError =
        QuasiEuclideanSDFGenerator::kMaxMultiplicativeError;
    if (min_distance_brute_force <= sdf.getDefaultCellValue()) {
      EXPECT_NEAR(sdf_value, min_distance_brute_force,
                  min_distance_brute_force * kMaxMultiplicativeError);
    } else {
      EXPECT_NEAR(sdf_value, sdf.getDefaultCellValue(),
                  sdf.getDefaultCellValue() * kMaxMultiplicativeError);
    }
  });
}
}  // namespace wavemap

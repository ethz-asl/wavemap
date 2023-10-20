#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/map/hashed_blocks.h"
#include "wavemap/map/hashed_wavelet_octree.h"
#include "wavemap/test/config_generator.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"
#include "wavemap/utils/sdf/quasi_euclidean_sdf_generator.h"

namespace wavemap {
class SdfGenerationTest : public FixtureBase,
                          public GeometryGenerator,
                          public ConfigGenerator {};

TEST_F(SdfGenerationTest, QuasiEuclideanSDFGenerator) {
  // Params
  const Index3D min_index = Index3D::Constant(-10);
  const Index3D max_index = Index3D::Constant(100);
  constexpr FloatingPoint kMaxSdfDistance = 2.f;

  // Create the map and occupancy classification util
  const auto config =
      ConfigGenerator::getRandomConfig<HashedWaveletOctree::Config>();
  HashedWaveletOctree map{config};
  const OccupancyClassifier classifier{};

  // Generate random obstacles
  auto obstacle_cells =
      GeometryGenerator::getRandomIndexVector<3>(10, 20, min_index, max_index);
  // Add a cube
  const FloatingPoint min_cell_width = map.getMinCellWidth();
  const IndexElement padding = std::ceil(kMaxSdfDistance / min_cell_width);
  const Index3D cube_center{2, 6, 4};
  for (const Index3D& index :
       Grid<3>(cube_center.array() - padding, cube_center.array() + padding)) {
    obstacle_cells.emplace_back(index);
  }

  // Set default occupancy to free
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
  sdf.forEachLeaf([&map, &classifier, &sdf_generator, &sdf, &obstacle_cells,
                   min_cell_width, padding](const OctreeIndex& node_index,
                                            FloatingPoint sdf_value) {
    // In unobserved space, the SDF should be uninitialized
    const FloatingPoint occupancy_value = map.getCellValue(node_index);
    if (OccupancyClassifier::isUnobserved(occupancy_value)) {
      // In unknown space the SDF should be uninitialized
      EXPECT_NEAR(sdf_value, sdf.getDefaultCellValue(), kEpsilon);
      return;
    }

    const Point3D node_center =
        convert::nodeIndexToCenterPoint(node_index, min_cell_width);

    // Find the closest surface using brute force
    FloatingPoint sdf_brute_force = sdf.getDefaultCellValue();
    if (classifier.isFree(occupancy_value)) {
      // In free space, the SDF should always be positive
      EXPECT_GT(sdf_value, 0.f);

      // Find the distance to the closest obstacle
      for (const auto& obstacle_cell : obstacle_cells) {
        const auto obstacle_aabb = convert::nodeIndexToAABB(
            OctreeIndex{0, obstacle_cell}, min_cell_width);
        sdf_brute_force =
            std::min(obstacle_aabb.minDistanceTo(node_center), sdf_brute_force);
      }
    } else {
      // Find the distance to the closest free cell
      for (const Index3D& neighbor_index :
           Grid<3>(node_index.position.array() - padding,
                   node_index.position.array() + padding)) {
        const FloatingPoint neighbor_occupancy_value =
            map.getCellValue(neighbor_index);
        if (classifier.isFree(neighbor_occupancy_value) &&
            OccupancyClassifier::isObserved(neighbor_occupancy_value)) {
          const auto free_cell_aabb = convert::nodeIndexToAABB(
              OctreeIndex{0, neighbor_index}, min_cell_width);
          sdf_brute_force = std::min(free_cell_aabb.minDistanceTo(node_center),
                                     sdf_brute_force);
        }
      }
      // Adjust the sign to reflect we're inside the obstacle
      sdf_brute_force = -sdf_brute_force;

      // In occupied space, the SDF should be
      if (std::abs(sdf_brute_force) < sdf.getDefaultCellValue()) {
        // Negative
        EXPECT_LT(sdf_value, 0.f);
      } else {
        // Or uninitialized
        EXPECT_NEAR(sdf_value, sdf.getDefaultCellValue(), kEpsilon);
      }
    }

    // Check that the SDF accurately approximates the min obstacle distance
    constexpr FloatingPoint kMaxMultiplicativeError =
        QuasiEuclideanSDFGenerator::kMaxMultiplicativeError;
    if (std::abs(sdf_brute_force) < sdf.getDefaultCellValue()) {
      EXPECT_NEAR(sdf_value, sdf_brute_force,
                  std::abs(sdf_brute_force) * kMaxMultiplicativeError)
          << "At index " << print::eigen::oneLine(node_index.position);
    } else {
      EXPECT_NEAR(sdf_value, sdf.getDefaultCellValue(),
                  sdf.getDefaultCellValue() * kMaxMultiplicativeError)
          << "At index " << print::eigen::oneLine(node_index.position);
    }
  });
}
}  // namespace wavemap

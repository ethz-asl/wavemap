#include <gtest/gtest.h>

#include "wavemap/core/common.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"
#include "wavemap/core/utils/iterate/grid_iterator.h"
#include "wavemap/core/utils/query/classified_map.h"
#include "wavemap/test/config_generator.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
class ClassifiedMapTest : public FixtureBase,
                          public GeometryGenerator,
                          public ConfigGenerator {};

TEST_F(ClassifiedMapTest, ClassificationResults) {
  constexpr int kNumRepetitions = 3;
  for (int i = 0; i < kNumRepetitions; ++i) {
    // Create a random map
    const auto config =
        ConfigGenerator::getRandomConfig<HashedWaveletOctree::Config>();
    HashedWaveletOctree map{config};
    const std::vector<Index3D> random_indices =
        GeometryGenerator::getRandomIndexVector<3>(
            1000u, 2000u, Index3D::Constant(-5000), Index3D::Constant(5000));
    for (const Index3D& index : random_indices) {
      const FloatingPoint update = getRandomUpdate();
      map.addToCellValue(index, update);
    }
    map.prune();

    // Generate the classified map
    const OccupancyClassifier classifier;
    const ClassifiedMap classified_map{map, classifier};

    auto perturb_classified_map_cache = [this, &classified_map]() {
      const auto random_index = getRandomNdtreeIndex<OctreeIndex>(
          Index3D::Constant(-1000), Index3D::Constant(1000), 0,
          classified_map.getTreeHeight());
      classified_map.getNodeOrAncestor(random_index);
    };

    // Test all leaves
    map.forEachLeaf(
        [&map, &classifier, &classified_map, &perturb_classified_map_cache](
            const OctreeIndex& cell_index, FloatingPoint cell_log_odds) {
          perturb_classified_map_cache();
          const auto cell_occupancy_type = classifier.classify(cell_log_odds);
          EXPECT_TRUE(classified_map.isFully(cell_index, cell_occupancy_type))
              << "For cell_index: " << cell_index.toString();
          for (IndexElement height = cell_index.height;
               height < map.getTreeHeight(); ++height) {
            perturb_classified_map_cache();
            EXPECT_TRUE(classified_map.has(
                cell_index.computeParentIndex(height), cell_occupancy_type))
                << "For cell_index.computeParentIndex(height): "
                << cell_index.computeParentIndex(height).toString();
          }
        });

    // Test subregions
    for (int test_idx = 0; test_idx < 100; ++test_idx) {
      const IndexElement region_height = getRandomNdtreeIndexHeight(0, 4);
      const auto subregion =
          OctreeIndex{0, random_indices[test_idx]}.computeParentIndex(
              region_height);

      bool has_free = false;
      bool has_occupied = false;
      bool has_unobserved = false;
      for (const auto& index :
           Grid(convert::nodeIndexToMinCornerIndex(subregion),
                convert::nodeIndexToMaxCornerIndex(subregion))) {
        const FloatingPoint cell_log_odds = map.getCellValue(index);
        has_free |= classifier.is(cell_log_odds, Occupancy::kFree);
        has_occupied |= classifier.is(cell_log_odds, Occupancy::kOccupied);
        has_unobserved |= classifier.is(cell_log_odds, Occupancy::kUnobserved);
      }

      EXPECT_EQ(classified_map.getValue(subregion),
                Occupancy::toMask(has_free, has_occupied, has_unobserved));
    }
  }
}
}  // namespace wavemap

#include <gtest/gtest.h>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/map/hashed_blocks.h"
#include "wavemap/core/map/hashed_chunked_wavelet_octree.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/map/volumetric_octree.h"
#include "wavemap/core/map/wavelet_octree.h"
#include "wavemap/test/config_generator.h"
#include "wavemap/test/eigen_utils.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
template <typename MapType>
class MapTest : public FixtureBase,
                public GeometryGenerator,
                public ConfigGenerator {
 protected:
  static constexpr FloatingPoint kAcceptableReconstructionError = 5e-2f;
};

using MapTypes =
    ::testing::Types<HashedBlocks, VolumetricOctree, WaveletOctree,
                     HashedWaveletOctree, HashedChunkedWaveletOctree>;
TYPED_TEST_SUITE(MapTest, MapTypes, );

TYPED_TEST(MapTest, InitializationAndClearing) {
  const auto config =
      ConfigGenerator::getRandomConfig<typename TypeParam::Config>();
  std::unique_ptr<MapBase> map_base_ptr = std::make_unique<TypeParam>(config);

  // NOTE: Empty data structures are allowed to have size 0 or 1, such that the
  //       tree based data structures can keep their root node allocated even
  //       when empty.
  EXPECT_TRUE(map_base_ptr->empty());
  EXPECT_LE(map_base_ptr->size(), 1u);
  const size_t empty_map_memory_usage = map_base_ptr->getMemoryUsage();

  for (const Index3D& random_index :
       GeometryGenerator::getRandomIndexVector<3>()) {
    map_base_ptr->setCellValue(random_index, 1.f);
  }
  EXPECT_FALSE(map_base_ptr->empty());
  EXPECT_GT(map_base_ptr->size(), 1u);
  EXPECT_GE(map_base_ptr->getMemoryUsage(), empty_map_memory_usage);
  map_base_ptr->clear();
  EXPECT_TRUE(map_base_ptr->empty());
  EXPECT_LE(map_base_ptr->size(), 1u);
  EXPECT_LE(map_base_ptr->getMemoryUsage(), empty_map_memory_usage);
}

TYPED_TEST(MapTest, Pruning) {
  const auto config =
      ConfigGenerator::getRandomConfig<typename TypeParam::Config>();
  std::unique_ptr<MapBase> map_base_ptr = std::make_unique<TypeParam>(config);
  const size_t empty_map_memory_usage = map_base_ptr->getMemoryUsage();

  // Check that pruning removes all zero cells
  constexpr int kMinNumRandomIndices = 5e1;
  constexpr int kMaxNumRandomIndices = 5e2;
  const auto zero_cell_indexes = GeometryGenerator::getRandomIndexVector<3>(
      kMinNumRandomIndices, kMaxNumRandomIndices);
  for (const Index3D& index : zero_cell_indexes) {
    map_base_ptr->setCellValue(index, 0.f);
  }
  map_base_ptr->prune();
  EXPECT_TRUE(map_base_ptr->empty());
  EXPECT_LE(map_base_ptr->size(), 1u);
  EXPECT_LE(map_base_ptr->getMemoryUsage(), empty_map_memory_usage);

  // Check that pruning removes no non-zero cells
  const auto nonzero_cell_indexes = GeometryGenerator::getRandomIndexVector<3>(
      kMinNumRandomIndices, kMaxNumRandomIndices);
  for (const Index3D& index : zero_cell_indexes) {
    map_base_ptr->setCellValue(index, 0.f);
  }
  for (const Index3D& index : nonzero_cell_indexes) {
    map_base_ptr->setCellValue(index, 1.f);
  }
  const size_t size_before_pruning = map_base_ptr->size();
  const size_t memory_usage_before_pruning = map_base_ptr->getMemoryUsage();
  map_base_ptr->prune();
  EXPECT_FALSE(map_base_ptr->empty());
  EXPECT_LE(map_base_ptr->size(), size_before_pruning);
  EXPECT_LE(map_base_ptr->getMemoryUsage(), memory_usage_before_pruning);
  for (const Index3D& index : nonzero_cell_indexes) {
    EXPECT_EQ(map_base_ptr->getCellValue(index), 1.f);
  }
}

TYPED_TEST(MapTest, MinMaxIndexGetters) {
  constexpr int kNumRepetitions = 3;
  for (int i = 0; i < kNumRepetitions; ++i) {
    const auto config =
        ConfigGenerator::getRandomConfig<typename TypeParam::Config>();
    std::unique_ptr<MapBase> map_base_ptr = std::make_unique<TypeParam>(config);
    {
      const Index3D map_min_index = map_base_ptr->getMinIndex();
      const Index3D map_max_index = map_base_ptr->getMaxIndex();
      EXPECT_EIGEN_EQ(map_min_index, Index3D::Zero());
      EXPECT_EIGEN_EQ(map_max_index, Index3D::Zero());
    }
    const std::vector<Index3D> random_indices =
        GeometryGenerator::getRandomIndexVector<3>();
    Index3D reference_min_index =
        Index3D::Constant(std::numeric_limits<IndexElement>::max());
    Index3D reference_max_index =
        Index3D::Constant(std::numeric_limits<IndexElement>::lowest());
    for (const Index3D& index : random_indices) {
      map_base_ptr->addToCellValue(index, 1.f);
      reference_min_index = reference_min_index.cwiseMin(index);
      reference_max_index = reference_max_index.cwiseMax(index);
    }
    {
      const Index3D map_min_index = map_base_ptr->getMinIndex();
      const Index3D map_max_index = map_base_ptr->getMaxIndex();
      EXPECT_EIGEN_LE(map_min_index, reference_min_index);
      EXPECT_EIGEN_GE(map_max_index, reference_max_index);
    }
  }
}

TYPED_TEST(MapTest, InsertionAndLeafVisitor) {
  constexpr int kNumRepetitions = 3;
  for (int i = 0; i < kNumRepetitions; ++i) {
    // Create a random map
    const auto config =
        ConfigGenerator::getRandomConfig<typename TypeParam::Config>();
    std::unique_ptr<MapBase> map_base_ptr = std::make_unique<TypeParam>(config);
    const std::vector<Index3D> random_indices =
        GeometryGenerator::getRandomIndexVector<3>(
            1000u, 2000u, Index3D::Constant(-5000), Index3D::Constant(5000));
    std::unordered_map<Index3D, FloatingPoint, IndexHash<3>> reference_map;
    for (const Index3D& index : random_indices) {
      const FloatingPoint update = TestFixture::getRandomUpdate();
      map_base_ptr->addToCellValue(index, update);
      auto new_value = map_base_ptr->getCellValue(index);
      if (new_value < map_base_ptr->getMinLogOdds() ||
          map_base_ptr->getMaxLogOdds() < new_value) {
        map_base_ptr->prune();
      }
      reference_map[index] = std::clamp(reference_map[index] + update,
                                        map_base_ptr->getMinLogOdds(),
                                        map_base_ptr->getMaxLogOdds());
    }

    // Check that the indexed leaf value visitor visits all non-zero cells
    const size_t reference_map_size = reference_map.size();
    std::unordered_map<Index3D, FloatingPoint, IndexHash<3>> false_positive_map;
    map_base_ptr->forEachLeaf(
        [&reference_map, &false_positive_map](const OctreeIndex& node_index,
                                              FloatingPoint value) {
          const Index3D index = convert::nodeIndexToMinCornerIndex(node_index);
          // Check that the values are correct
          if (reference_map.count(index)) {
            EXPECT_NEAR(value, reference_map[index],
                        TestFixture::kAcceptableReconstructionError *
                            (1.f + reference_map[index]))
                << "At node index " << node_index.toString();
            // Remove cell from the reference map to indicate it was visited
            reference_map.erase(index);
          } else {
            EXPECT_NEAR(value, 0.f, TestFixture::kAcceptableReconstructionError)
                << "At node index " << node_index.toString();
            if (TestFixture::kAcceptableReconstructionError < std::abs(value)) {
              false_positive_map.emplace(index, value);
            }
          }
        });

    // If all non-zero values were visited, reference map should now be empty
    auto IndexToString = [](const Index3D& index) -> std::string {
      std::stringstream ss;
      ss << print::eigen::oneLine(index);
      return ss.str();
    };
    auto PrintMap = [IndexToString](const auto& map) -> std::string {
      if (map.empty()) {
        return "[empty]";
      } else {
        return std::accumulate(
            std::next(map.cbegin()), map.cend(),
            IndexToString(map.cbegin()->first) + ": " +
                std::to_string(map.cbegin()->second) + "\n",
            [&IndexToString](auto str, const auto& kv) -> std::string {
              return std::move(str) + IndexToString(kv.first) + ": " +
                     std::to_string(kv.second) + "\n";
            });
      }
    };
    EXPECT_TRUE(reference_map.empty())
        << "Leaf visitor missed " << reference_map.size() << " out of "
        << reference_map_size << " non-zero reference map values:\n"
        << PrintMap(reference_map) << "and falsely returned:\n"
        << PrintMap(false_positive_map);
  }
}

// TODO(victorr): For classes derived from VolumetricOctreeInterface, test
//                NodeIndex based setters and getters (incl. whether values of
//                all children are updated but nothing spills to the
//                neighbors)
}  // namespace wavemap

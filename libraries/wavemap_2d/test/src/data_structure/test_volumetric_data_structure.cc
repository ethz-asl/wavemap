#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/test/fixture_base.h>
#include <wavemap_common/utils/container_print_utils.h>

#include "wavemap_2d/data_structure/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/dense_grid.h"
#include "wavemap_2d/data_structure/differencing_quadtree.h"
#include "wavemap_2d/data_structure/hashed_blocks.h"
#include "wavemap_2d/data_structure/simple_quadtree.h"
#include "wavemap_2d/data_structure/volumetric_data_structure.h"
#include "wavemap_2d/data_structure/wavelet_tree.h"

namespace wavemap {
template <typename VolumetricDataStructureType>
class VolumetricDataStructureTest : public FixtureBase {
 protected:
  static constexpr FloatingPoint kAcceptableReconstructionError = 5e-2f;
};

using VolumetricDataStructureTypes = ::testing::Types<
    DenseGrid<UnboundedOccupancyCell>, DenseGrid<SaturatingOccupancyCell>,
    HashedBlocks<UnboundedOccupancyCell>, HashedBlocks<SaturatingOccupancyCell>,
    SimpleQuadtree<UnboundedOccupancyCell>,
    SimpleQuadtree<SaturatingOccupancyCell>,
    DifferencingQuadtree<UnboundedOccupancyCell>,
    DifferencingQuadtree<SaturatingOccupancyCell>,
    WaveletTree<UnboundedOccupancyCell>, WaveletTree<SaturatingOccupancyCell>>;
TYPED_TEST_SUITE(VolumetricDataStructureTest, VolumetricDataStructureTypes, );

TYPED_TEST(VolumetricDataStructureTest, InitializationAndClearing) {
  std::unique_ptr<VolumetricDataStructure> map_base_ptr =
      std::make_unique<TypeParam>(TestFixture::getRandomMinCellWidth());

  // NOTE: Empty data structures are allowed to have size 0 or 1, such that the
  //       tree based data structures can keep their root node allocated even
  //       when empty.
  EXPECT_TRUE(map_base_ptr->empty());
  EXPECT_LE(map_base_ptr->size(), 1u);
  const size_t empty_map_memory_usage = map_base_ptr->getMemoryUsage();

  for (const Index2D& random_index :
       TestFixture::template getRandomIndexVector<2>()) {
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

TYPED_TEST(VolumetricDataStructureTest, Pruning) {
  std::unique_ptr<VolumetricDataStructure> map_base_ptr =
      std::make_unique<TypeParam>(TestFixture::getRandomMinCellWidth());
  const size_t empty_map_memory_usage = map_base_ptr->getMemoryUsage();

  // Check that pruning removes all zero cells
  const auto zero_cell_indexes =
      TestFixture::template getRandomIndexVector<2>();
  for (const Index2D& index : zero_cell_indexes) {
    map_base_ptr->setCellValue(index, 0.f);
  }
  map_base_ptr->prune();
  EXPECT_TRUE(map_base_ptr->empty());
  EXPECT_LE(map_base_ptr->size(), 1u);
  EXPECT_LE(map_base_ptr->getMemoryUsage(), empty_map_memory_usage);

  // Check that pruning removes no non-zero cells
  const auto non_zero_cell_indexes =
      TestFixture::template getRandomIndexVector<2>();
  for (const Index2D& index : zero_cell_indexes) {
    map_base_ptr->setCellValue(index, 0.f);
  }
  for (const Index2D& index : non_zero_cell_indexes) {
    map_base_ptr->setCellValue(index, 1.f);
  }
  const size_t size_before_pruning = map_base_ptr->size();
  const size_t memory_usage_before_pruning = map_base_ptr->getMemoryUsage();
  map_base_ptr->prune();
  EXPECT_FALSE(map_base_ptr->empty());
  EXPECT_LE(map_base_ptr->size(), size_before_pruning);
  EXPECT_LE(map_base_ptr->getMemoryUsage(), memory_usage_before_pruning);
  for (const Index2D& index : non_zero_cell_indexes) {
    EXPECT_EQ(map_base_ptr->getCellValue(index), 1.f);
  }
}

TYPED_TEST(VolumetricDataStructureTest, MinMaxIndexGetters) {
  constexpr int kNumRepetitions = 3;
  for (int i = 0; i < kNumRepetitions; ++i) {
    std::unique_ptr<VolumetricDataStructure> map_base_ptr =
        std::make_unique<TypeParam>(TestFixture::getRandomMinCellWidth());
    {
      const Index2D map_min_index = map_base_ptr->getMinIndex();
      const Index2D map_max_index = map_base_ptr->getMaxIndex();
      for (QuadtreeIndex::Element dim = 0; dim < QuadtreeIndex::kDim; ++dim) {
        EXPECT_EQ(map_min_index[dim], 0) << " along dimension " << dim;
        EXPECT_EQ(map_max_index[dim], 0) << " along dimension " << dim;
      }
    }
    const std::vector<Index2D> random_indices =
        TestFixture::template getRandomIndexVector<2>();
    Index2D reference_min_index =
        Index2D::Constant(std::numeric_limits<IndexElement>::max());
    Index2D reference_max_index =
        Index2D::Constant(std::numeric_limits<IndexElement>::lowest());
    for (const Index2D& index : random_indices) {
      map_base_ptr->addToCellValue(index, 1.f);
      reference_min_index = reference_min_index.cwiseMin(index);
      reference_max_index = reference_max_index.cwiseMax(index);
    }
    {
      const Index2D map_min_index = map_base_ptr->getMinIndex();
      const Index2D map_max_index = map_base_ptr->getMaxIndex();
      for (QuadtreeIndex::Element dim = 0; dim < QuadtreeIndex::kDim; ++dim) {
        EXPECT_LE(map_min_index[dim], reference_min_index[dim])
            << " along dimension " << dim;
        EXPECT_GE(map_max_index[dim], reference_max_index[dim])
            << " along dimension " << dim;
      }
    }
  }
}

TYPED_TEST(VolumetricDataStructureTest, InsertionAndLeafVisitor) {
  constexpr int kNumRepetitions = 3;
  for (int i = 0; i < kNumRepetitions; ++i) {
    // Create a random map
    std::unique_ptr<VolumetricDataStructure> map_base_ptr =
        std::make_unique<TypeParam>(TestFixture::getRandomMinCellWidth());
    const std::vector<Index2D> random_indices =
        TestFixture::template getRandomIndexVector<2>(
            2u, 100u, Index2D::Constant(-100), Index2D::Constant(100));
    Index2D reference_min_index =
        Index2D::Constant(std::numeric_limits<IndexElement>::max());
    Index2D reference_max_index =
        Index2D::Constant(std::numeric_limits<IndexElement>::lowest());
    std::unordered_map<Index2D, FloatingPoint, VoxbloxIndexHash> reference_map;
    for (const Index2D& index : random_indices) {
      for (const FloatingPoint update : TestFixture::getRandomUpdateVector()) {
        map_base_ptr->addToCellValue(index, update);
        if (TypeParam::kRequiresPruningForThresholding) {
          map_base_ptr->prune();
        }
        reference_map[index] =
            TypeParam::CellType::add(reference_map[index], update);
        reference_min_index = reference_min_index.cwiseMin(index);
        reference_max_index = reference_max_index.cwiseMax(index);
      }
    }

    // Check that the map is complete and correct (incl. truncation)
    for (const Index2D& index : Grid(reference_min_index - Index2D::Ones(),
                                     reference_max_index + Index2D::Ones())) {
      if (reference_map.count(index)) {
        EXPECT_NEAR(map_base_ptr->getCellValue(index), reference_map[index],
                    TestFixture::kAcceptableReconstructionError *
                        (1.f + reference_map[index]));
      } else {
        EXPECT_NEAR(map_base_ptr->getCellValue(index), 0.f,
                    TestFixture::kAcceptableReconstructionError);
      }
    }

    // Check that the indexed leaf value visitor visits all non-zero cells
    const size_t reference_map_size = reference_map.size();
    map_base_ptr->forEachLeaf([&reference_map](const QuadtreeIndex& node_index,
                                               FloatingPoint value) {
      const Index2D index = convert::nodeIndexToMinCornerIndex(node_index);
      // Check that the values are correct
      if (reference_map.count(index)) {
        EXPECT_NEAR(value, reference_map[index],
                    TestFixture::kAcceptableReconstructionError *
                        (1.f + reference_map[index]));
        // Remove cell from the reference map to indicate it was visited
        reference_map.erase(index);
      } else {
        EXPECT_NEAR(value, 0.f, TestFixture::kAcceptableReconstructionError);
      }
    });
    // If all non-zero values were visited, reference map should now be empty
    // TODO(victorr): Clean up the printing
    auto IndexToString = [](const Index2D& index) -> std::string {
      std::stringstream ss;
      ss << EigenFormat::oneLine(index);
      return ss.str();
    };
    EXPECT_TRUE(reference_map.empty())
        << "Leaf visitor missed " << reference_map.size() << " out of "
        << reference_map_size << " non-zero reference map values:\n"
        << std::accumulate(
               std::next(reference_map.cbegin()), reference_map.cend(),
               IndexToString(reference_map.cbegin()->first) + ": " +
                   std::to_string(reference_map.cbegin()->second) + "\n",
               [&IndexToString](auto str, const auto& kv) -> std::string {
                 return std::move(str) + IndexToString(kv.first) + ": " +
                        std::to_string(kv.second) + "\n";
               });
  }
}

// TODO(victorr): For classes derived from VolumetricQuadtreeInterface, test
//                NodeIndex based setters and getters (incl. whether values of
//                all children are updated but nothing spills to the
//                neighbors)
}  // namespace wavemap

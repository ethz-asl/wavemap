#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/volumetric/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/volumetric/dense_grid.h"
#include "wavemap_2d/data_structure/volumetric/differencing_quadtree.h"
#include "wavemap_2d/data_structure/volumetric/hashed_blocks.h"
#include "wavemap_2d/data_structure/volumetric/simple_quadtree.h"
#include "wavemap_2d/data_structure/volumetric/volumetric_data_structure.h"
#include "wavemap_2d/data_structure/volumetric/wavelet_tree.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
template <typename VolumetricDataStructureType>
class VolumetricDataStructureTest : public FixtureBase {
 protected:
  static constexpr FloatingPoint kAcceptableReconstructionError = 1e-3f;
};

using VolumetricDataStructureTypes = ::testing::Types<
    DenseGrid<UnboundedOccupancyCell>, DenseGrid<SaturatingOccupancyCell>,
    HashedBlocks<UnboundedOccupancyCell>, HashedBlocks<SaturatingOccupancyCell>,
    SimpleQuadtree<UnboundedOccupancyCell>,
    SimpleQuadtree<SaturatingOccupancyCell>,
    DifferencingQuadtree<UnboundedOccupancyCell>,
    DifferencingQuadtree<SaturatingOccupancyCell>, WaveletTree>;
TYPED_TEST_SUITE(VolumetricDataStructureTest, VolumetricDataStructureTypes, );

// TODO(victorr): Also test clear, prune, getMin/MaxPossibleIndex,
//                getMin/MaxIndex and forEachLeaf methods
// TODO(victorr): For classes derived from VolumetricQuadtreeInterface, test
//                NodeIndex based setters and getters (incl. whether values of
//                all children are updated but nothing spills to the neighbors)

TYPED_TEST(VolumetricDataStructureTest, Insertion) {
  constexpr int kNumRepetitions = 10;
  for (int i = 0; i < kNumRepetitions; ++i) {
    std::unique_ptr<VolumetricDataStructure> map_base_ptr =
        std::make_unique<TypeParam>(TestFixture::getRandomMinCellWidth());
    const std::vector<Index> random_indices =
        TestFixture::getRandomIndexVector();
    for (const Index& random_index : random_indices) {
      FloatingPoint expected_value = 0.f;
      map_base_ptr->setCellValue(random_index, 0.f);
      for (const FloatingPoint random_update :
           TestFixture::getRandomUpdateVector()) {
        map_base_ptr->addToCellValue(random_index, random_update);
        map_base_ptr->prune();
        expected_value =
            TypeParam::CellType::add(expected_value, random_update);
      }
      EXPECT_NEAR(
          map_base_ptr->getCellValue(random_index), expected_value,
          TestFixture::kAcceptableReconstructionError * (1.f + expected_value));
    }
  }
}
}  // namespace wavemap_2d

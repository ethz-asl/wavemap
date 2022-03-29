#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/volumetric/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/volumetric/dense_grid.h"
#include "wavemap_2d/data_structure/volumetric/hashed_blocks.h"
#include "wavemap_2d/data_structure/volumetric/scalar_quadtree.h"
#include "wavemap_2d/data_structure/volumetric/volumetric_data_structure.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
template <typename VolumetricDataStructureType>
using VolumetricDataStructureTest = FixtureBase;

using VolumetricDataStructureTypes = ::testing::Types<
    DenseGrid<UnboundedOccupancyCell>, DenseGrid<SaturatingOccupancyCell>,
    HashedBlocks<UnboundedOccupancyCell>, HashedBlocks<SaturatingOccupancyCell>,
    ScalarQuadtree<UnboundedOccupancyCell>,
    ScalarQuadtree<SaturatingOccupancyCell>>;
TYPED_TEST_SUITE(VolumetricDataStructureTest, VolumetricDataStructureTypes);

// TODO(victorr): Test remaining interfaces of VolumetricDataStructure

TYPED_TEST(VolumetricDataStructureTest, Insertion) {
  constexpr int kNumRepetitions = 10;
  for (int i = 0; i < kNumRepetitions; ++i) {
    std::unique_ptr<VolumetricDataStructure> map_base_ptr =
        std::make_unique<TypeParam>(TestFixture::getRandomResolution());
    const std::vector<Index> random_indices =
        TestFixture::getRandomIndexVector();
    for (const Index& random_index : random_indices) {
      FloatingPoint expected_value = 0.f;
      map_base_ptr->setCellValue(random_index, 0.f);
      for (const FloatingPoint random_update :
           TestFixture::getRandomUpdateVector()) {
        map_base_ptr->addToCellValue(random_index, random_update);
        expected_value = std::max(TypeParam::CellType::kLowerBound,
                                  std::min(expected_value + random_update,
                                           TypeParam::CellType::kUpperBound));
      }
      EXPECT_NEAR(map_base_ptr->getCellValue(random_index), expected_value,
                  expected_value * 1e-6);
    }
  }
}
}  // namespace wavemap_2d

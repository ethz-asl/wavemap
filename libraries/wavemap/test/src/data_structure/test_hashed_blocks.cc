#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/data_structure/volumetric/cell_types/occupancy_cell.h"
#include "wavemap/data_structure/volumetric/hashed_blocks.h"
#include "wavemap/test/fixture_base.h"

namespace wavemap {
template <typename TypeParamT>
using DenseGridTest = FixtureBase;

template <int dim, typename CellT>
struct TypeParamTemplate {
  static constexpr int kDim = dim;
  using CellType = CellT;
};
using TypeParams =
    ::testing::Types<TypeParamTemplate<2, UnboundedOccupancyCell>,
                     TypeParamTemplate<2, SaturatingOccupancyCell>,
                     TypeParamTemplate<3, UnboundedOccupancyCell>,
                     TypeParamTemplate<3, SaturatingOccupancyCell>>;
TYPED_TEST_SUITE(DenseGridTest, TypeParams, );

// NOTE: Insertion tests are performed as part of the test suite for the
//       VolumetricDataStructure interface.

TYPED_TEST(DenseGridTest, Initialization) {
  constexpr int kDim = TypeParam::kDim;

  const FloatingPoint random_min_cell_width =
      TestFixture::getRandomMinCellWidth();
  HashedBlocks<typename TypeParam::CellType, kDim> map(
      VolumetricDataStructureConfig{random_min_cell_width});
  EXPECT_EQ(map.getMinCellWidth(), random_min_cell_width);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
  EXPECT_EQ(map.getMinIndex(), Index<kDim>::Zero());
  EXPECT_EQ(map.getMaxIndex(), Index<kDim>::Zero());
}
}  // namespace wavemap

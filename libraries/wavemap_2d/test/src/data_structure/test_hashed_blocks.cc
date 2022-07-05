#include <gtest/gtest.h>
#include <wavemap_common/common.h>

#include "wavemap_2d/data_structure/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/hashed_blocks.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap {
template <typename CellType>
using DenseGridTest = FixtureBase;

using CellTypes =
    ::testing::Types<UnboundedOccupancyCell, SaturatingOccupancyCell>;
TYPED_TEST_SUITE(DenseGridTest, CellTypes, );

// NOTE: Insertion tests are performed as part of the test suite for the
//       VolumetricDataStructure interface.

TYPED_TEST(DenseGridTest, Initialization) {
  const FloatingPoint random_min_cell_width =
      TestFixture::getRandomMinCellWidth();
  HashedBlocks<TypeParam> map(random_min_cell_width);
  EXPECT_EQ(map.getMinCellWidth(), random_min_cell_width);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
  EXPECT_EQ(map.getMinIndex(), Index::Zero());
  EXPECT_EQ(map.getMaxIndex(), Index::Zero());
}
}  // namespace wavemap

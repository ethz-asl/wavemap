#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/volumetric/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/volumetric/hashed_blocks.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
template <typename CellType>
using DenseGridTest = FixtureBase;

using CellTypes =
    ::testing::Types<UnboundedOccupancyCell, SaturatingOccupancyCell>;
TYPED_TEST_SUITE(DenseGridTest, CellTypes);

// NOTE: Insertion tests are performed as part of the test suite for the
//       VolumetricDataStructure interface.

TYPED_TEST(DenseGridTest, Initialization) {
  const FloatingPoint random_resolution = TestFixture::getRandomResolution();
  HashedBlocks<TypeParam> map(random_resolution);
  EXPECT_EQ(map.getResolution(), random_resolution);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
  EXPECT_EQ(map.getMinIndex(), Index::Zero());
  EXPECT_EQ(map.getMaxIndex(), Index::Zero());
}
}  // namespace wavemap_2d

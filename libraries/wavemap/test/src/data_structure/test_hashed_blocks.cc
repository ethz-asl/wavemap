#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/data_structure/volumetric/hashed_blocks.h"
#include "wavemap/test/fixture_base.h"

namespace wavemap {
using HashedBlocksTest = FixtureBase;

// NOTE: Insertion tests are performed as part of the test suite for the
//       VolumetricDataStructure interface.

TEST_F(HashedBlocksTest, Initialization) {
  const FloatingPoint random_min_cell_width = getRandomMinCellWidth();
  HashedBlocks map(VolumetricDataStructureConfig{random_min_cell_width});
  EXPECT_EQ(map.getMinCellWidth(), random_min_cell_width);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
  EXPECT_EQ(map.getMinIndex(), Index3D::Zero());
  EXPECT_EQ(map.getMaxIndex(), Index3D::Zero());
}
}  // namespace wavemap

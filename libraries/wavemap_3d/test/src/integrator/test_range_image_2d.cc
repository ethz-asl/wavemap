#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/test/fixture_base.h>

#include "wavemap_3d/integrator/scan_integrator/range_image_2d.h"

namespace wavemap {
using RangeImage2DTest = FixtureBase;

TEST_F(RangeImage2DTest, ConstructorAndAccessors) {
  for (int idx = 0; idx < 10; ++idx) {
    const IndexElement num_rows = getRandomIndexElement(1, 2048);
    const IndexElement num_columns = getRandomIndexElement(1, 2048);
    RangeImage2D range_image(num_rows, num_columns);

    EXPECT_FALSE(range_image.empty());
    EXPECT_EQ(range_image.size(), num_rows * num_columns);
    EXPECT_EQ(range_image.getNumRows(), num_rows);
    EXPECT_EQ(range_image.getNumColumns(), num_columns);

    range_image.clear();
    EXPECT_TRUE(range_image.empty());
    EXPECT_EQ(range_image.size(), 0);
    EXPECT_EQ(range_image.getNumRows(), 0);
    EXPECT_EQ(range_image.getNumColumns(), 0);

    const IndexElement new_num_rows = getRandomIndexElement(1, 1024);
    const IndexElement new_num_columns = getRandomIndexElement(1, 1024);
    range_image.resize(new_num_rows, new_num_columns);
    EXPECT_EQ(range_image.size(), new_num_rows * new_num_columns);
    EXPECT_EQ(range_image.getNumRows(), new_num_rows);
    EXPECT_EQ(range_image.getNumColumns(), new_num_columns);
  }
}
}  // namespace wavemap

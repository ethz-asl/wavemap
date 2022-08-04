#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/test/fixture_base.h>

#include "wavemap_2d/integrator/scan_integrator/range_image_1d.h"

namespace wavemap {
using RangeImageTest = FixtureBase;

TEST_F(RangeImageTest, ConstructorAndAccessors) {
  for (int idx = 0; idx < 10; ++idx) {
    const IndexElement num_beams = getRandomIndexElement(1, 2048);
    RangeImage1D range_image(num_beams);

    EXPECT_FALSE(range_image.empty());
    EXPECT_EQ(range_image.size(), num_beams);

    EXPECT_EQ(range_image.getNumBeams(), num_beams);

    range_image.clear();
    EXPECT_TRUE(range_image.empty());
    EXPECT_EQ(range_image.size(), 0);
    EXPECT_EQ(range_image.getNumBeams(), 0);

    const IndexElement new_num_beams = getRandomIndexElement(1, 1024);
    range_image.resize(new_num_beams);
    EXPECT_EQ(range_image.size(), new_num_beams);
    EXPECT_EQ(range_image.getNumBeams(), new_num_beams);
  }
}
}  // namespace wavemap

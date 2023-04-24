#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/data_structure/image.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
class ImageTest : public FixtureBase, public GeometryGenerator {};

TEST_F(ImageTest, ConstructorAndAccessors) {
  for (int idx = 0; idx < 10; ++idx) {
    const IndexElement num_rows = getRandomIndexElement(1, 2048);
    const IndexElement num_columns = getRandomIndexElement(1, 2048);
    Image<> image(num_rows, num_columns);

    EXPECT_FALSE(image.empty());
    EXPECT_EQ(image.size(), num_rows * num_columns);
    EXPECT_EQ(image.getNumRows(), num_rows);
    EXPECT_EQ(image.getNumColumns(), num_columns);

    image.clear();
    EXPECT_TRUE(image.empty());
    EXPECT_EQ(image.size(), 0);
    EXPECT_EQ(image.getNumRows(), 0);
    EXPECT_EQ(image.getNumColumns(), 0);

    const IndexElement new_num_rows = getRandomIndexElement(1, 1024);
    const IndexElement new_num_columns = getRandomIndexElement(1, 1024);
    image.resize(new_num_rows, new_num_columns);
    EXPECT_EQ(image.size(), new_num_rows * new_num_columns);
    EXPECT_EQ(image.getNumRows(), new_num_rows);
    EXPECT_EQ(image.getNumColumns(), new_num_columns);
  }
}
}  // namespace wavemap

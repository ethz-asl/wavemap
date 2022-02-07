#include <gtest/gtest.h>

#include "wavemap_2d/datastructure/cell.h"
#include "wavemap_2d/datastructure/hashed_blocks/hashed_blocks.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
template <typename CellType>
class DenseGridTest : public FixtureBase {};

using CellTypes = ::testing::Types<UnboundedCell, SaturatingCell<>>;
TYPED_TEST_SUITE(DenseGridTest, CellTypes);

TYPED_TEST(DenseGridTest, Initialization) {
  const FloatingPoint random_resolution = TestFixture::getRandomResolution();
  HashedBlocks<TypeParam> map(random_resolution);
  EXPECT_EQ(map.getResolution(), random_resolution);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
  EXPECT_EQ(map.getMinIndex(), Index::Zero());
  EXPECT_EQ(map.getMaxIndex(), Index::Zero());
}

TYPED_TEST(DenseGridTest, InsertionTest) {
  constexpr int kNumRepetitions = 10;
  for (int i = 0; i < kNumRepetitions; ++i) {
    HashedBlocks<TypeParam> map(TestFixture::getRandomResolution());
    const std::vector<Index> random_indices =
        TestFixture::getRandomIndexVector();
    for (const Index& random_index : random_indices) {
      FloatingPoint expected_value = 0.f;
      map.setCellValue(random_index, 0.f);
      for (const FloatingPoint random_update :
           TestFixture::getRandomUpdateVector()) {
        map.addToCellValue(random_index, random_update);
        expected_value = std::max(
            TypeParam::kLowerBound,
            std::min(expected_value + random_update, TypeParam::kUpperBound));
      }
      EXPECT_NEAR(map.getCellValue(random_index), expected_value,
                  expected_value * 1e-6);
    }
  }
}
}  // namespace wavemap_2d

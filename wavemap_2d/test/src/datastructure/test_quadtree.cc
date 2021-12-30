#include <gtest/gtest.h>

#include "wavemap_2d/datastructure/cell.h"
#include "wavemap_2d/datastructure/quadtree/quadtree.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
template <typename CellType>
class QuadtreeTest : public FixtureBase {
 protected:
  static void compare(const Quadtree<CellType>& map_reference,
                      const Quadtree<CellType>& map_to_test,
                      FloatingPoint cell_value_error_tolerance) {
    ASSERT_EQ(map_reference.empty(), map_to_test.empty());
    ASSERT_EQ(map_reference.size(), map_to_test.size());
    ASSERT_EQ(map_reference.getResolution(), map_to_test.getResolution());

    //    int reported_error_count = 0;
    //    constexpr int kMaxNumReportedErrors = 10;
    //    const Index min_index = map_reference.getMinIndex();
    //    const Index max_index = map_reference.getMaxIndex();
    //    for (Index index = min_index; index.x() <= max_index.x(); ++index.x())
    //    {
    //      for (index.y() = min_index.y(); index.y() <= max_index.y();
    //      ++index.y()) {
    //        auto reference_value = map_reference.getCellValue(index);
    //        auto test_value = map_to_test.getCellValue(index);
    //        if (cell_value_error_tolerance <
    //            std::abs(reference_value - test_value)) {
    //          ADD_FAILURE() << std::setprecision(4)
    //                        << "Difference between the reference ("
    //                        << reference_value << ") and test cell (" <<
    //                        test_value
    //                        << ") values at index (" << index
    //                        << ") exceeds the configured threshold ("
    //                        << cell_value_error_tolerance << ").";
    //          if (kMaxNumReportedErrors < reported_error_count++) {
    //            FAIL() << "Too many errors. Aborting comparison between
    //            reference "
    //                      "and test map.";
    //          }
    //        }
    //      }
    //    }
  }

  Quadtree<CellType> getRandomMap() {
    Quadtree<CellType> random_map(getRandomResolution());
    const Index min_index = -getRandomIndex().cwiseAbs();
    const Index max_index = getRandomIndex().cwiseAbs();
    random_map.addToCellValue(min_index, 0.f);
    random_map.addToCellValue(max_index, 0.f);
    for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
      for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
        random_map.addToCellValue(index, getRandomUpdate());
      }
    }
    return random_map;
  }
};

using CellTypes = ::testing::Types<UnboundedCell, SaturatingCell<>>;
TYPED_TEST_SUITE(QuadtreeTest, CellTypes);

TYPED_TEST(QuadtreeTest, Initialization) {
  const FloatingPoint random_resolution = TestFixture::getRandomResolution();
  Quadtree<TypeParam> map(random_resolution);
  EXPECT_EQ(map.getResolution(), random_resolution);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
}

TYPED_TEST(QuadtreeTest, Resizing) {
  constexpr int kNumRepetitions = 3;
  for (int i = 0; i < kNumRepetitions; ++i) {
    Quadtree<TypeParam> map(TestFixture::getRandomResolution());
    ASSERT_TRUE(map.empty());
    ASSERT_EQ(map.size(), 0u);

    const std::vector<Index> random_indices =
        TestFixture::getRandomIndexVector();

    const Index& first_random_index = random_indices[0];
    map.addToCellValue(first_random_index, 0.f);
    EXPECT_FALSE(map.empty());
    EXPECT_EQ(map.size(), map.getMaxDepth());

    Index min_index = first_random_index;
    Index max_index = first_random_index;
    for (auto index_it = ++random_indices.begin();
         index_it != random_indices.end(); ++index_it) {
      min_index = min_index.cwiseMin(*index_it);
      max_index = max_index.cwiseMax(*index_it);
      map.addToCellValue(*index_it, 0.f);
    }
    EXPECT_GE(map.size(), map.getMaxDepth());
    size_t max_unique_nodes = 0u;
    const size_t num_inserted_nodes = random_indices.size();
    for (int depth = 0; depth <= map.getMaxDepth(); ++depth) {
      const size_t max_unique_nodes_at_depth = std::exp2(MapDimension * depth);
      if (max_unique_nodes_at_depth < num_inserted_nodes) {
        max_unique_nodes += max_unique_nodes_at_depth;
      } else {
        max_unique_nodes += num_inserted_nodes;
      }
    }
    EXPECT_LE(map.size(), max_unique_nodes);

    for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
      for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
        EXPECT_FLOAT_EQ(map.getCellValue(index), 0.f);
      }
    }

    map.clear();
    EXPECT_TRUE(map.empty());
    EXPECT_EQ(map.size(), 0u);
  }
}

TYPED_TEST(QuadtreeTest, InsertionTest) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    Quadtree<TypeParam> map(TestFixture::getRandomResolution());
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

TEST(DummyTests, UnitializedMemory) {
  FloatingPoint dummy;
  std::cout << dummy << std::endl;
}

TEST(DummyTests, MemoryLeaks) {
  auto* dummy_ptr = new FloatingPoint;
  *dummy_ptr = 5.f;
}

TEST(DummyTests, UndefinedBehavior) {
  int k = 0x7fffffff;
  k += 0x7fffffff;
  std::cout << k << std::endl;
}
}  // namespace wavemap_2d

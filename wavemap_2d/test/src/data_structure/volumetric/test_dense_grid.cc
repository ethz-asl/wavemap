#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/volumetric/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/volumetric/dense_grid.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
template <typename CellType>
class DenseGridTest : public FixtureBase {
 protected:
  static void compare(const DenseGrid<CellType>& map_reference,
                      const DenseGrid<CellType>& map_to_test,
                      FloatingPoint cell_value_error_tolerance) {
    ASSERT_EQ(map_reference.empty(), map_to_test.empty());
    ASSERT_EQ(map_reference.size(), map_to_test.size());
    ASSERT_EQ(map_reference.getMinIndex(), map_to_test.getMinIndex());
    ASSERT_EQ(map_reference.getMaxIndex(), map_to_test.getMaxIndex());
    ASSERT_EQ(map_reference.getResolution(), map_to_test.getResolution());

    int reported_error_count = 0;
    constexpr int kMaxNumReportedErrors = 10;
    const Index min_index = map_reference.getMinIndex();
    const Index max_index = map_reference.getMaxIndex();
    for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
      for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
        auto reference_value = map_reference.getCellValue(index);
        auto test_value = map_to_test.getCellValue(index);
        if (cell_value_error_tolerance <
            std::abs(reference_value - test_value)) {
          ADD_FAILURE() << std::setprecision(4)
                        << "Difference between the reference ("
                        << reference_value << ") and test cell (" << test_value
                        << ") values at index (" << index
                        << ") exceeds the configured threshold ("
                        << cell_value_error_tolerance << ").";
          if (kMaxNumReportedErrors < reported_error_count++) {
            FAIL() << "Too many errors. Aborting comparison between reference "
                      "and test map.";
          }
        }
      }
    }
  }

  DenseGrid<CellType> getRandomMap() {
    DenseGrid<CellType> random_map(getRandomResolution());
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

using CellTypes =
    ::testing::Types<UnboundedOccupancyCell, SaturatingOccupancyCell>;
TYPED_TEST_SUITE(DenseGridTest, CellTypes, );

// NOTE: Insertion tests are performed as part of the test suite for the
//       VolumetricDataStructure interface.

TYPED_TEST(DenseGridTest, Initialization) {
  const FloatingPoint random_resolution = TestFixture::getRandomResolution();
  DenseGrid<TypeParam> map(random_resolution);
  EXPECT_EQ(map.getResolution(), random_resolution);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
  EXPECT_EQ(map.dimensions(), Index(0, 0));
  EXPECT_EQ(map.getMinIndex(), Index::Zero());
  EXPECT_EQ(map.getMaxIndex(), Index::Zero());
}

TYPED_TEST(DenseGridTest, Resizing) {
  constexpr int kNumRepetitions = 5;
  for (int i = 0; i < kNumRepetitions; ++i) {
    DenseGrid<TypeParam> map(TestFixture::getRandomResolution());
    ASSERT_TRUE(map.empty());
    ASSERT_EQ(map.dimensions(), Index(0, 0));

    const std::vector<Index> random_indices =
        TestFixture::getRandomIndexVector();

    const Index& first_random_index = random_indices[0];
    map.addToCellValue(first_random_index, 0.f);
    EXPECT_FALSE(map.empty());
    EXPECT_EQ(map.dimensions(), Index(1, 1));
    EXPECT_EQ(map.getMinIndex(), first_random_index);
    EXPECT_EQ(map.getMaxIndex(), first_random_index);

    Index min_index = first_random_index;
    Index max_index = first_random_index;
    for (auto index_it = ++random_indices.begin();
         index_it != random_indices.end(); ++index_it) {
      min_index = min_index.cwiseMin(*index_it);
      max_index = max_index.cwiseMax(*index_it);
      map.addToCellValue(*index_it, 0.f);
    }
    Index min_map_size = max_index - min_index + Index::Ones();
    EXPECT_EQ(map.dimensions(), min_map_size);
    EXPECT_EQ(map.getMinIndex(), min_index);
    EXPECT_EQ(map.getMaxIndex(), max_index);

    for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
      for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
        EXPECT_FLOAT_EQ(map.getCellValue(index), 0.f);
      }
    }

    map.clear();
    EXPECT_TRUE(map.empty());
    EXPECT_EQ(map.dimensions(), Index(0, 0));
    EXPECT_EQ(map.getMinIndex(), Index::Zero());
    EXPECT_EQ(map.getMaxIndex(), Index::Zero());
  }
}

TYPED_TEST(DenseGridTest, Serialization) {
  constexpr int kNumRepetitions = 2;
  for (int i = 0; i < kNumRepetitions; ++i) {
    constexpr FloatingPoint kUnboundedCellErrorValueTolerance = 0.5f;
    constexpr FloatingPoint kBoundedCellErrorValueTolerance = 1e-4f;

    const std::string data_structure_name =
        ::testing::UnitTest::GetInstance()->current_test_info()->type_param();
    for (const bool use_floating_precision : {true, false}) {
      const std::string kTempFilePath =
          "/tmp/map_" + data_structure_name +
          (use_floating_precision ? "_floating" : "_fixed");
      DenseGrid<TypeParam> map = TestFixture::getRandomMap();
      ASSERT_TRUE(map.save(kTempFilePath, use_floating_precision));

      DenseGrid<TypeParam> loaded_map(map.getResolution());
      ASSERT_TRUE(loaded_map.load(kTempFilePath, use_floating_precision));

      if (TypeParam::isFullyBounded) {
        TestFixture::compare(map, loaded_map, kBoundedCellErrorValueTolerance);
      } else {
        TestFixture::compare(map, loaded_map,
                             kUnboundedCellErrorValueTolerance);
      }
    }
  }
}
}  // namespace wavemap_2d

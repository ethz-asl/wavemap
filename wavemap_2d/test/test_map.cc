#include <gtest/gtest.h>

#include "wavemap_2d/datastructure/cell.h"
#include "wavemap_2d/datastructure/datastructure_base.h"
#include "wavemap_2d/datastructure/dense_grid.h"
#include "wavemap_2d/utils/random_number_generator.h"

namespace wavemap_2d {
template <typename DataStructureType>
class MapTest : public ::testing::Test {
 protected:
  void SetUp() override {
    random_number_generator_ = std::make_unique<RandomNumberGenerator>();
  }

  static void compare(const DataStructureBase& map_reference,
                      const DataStructureBase& map_to_test) {
    ASSERT_EQ(map_reference.empty(), map_to_test.empty());
    ASSERT_EQ(map_reference.size(), map_to_test.size());
    ASSERT_EQ(map_reference.getMinIndex(), map_to_test.getMinIndex());
    ASSERT_EQ(map_reference.getMaxIndex(), map_to_test.getMaxIndex());
    ASSERT_EQ(map_reference.getResolution(), map_to_test.getResolution());

    int reported_error_count = 0;
    constexpr int kMaxNumReportedErrors = 30;
    const Index min_index = map_reference.getMinIndex();
    const Index max_index = map_reference.getMaxIndex();
    for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
      for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
        auto reference_value = map_reference.getCellValue(index);
        auto test_value = map_to_test.getCellValue(index);
        if (kCellValueErrorTolerance < std::abs(reference_value - test_value)) {
          ADD_FAILURE() << std::setprecision(4)
                        << "Difference between the reference ("
                        << reference_value << ") and test cell (" << test_value
                        << ") values at index (" << index
                        << ") exceeds the configured threshold ("
                        << kCellValueErrorTolerance << ").";
          if (kMaxNumReportedErrors < reported_error_count++) {
            FAIL() << "Too many errors. Aborting comparison between reference "
                      "and test map.";
          }
        }
      }
    }
  }

  FloatingPoint getRandomResolution() const {
    constexpr FloatingPoint kMinResolution = 1e-3;
    constexpr FloatingPoint kMaxResolution = 1e2;
    return random_number_generator_->getRandomRealNumber(kMinResolution,
                                                         kMaxResolution);
  }
  IndexElement getRandomIndexElement() const {
    constexpr IndexElement kMinCoordinate = -1e3;
    constexpr IndexElement kMaxCoordinate = 1e3;
    return random_number_generator_->getRandomInteger(kMinCoordinate,
                                                      kMaxCoordinate);
  }
  Index getRandomIndex() const {
    return {getRandomIndexElement(), getRandomIndexElement()};
  }
  std::vector<Index> getRandomIndexVector() const {
    constexpr size_t kMinNumIndices = 2u;
    constexpr size_t kMaxNumIndices = 100u;
    const size_t num_indices = random_number_generator_->getRandomInteger(
        kMinNumIndices, kMaxNumIndices);
    std::vector<Index> random_indices;
    random_indices.reserve(num_indices);
    for (size_t idx = 0u; idx < num_indices; ++idx) {
      random_indices.emplace_back(getRandomIndex());
    }
    return random_indices;
  }
  FloatingPoint getRandomUpdate() const {
    constexpr FloatingPoint kMinUpdate = 1e-2;
    constexpr FloatingPoint kMaxUpdate = 1e2;
    return random_number_generator_->getRandomRealNumber(kMinUpdate,
                                                         kMaxUpdate);
  }
  std::vector<FloatingPoint> getRandomUpdateVector() const {
    constexpr size_t kMinNumUpdates = 0u;
    constexpr size_t kMaxNumUpdates = 100u;
    const size_t num_updates = random_number_generator_->getRandomInteger(
        kMinNumUpdates, kMaxNumUpdates);
    std::vector<FloatingPoint> random_updates(num_updates);
    std::generate(random_updates.begin(), random_updates.end(),
                  [this]() { return getRandomUpdate(); });
    return random_updates;
  }
  DataStructureType getRandomMap() {
    DataStructureType random_map(getRandomResolution());
    const Index min_index = -getRandomIndex().cwiseAbs();
    const Index max_index = getRandomIndex().cwiseAbs();
    random_map.updateCell(min_index, 0.f);
    random_map.updateCell(max_index, 0.f);
    for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
      for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
        random_map.updateCell(index, getRandomUpdate());
      }
    }
    return random_map;
  }

 private:
  // TODO(victorr): Tighten this once truncation and rescaling has been
  //                implemented
  static constexpr FloatingPoint kCellValueErrorTolerance = 1.f;
  std::unique_ptr<RandomNumberGenerator> random_number_generator_;
};

using DataStructureTypes =
    ::testing::Types<DenseGrid<UnboundedCell>, DenseGrid<SaturatingCell<>>>;
TYPED_TEST_SUITE(MapTest, DataStructureTypes);

TYPED_TEST(MapTest, Initialization) {
  const FloatingPoint random_resolution = TestFixture::getRandomResolution();
  TypeParam map(random_resolution);
  EXPECT_EQ(map.getResolution(), random_resolution);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), Index(0, 0));
  EXPECT_EQ(map.getMinIndex(), Index::Zero());
  EXPECT_EQ(map.getMaxIndex(), Index::Zero());
}

TYPED_TEST(MapTest, Resizing) {
  TypeParam map(TestFixture::getRandomResolution());
  ASSERT_TRUE(map.empty());
  ASSERT_EQ(map.size(), Index(0, 0));

  const std::vector<Index> random_indices = TestFixture::getRandomIndexVector();

  const Index& first_random_index = random_indices[0];
  map.updateCell(first_random_index, 0.f);
  EXPECT_FALSE(map.empty());
  EXPECT_EQ(map.size(), Index(1, 1));
  EXPECT_EQ(map.getMinIndex(), first_random_index);
  EXPECT_EQ(map.getMaxIndex(), first_random_index);

  Index min_index = first_random_index;
  Index max_index = first_random_index;
  for (auto index_it = ++random_indices.begin();
       index_it != random_indices.end(); ++index_it) {
    min_index = min_index.cwiseMin(*index_it);
    max_index = max_index.cwiseMax(*index_it);
    map.updateCell(*index_it, 0.f);
  }
  Index min_map_size = max_index - min_index + Index::Ones();
  EXPECT_EQ(map.size(), min_map_size);
  EXPECT_EQ(map.getMinIndex(), min_index);
  EXPECT_EQ(map.getMaxIndex(), max_index);

  for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
    for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
      EXPECT_FLOAT_EQ(map.getCellValue(index), 0.f);
    }
  }

  map.clear();
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), Index(0, 0));
  EXPECT_EQ(map.getMinIndex(), Index::Zero());
  EXPECT_EQ(map.getMaxIndex(), Index::Zero());
}

TYPED_TEST(MapTest, InsertionTest) {
  TypeParam map(TestFixture::getRandomResolution());
  const std::vector<Index> random_indices = TestFixture::getRandomIndexVector();
  for (const Index& random_index : random_indices) {
    FloatingPoint expected_value = 0.f;
    for (const FloatingPoint random_update :
         TestFixture::getRandomUpdateVector()) {
      map.updateCell(random_index, random_update);
      expected_value = std::max(TypeParam::cell_type::kLowerBound,
                                std::min(expected_value + random_update,
                                         TypeParam::cell_type::kUpperBound));
    }
    EXPECT_NEAR(map.getCellValue(random_index), expected_value,
                expected_value * 1e-2);
  }
}

TYPED_TEST(MapTest, Serialization) {
  const std::string datastructure_name =
      ::testing::UnitTest::GetInstance()->current_test_info()->type_param();
  for (const bool use_floating_precision : {true, false}) {
    const std::string kTempFilePath =
        "/tmp/map_" + datastructure_name +
        (use_floating_precision ? "_floating" : "_fixed");
    TypeParam map = TestFixture::getRandomMap();
    ASSERT_TRUE(map.save(kTempFilePath, use_floating_precision));

    TypeParam loaded_map(map.getResolution());
    ASSERT_TRUE(loaded_map.load(kTempFilePath, use_floating_precision));

    TestFixture::compare(loaded_map, map);
  }
}

TEST(CellTest, CellTraits) {
  using UnboundedCell = CellTraits<FloatingPoint, FloatingPoint, int>;
  EXPECT_FALSE(UnboundedCell::hasLowerBound);
  EXPECT_FALSE(UnboundedCell::hasUpperBound);

  using LowerBoundedCell = CellTraits<FloatingPoint, FloatingPoint, int, -2>;
  EXPECT_TRUE(LowerBoundedCell::hasLowerBound);
  EXPECT_FALSE(LowerBoundedCell::hasUpperBound);

  using UpperBoundedCell =
      CellTraits<FloatingPoint, FloatingPoint, int,
                 std::numeric_limits<BoundType>::lowest(), 4>;
  EXPECT_FALSE(UpperBoundedCell::hasLowerBound);
  EXPECT_TRUE(UpperBoundedCell::hasUpperBound);

  using FullyBoundedCell = CellTraits<FloatingPoint, FloatingPoint, int, -2, 4>;
  EXPECT_TRUE(FullyBoundedCell::hasLowerBound);
  EXPECT_TRUE(FullyBoundedCell::hasUpperBound);
}
}  // namespace wavemap_2d

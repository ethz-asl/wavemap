#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/generic/pointcloud.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
class PointcloudTest : public FixtureBase {
 protected:
  static void compare(const std::vector<Point>& point_vector,
                      const Pointcloud<>& pointcloud) {
    ASSERT_EQ(point_vector.empty(), pointcloud.empty());
    ASSERT_EQ(point_vector.size(), pointcloud.size());
    size_t point_idx = 0u;
    for (const Point& point : point_vector) {
      EXPECT_EQ(point, pointcloud[point_idx++]);
    }
  }
  static void compare(const Pointcloud<>::PointcloudData& point_matrix,
                      const Pointcloud<>& pointcloud) {
    ASSERT_EQ(point_matrix.size() == 0, pointcloud.empty());
    ASSERT_EQ(point_matrix.cols(), pointcloud.size());
    for (Eigen::Index point_idx = 0; point_idx < point_matrix.cols();
         ++point_idx) {
      EXPECT_EQ(point_matrix.col(point_idx), pointcloud[point_idx]);
    }
  }
  static void compare(const Pointcloud<>& pointcloud_reference,
                      const Pointcloud<>& pointcloud_to_test) {
    ASSERT_EQ(pointcloud_reference.empty(), pointcloud_to_test.empty());
    ASSERT_EQ(pointcloud_reference.size(), pointcloud_to_test.size());
    for (size_t point_idx = 0u; point_idx < pointcloud_reference.size();
         ++point_idx) {
      EXPECT_EQ(pointcloud_reference[point_idx], pointcloud_to_test[point_idx]);
    }
  }

  unsigned int getRandomPointcloudSize() const {
    constexpr unsigned int kMinSize = 1u;
    constexpr unsigned int kMaxSize = 1000u;
    return random_number_generator_->getRandomInteger(kMinSize, kMaxSize);
  }
  std::vector<Point> getRandomPointVector() const {
    std::vector<Point> random_point_vector(getRandomPointcloudSize());
    std::generate(random_point_vector.begin(), random_point_vector.end(),
                  []() { return getRandomPoint(); });
    return random_point_vector;
  }
  Pointcloud<>::PointcloudData getRandomPointMatrix() const {
    constexpr FloatingPoint kMaxCoordinate = 1e3;
    const Eigen::Index random_length = getRandomPointcloudSize();
    Pointcloud<>::PointcloudData random_point_matrix =
        Pointcloud<>::PointcloudData::Random(Pointcloud<>::kPointDimensions,
                                             random_length);
    random_point_matrix *= kMaxCoordinate;
    return random_point_matrix;
  }
};

TEST_F(PointcloudTest, DefaultInitialize) {
  Pointcloud<> default_pointcloud;
  EXPECT_TRUE(default_pointcloud.empty());
  EXPECT_EQ(default_pointcloud.size(), 0u);
  for (auto it = default_pointcloud.begin(); it != default_pointcloud.end();
       ++it) {
    ADD_FAILURE();
  }
}

TEST_F(PointcloudTest, GetAndSetSize) {
  Pointcloud<> pointcloud;
  ASSERT_TRUE(pointcloud.empty());
  ASSERT_EQ(pointcloud.size(), 0u);

  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    unsigned int random_new_size = getRandomPointcloudSize();
    pointcloud.resize(random_new_size);
    EXPECT_FALSE(pointcloud.empty());
    EXPECT_EQ(pointcloud.size(), random_new_size);

    pointcloud.clear();
    EXPECT_TRUE(pointcloud.empty());
    EXPECT_EQ(pointcloud.size(), 0u);
  }
}

TEST_F(PointcloudTest, InitializeFromStl) {
  std::vector<Point> empty_point_vector;
  Pointcloud<> empty_pointcloud(empty_point_vector);
  EXPECT_TRUE(empty_pointcloud.empty());

  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    std::vector<Point> random_point_vector = getRandomPointVector();
    Pointcloud<> random_pointcloud(random_point_vector);
    compare(random_point_vector, random_pointcloud);
  }
}

TEST_F(PointcloudTest, InitializeFromEigen) {
  Pointcloud<>::PointcloudData empty_point_matrix;
  Pointcloud<> empty_pointcloud(empty_point_matrix);
  EXPECT_TRUE(empty_pointcloud.empty());

  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    Pointcloud<>::PointcloudData random_point_matrix = getRandomPointMatrix();
    Pointcloud<> random_pointcloud(random_point_matrix);
    compare(random_point_matrix, random_pointcloud);
  }
}

TEST_F(PointcloudTest, CopyInitializationAndAssignment) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    const Pointcloud<> random_pointcloud(getRandomPointMatrix());

    Pointcloud<> copy_initialized_random_pointcloud(random_pointcloud);
    compare(random_pointcloud, copy_initialized_random_pointcloud);

    Pointcloud<> copy_assigned_random_pointcloud;
    copy_assigned_random_pointcloud = random_pointcloud;
    compare(random_pointcloud, copy_assigned_random_pointcloud);
  }
}

TEST_F(PointcloudTest, Iterators) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    std::vector<Point> random_point_vector = getRandomPointVector();

    Pointcloud<> random_pointcloud(random_point_vector);
    size_t point_idx = 0u;
    for (const auto& random_point : random_pointcloud) {
      EXPECT_EQ(random_point, random_point_vector[point_idx++]);
    }
    EXPECT_EQ(point_idx, random_pointcloud.size());

    const Pointcloud<> const_random_pointcloud(random_point_vector);
    point_idx = 0u;
    for (const auto& random_point : const_random_pointcloud) {
      EXPECT_EQ(random_point, random_point_vector[point_idx++]);
    }
    EXPECT_EQ(point_idx, const_random_pointcloud.size());

    // TODO(victorr): Add test that checks if modifying when iterating by
    //                reference changes the source data as expected
  }
}

class PosedPointcloudTest : public PointcloudTest {
 protected:
  static void compare(const PosedPointcloud<>& pointcloud_reference,
                      const PosedPointcloud<>& pointcloud_to_test) {
    EXPECT_EQ(pointcloud_reference.getOrigin(), pointcloud_to_test.getOrigin());
    EXPECT_EQ(pointcloud_reference.getPose(), pointcloud_to_test.getPose());

    PointcloudTest::compare(pointcloud_reference.getPointsLocal(),
                            pointcloud_to_test.getPointsLocal());
    PointcloudTest::compare(pointcloud_reference.getPointsGlobal(),
                            pointcloud_to_test.getPointsGlobal());
  }

  Transformation getRandomTransformation() {
    const Rotation random_rotation(getRandomAngle());
    const Vector random_translation(Point().setRandom());
    return {random_rotation, random_translation};
  }
};

TEST_F(PosedPointcloudTest, InitializationAndCopying) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    // Initialize
    const Transformation random_transformation = getRandomTransformation();
    const Pointcloud<> random_pointcloud(getRandomPointVector());
    PosedPointcloud random_posed_pointcloud(random_transformation,
                                            random_pointcloud);

    // Test initialization
    EXPECT_EQ(random_posed_pointcloud.getOrigin(),
              random_transformation.getPosition());
    EXPECT_EQ(random_posed_pointcloud.getPose(), random_transformation);
    PointcloudTest::compare(random_posed_pointcloud.getPointsLocal(),
                            random_pointcloud);

    // Test copying
    PosedPointcloud copy_initialized_posed_pointcloud(random_posed_pointcloud);
    compare(random_posed_pointcloud, copy_initialized_posed_pointcloud);

    PosedPointcloud copy_assigned_posed_pointcloud;
    copy_assigned_posed_pointcloud = random_posed_pointcloud;
    compare(random_posed_pointcloud, copy_assigned_posed_pointcloud);
  }
}

TEST_F(PosedPointcloudTest, Transformation) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    const std::vector<Point> random_points_C = getRandomPointVector();
    const Transformation random_T_W_C = getRandomTransformation();
    PosedPointcloud random_posed_pointcloud(random_T_W_C,
                                            Pointcloud<>(random_points_C));

    Pointcloud<> pointcloud_W = random_posed_pointcloud.getPointsGlobal();
    ASSERT_EQ(pointcloud_W.size(), random_points_C.size());
    size_t point_idx = 0u;
    for (const Point& point_C : random_points_C) {
      const Point point_W = random_T_W_C * point_C;
      const Point& posed_pointcloud_point_W = pointcloud_W[point_idx];

      for (int axis = 0; axis < Point::Base::RowsAtCompileTime; ++axis) {
        EXPECT_FLOAT_EQ(point_W[axis], posed_pointcloud_point_W[axis])
            << " mismatch for point " << point_idx << " along axis " << axis;
      }
      ++point_idx;
    }
  }
}
}  // namespace wavemap_2d

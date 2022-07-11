#include <gtest/gtest.h>

#include "wavemap_common/common.h"
#include "wavemap_common/data_structure/pointcloud.h"
#include "wavemap_common/test/fixture_base.h"
#include "wavemap_common/utils/eigen_format.h"

namespace wavemap {
template <typename PointT>
class PointcloudTest : public FixtureBase {
 protected:
  static void compare(const std::vector<PointT>& point_vector,
                      const Pointcloud<PointT>& pointcloud) {
    ASSERT_EQ(point_vector.empty(), pointcloud.empty());
    ASSERT_EQ(point_vector.size(), pointcloud.size());
    size_t point_idx = 0u;
    for (const PointT& point : point_vector) {
      EXPECT_EQ(point, pointcloud[point_idx++]);
    }
  }
  static void compare(
      const typename Pointcloud<PointT>::PointcloudData& point_matrix,
      const Pointcloud<PointT>& pointcloud) {
    ASSERT_EQ(point_matrix.size() == 0, pointcloud.empty());
    ASSERT_EQ(point_matrix.cols(), pointcloud.size());
    for (Eigen::Index point_idx = 0; point_idx < point_matrix.cols();
         ++point_idx) {
      EXPECT_EQ(point_matrix.col(point_idx), pointcloud[point_idx]);
    }
  }
  static void compare(const Pointcloud<PointT>& pointcloud_reference,
                      const Pointcloud<PointT>& pointcloud_to_test) {
    ASSERT_EQ(pointcloud_reference.empty(), pointcloud_to_test.empty());
    ASSERT_EQ(pointcloud_reference.size(), pointcloud_to_test.size());
    for (size_t point_idx = 0u; point_idx < pointcloud_reference.size();
         ++point_idx) {
      EXPECT_EQ(pointcloud_reference[point_idx], pointcloud_to_test[point_idx]);
    }
  }

  typename Pointcloud<PointT>::PointcloudData getRandomPointMatrix() const {
    constexpr FloatingPoint kMaxCoordinate = 1e3;
    const Eigen::Index random_length = getRandomPointcloudSize();
    typename Pointcloud<PointT>::PointcloudData random_point_matrix =
        Pointcloud<PointT>::PointcloudData::Random(dim<PointT>, random_length);
    random_point_matrix *= kMaxCoordinate;
    return random_point_matrix;
  }
};

using PointTypes = ::testing::Types<Point2D, Point3D>;
TYPED_TEST_SUITE(PointcloudTest, PointTypes, );

TYPED_TEST(PointcloudTest, DefaultInitialize) {
  Pointcloud<TypeParam> default_pointcloud;
  EXPECT_TRUE(default_pointcloud.empty());
  EXPECT_EQ(default_pointcloud.size(), 0u);

  // Ensure that the iterator range is also empty
  std::for_each(default_pointcloud.begin(), default_pointcloud.end(),
                [](const auto&) { ADD_FAILURE(); });
}

TYPED_TEST(PointcloudTest, GetAndSetSize) {
  Pointcloud<TypeParam> pointcloud;
  ASSERT_TRUE(pointcloud.empty());
  ASSERT_EQ(pointcloud.size(), 0u);

  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    unsigned int random_new_size = TestFixture::getRandomPointcloudSize();
    pointcloud.resize(random_new_size);
    EXPECT_FALSE(pointcloud.empty());
    EXPECT_EQ(pointcloud.size(), random_new_size);

    pointcloud.clear();
    EXPECT_TRUE(pointcloud.empty());
    EXPECT_EQ(pointcloud.size(), 0u);
  }
}

TYPED_TEST(PointcloudTest, InitializeFromStl) {
  std::vector<TypeParam> empty_point_vector;
  Pointcloud<TypeParam> empty_pointcloud(empty_point_vector);
  EXPECT_TRUE(empty_pointcloud.empty());

  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    const auto random_point_vector =
        TestFixture::template getRandomPointVector<dim<TypeParam>>();
    Pointcloud<TypeParam> random_pointcloud(random_point_vector);
    TestFixture::compare(random_point_vector, random_pointcloud);
  }
}

TYPED_TEST(PointcloudTest, InitializeFromEigen) {
  typename Pointcloud<TypeParam>::PointcloudData empty_point_matrix;
  Pointcloud<TypeParam> empty_pointcloud(empty_point_matrix);
  EXPECT_TRUE(empty_pointcloud.empty());

  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    typename Pointcloud<TypeParam>::PointcloudData random_point_matrix =
        TestFixture::getRandomPointMatrix();
    Pointcloud<TypeParam> random_pointcloud(random_point_matrix);
    TestFixture::compare(random_point_matrix, random_pointcloud);
  }
}

TYPED_TEST(PointcloudTest, CopyInitializationAndAssignment) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    const Pointcloud<TypeParam> random_pointcloud(
        TestFixture::getRandomPointMatrix());

    Pointcloud<TypeParam> copy_initialized_random_pointcloud(random_pointcloud);
    TestFixture::compare(random_pointcloud, copy_initialized_random_pointcloud);

    Pointcloud<TypeParam> copy_assigned_random_pointcloud;
    copy_assigned_random_pointcloud = random_pointcloud;
    TestFixture::compare(random_pointcloud, copy_assigned_random_pointcloud);
  }
}

TYPED_TEST(PointcloudTest, Iterators) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    const auto random_point_vector =
        TestFixture::template getRandomPointVector<dim<TypeParam>>();

    Pointcloud<TypeParam> random_pointcloud(random_point_vector);
    size_t point_idx = 0u;
    for (const auto& random_point : random_pointcloud) {
      EXPECT_EQ(random_point, random_point_vector[point_idx++]);
    }
    EXPECT_EQ(point_idx, random_pointcloud.size());

    const Pointcloud<TypeParam> const_random_pointcloud(random_point_vector);
    point_idx = 0u;
    for (const auto& random_point : const_random_pointcloud) {
      EXPECT_EQ(random_point, random_point_vector[point_idx++]);
    }
    EXPECT_EQ(point_idx, const_random_pointcloud.size());
  }
}

template <typename PointPoseT>
class PosedPointcloudTest
    : public PointcloudTest<typename PointPoseT::first_type> {
 protected:
  using PointType = typename PointPoseT::first_type;
  using PoseType = typename PointPoseT::second_type;

  static void compare(
      const PosedPointcloud<PointType, PoseType>& pointcloud_reference,
      const PosedPointcloud<PointType, PoseType>& pointcloud_to_test) {
    EXPECT_EQ(pointcloud_reference.getOrigin(), pointcloud_to_test.getOrigin());
    EXPECT_EQ(pointcloud_reference.getPose(), pointcloud_to_test.getPose());

    PointcloudTest<PointType>::compare(pointcloud_reference.getPointsLocal(),
                                       pointcloud_to_test.getPointsLocal());
    PointcloudTest<PointType>::compare(pointcloud_reference.getPointsGlobal(),
                                       pointcloud_to_test.getPointsGlobal());
  }
};

using PointPoseTypes = ::testing::Types<std::pair<Point2D, Transformation2D>,
                                        std::pair<Point3D, Transformation3D>>;
TYPED_TEST_SUITE(PosedPointcloudTest, PointPoseTypes, );

TYPED_TEST(PosedPointcloudTest, InitializationAndCopying) {
  using PointType = typename TypeParam::first_type;
  using PoseType = typename TypeParam::second_type;
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    // Initialize
    const PoseType random_transformation =
        TestFixture::template getRandomTransformation<dim<PoseType>>();
    const Pointcloud<PointType> random_pointcloud(
        TestFixture::template getRandomPointVector<dim<PointType>>());
    PosedPointcloud random_posed_pointcloud(random_transformation,
                                            random_pointcloud);

    // Test initialization
    EXPECT_EQ(random_posed_pointcloud.getOrigin(),
              random_transformation.getPosition());
    EXPECT_EQ(random_posed_pointcloud.getPose(), random_transformation);
    PointcloudTest<PointType>::compare(random_posed_pointcloud.getPointsLocal(),
                                       random_pointcloud);

    // Test copying
    PosedPointcloud copy_initialized_posed_pointcloud(random_posed_pointcloud);
    TestFixture::compare(random_posed_pointcloud,
                         copy_initialized_posed_pointcloud);

    PosedPointcloud<PointType, PoseType> copy_assigned_posed_pointcloud;
    copy_assigned_posed_pointcloud = random_posed_pointcloud;
    TestFixture::compare(random_posed_pointcloud,
                         copy_assigned_posed_pointcloud);
  }
}

TYPED_TEST(PosedPointcloudTest, Transformations) {
  using PointType = typename TypeParam::first_type;
  using PoseType = typename TypeParam::second_type;
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    const std::vector<PointType> random_points_C =
        TestFixture::template getRandomPointVector<dim<PointType>>();
    const PoseType random_T_W_C =
        TestFixture::template getRandomTransformation<dim<PoseType>>();
    const PosedPointcloud random_posed_pointcloud(
        random_T_W_C, Pointcloud<PointType>(random_points_C));

    const Pointcloud pointcloud_W = random_posed_pointcloud.getPointsGlobal();
    ASSERT_EQ(pointcloud_W.size(), random_points_C.size());
    size_t point_idx = 0u;
    constexpr FloatingPoint kMaxAcceptableOffset = 1e-3f;
    for (const auto& point_C : random_points_C) {
      const PointType point_W = random_T_W_C * point_C;
      const PointType& posed_pointcloud_point_W = pointcloud_W[point_idx];
      EXPECT_LE((posed_pointcloud_point_W - point_W).norm(),
                kMaxAcceptableOffset)
          << " for points " << EigenFormat::oneLine(posed_pointcloud_point_W)
          << " and " << EigenFormat::oneLine(point_W);
      ++point_idx;
    }
  }
}
}  // namespace wavemap

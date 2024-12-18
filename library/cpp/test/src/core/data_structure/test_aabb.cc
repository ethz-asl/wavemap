#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/print/eigen.h"
#include "wavemap/core/utils/shape/aabb.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
template <typename PointT>
class AabbTest : public FixtureBase, public GeometryGenerator {};

// TODO(victorr): Template AABB class and tests directly on dim instead
using PointTypes = ::testing::Types<Point2D, Point3D>;
TYPED_TEST_SUITE(AabbTest, PointTypes, );

TYPED_TEST(AabbTest, InitializationAndInclusion) {
  for (const auto& random_point :
       GeometryGenerator::getRandomPointVector<dim_v<TypeParam>>()) {
    AABB<TypeParam> aabb;
    EXPECT_FALSE(aabb.contains(random_point))
        << "The uninitialized AABB should be empty.";
    aabb.insert(random_point);
    EXPECT_TRUE(aabb.contains(random_point))
        << "The AABB should contain points after they have been included.";
  }
}

TYPED_TEST(AabbTest, ClosestPointsAndDistances) {
  struct QueryAndExpectedResults {
    AABB<TypeParam> aabb;
    TypeParam query_point;

    QueryAndExpectedResults(AABB<TypeParam> aabb, TypeParam query_point)
        : aabb(std::move(aabb)), query_point(std::move(query_point)) {}

    std::string getDescription() const {
      std::stringstream ss;
      ss << "For aabb " << aabb.toString() << " and query_point "
         << print::eigen::oneLine(query_point) << ".";
      return ss.str();
    }
  };

  // Generate test set
  std::vector<QueryAndExpectedResults> tests;
  {
    std::vector<AABB<TypeParam>> aabbs{{TypeParam::Zero(), TypeParam::Ones()}};
    for (int dim_idx = 0; dim_idx < dim_v<TypeParam>; ++dim_idx) {
      const TypeParam min_corner = TypeParam::Zero();
      TypeParam max_corner = TypeParam::Ones();
      max_corner[dim_idx] = 0.5f;
      aabbs.emplace_back(min_corner, max_corner);
    }
    for (const auto& aabb : aabbs) {
      tests.emplace_back(aabb, TypeParam::Zero());
    }
    for (int direction = 1; direction < AABB<TypeParam>::kNumCorners;
         ++direction) {
      for (const FloatingPoint scale : {0.1f, 1.f, 3.f, 30.f}) {
        for (const FloatingPoint sign : {-1.f, 1.f}) {
          Vector<dim_v<TypeParam>> translation;
          for (int dim_idx = 0; dim_idx < dim_v<TypeParam>; ++dim_idx) {
            translation[dim_idx] = (direction >> dim_idx) & 0b1;
          }
          translation *= sign * scale;
          for (const auto& aabb : aabbs) {
            const AABB<TypeParam> aabb_translated{aabb.min + translation,
                                                  aabb.max + translation};
            tests.emplace_back(aabb, translation);
            tests.emplace_back(aabb_translated, TypeParam::Zero());
            tests.emplace_back(aabb_translated, translation);
          }
        }
      }
    }
  }

  // Run tests
  for (const QueryAndExpectedResults& test : tests) {
    // Find the closest and furthest point
    TypeParam closest_point;
    TypeParam furthest_point;
    // Check for closest/furthest points on the AABB's edges
    for (int dim_idx = 0; dim_idx < dim_v<TypeParam>; ++dim_idx) {
      const FloatingPoint query_coord = test.query_point[dim_idx];
      const FloatingPoint aabb_min_coord = test.aabb.min[dim_idx];
      const FloatingPoint aabb_max_coord = test.aabb.max[dim_idx];
      closest_point[dim_idx] =
          std::clamp(query_coord, aabb_min_coord, aabb_max_coord);
      if (std::abs(aabb_min_coord - query_coord) <
          std::abs(aabb_max_coord - query_coord)) {
        furthest_point[dim_idx] = aabb_max_coord;
      } else {
        furthest_point[dim_idx] = aabb_min_coord;
      }
    }
    const FloatingPoint min_distance =
        (closest_point - test.query_point).norm();
    const FloatingPoint max_distance =
        (furthest_point - test.query_point).norm();
    if ((test.aabb.min.array() <= test.query_point.array() &&
         test.query_point.array() <= test.aabb.max.array())
            .all()) {
      ASSERT_LE(min_distance, kEpsilon);
      ASSERT_NEAR(closest_point.x(), test.query_point.x(), kEpsilon);
      ASSERT_NEAR(closest_point.y(), test.query_point.y(), kEpsilon);
    }

    // Check closest and furthest point queries and distances
    if (min_distance <= 0) {
      EXPECT_TRUE(test.aabb.contains(test.query_point))
          << test.getDescription();
    }

    const TypeParam returned_closest_point =
        test.aabb.closestPointTo(test.query_point);
    EXPECT_NEAR(returned_closest_point.x(), closest_point.x(), kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(returned_closest_point.y(), closest_point.y(), kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(test.aabb.minDistanceTo(test.query_point), min_distance,
                kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(test.aabb.minSquaredDistanceTo(test.query_point),
                (test.query_point - closest_point).squaredNorm(), kEpsilon)
        << test.getDescription();

    const TypeParam returned_furthest_point =
        test.aabb.furthestPointFrom(test.query_point);
    EXPECT_NEAR(returned_furthest_point.x(), furthest_point.x(), kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(returned_furthest_point.y(), furthest_point.y(), kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(test.aabb.maxDistanceTo(test.query_point), max_distance,
                kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(test.aabb.maxSquaredDistanceTo(test.query_point),
                (test.query_point - furthest_point).squaredNorm(), kEpsilon)
        << test.getDescription();
  }
}
}  // namespace wavemap

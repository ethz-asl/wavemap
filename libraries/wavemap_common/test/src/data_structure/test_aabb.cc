#include <gtest/gtest.h>

#include "wavemap_common/common.h"
#include "wavemap_common/data_structure/aabb.h"
#include "wavemap_common/test/fixture_base.h"
#include "wavemap_common/utils/eigen_format.h"

namespace wavemap {
using AabbTest = FixtureBase;

// TODO(victorr): Also test for 3D
constexpr int kDim = 2;

TEST_F(AabbTest, InitializationAndInclusion) {
  for (const Point<kDim>& random_point : getRandomPointVector<kDim>()) {
    AABB<Point<kDim>> aabb;
    EXPECT_FALSE(aabb.containsPoint(random_point))
        << "The uninitialized AABB should be empty.";
    aabb.includePoint(random_point);
    EXPECT_TRUE(aabb.containsPoint(random_point))
        << "The AABB should contain points after they have been included.";
  }
}

TEST_F(AabbTest, ClosestPointsAndDistances) {
  struct QueryAndExpectedResults {
    AABB<Point<kDim>> aabb;
    Point<kDim> query_point;

    QueryAndExpectedResults(AABB<Point<kDim>> aabb, Point<kDim> query_point)
        : aabb(std::move(aabb)), query_point(std::move(query_point)) {}

    std::string getDescription() const {
      std::stringstream ss;
      ss << "For aabb " << aabb.toString() << " and query_point "
         << EigenFormat::oneLine(query_point) << ".";
      return ss.str();
    }
  };

  // Generate test set
  std::vector<QueryAndExpectedResults> tests;
  {
    std::vector<AABB<Point<kDim>>> aabbs{
        {Point<kDim>::Zero(), Point<kDim>::Ones()},
        {Point<kDim>::Zero(), Point<kDim>{0.5f, 1.f}},
        {Point<kDim>::Zero(), Point<kDim>{1.f, 0.5f}}};
    for (const auto& aabb : aabbs) {
      tests.emplace_back(aabb, Point<kDim>::Zero());
    }
    for (int direction = 1; direction < 4; ++direction) {
      for (const FloatingPoint scale : {0.1f, 1.f, 3.f, 30.f}) {
        for (const FloatingPoint sign : {-1.f, 1.f}) {
          const Vector<kDim> translation =
              sign * scale * Vector<kDim>{direction & 0b01, direction & 0b10};
          for (const auto& aabb : aabbs) {
            const AABB<Point<kDim>> aabb_translated{aabb.min + translation,
                                                    aabb.max + translation};
            tests.emplace_back(aabb, translation);
            tests.emplace_back(aabb_translated, Point<kDim>::Zero());
            tests.emplace_back(aabb_translated, translation);
          }
        }
      }
    }
  }

  // Run tests
  for (const QueryAndExpectedResults& test : tests) {
    // Find the closest and furthest point
    Point<kDim> closest_point;
    Point<kDim> furthest_point;
    // Check for closest/furthest points on the AABB's edges
    for (int dim_idx = 0; dim_idx < 2; ++dim_idx) {
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
      EXPECT_TRUE(test.aabb.containsPoint(test.query_point))
          << test.getDescription();
    }

    const Point<kDim> returned_closest_point =
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

    const Point<kDim> returned_furthest_point =
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

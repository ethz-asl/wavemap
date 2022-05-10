#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/generic/aabb.h"
#include "wavemap_2d/test/fixture_base.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
using AabbTest = FixtureBase;

TEST_F(AabbTest, InitializationAndInclusion) {
  for (const Point& random_point : getRandomPointVector()) {
    AABB<Point> aabb;
    EXPECT_FALSE(aabb.containsPoint(random_point))
        << "The uninitialized AABB should be empty.";
    aabb.includePoint(random_point);
    EXPECT_TRUE(aabb.containsPoint(random_point))
        << "The AABB should contain points after they have been included.";
  }
}

TEST_F(AabbTest, ClosestPointsAndDistances) {
  struct QueryAndExpectedResults {
    Point query_point;
    Point closest_point;
    Point furthest_point;

    QueryAndExpectedResults(Point _query_point, Point _closest_point,
                            Point _furthest_point)
        : query_point(std::move(_query_point)),
          closest_point(std::move(_closest_point)),
          furthest_point(std::move(_furthest_point)) {}

    std::string getDescription() const {
      std::stringstream ss;
      ss << "For query_point " << EigenFormat::oneLine(query_point)
         << ", closest_point " << EigenFormat::oneLine(closest_point)
         << "and furthest_point " << EigenFormat::oneLine(furthest_point)
         << ".";
      return ss.str();
    }
  };

  AABB<Point> unit_aabb{Point::Zero(), Point::Ones()};
  std::vector<QueryAndExpectedResults> tests;
  tests.emplace_back(Point::Zero(), Point::Zero(), Point::Ones());
  tests.emplace_back(Point::Ones(), Point::Ones(), Point::Zero());
  tests.emplace_back(Point::Constant(0.4f), Point::Constant(0.4f),
                     Point::Ones());
  tests.emplace_back(Point::Constant(0.6f), Point::Constant(0.6f),
                     Point::Zero());
  tests.emplace_back(-Point::Ones(), Point::Zero(), Point::Ones());
  tests.emplace_back(Point::Constant(2.f), Point::Ones(), Point::Zero());
  tests.emplace_back(Point::UnitX(), Point::UnitX(), Point::UnitY());
  tests.emplace_back(0.4f * Point::UnitX(), 0.4f * Point::UnitX(),
                     Point::Ones());
  tests.emplace_back(0.6f * Point::UnitX(), 0.6f * Point::UnitX(),
                     Point::UnitY());
  tests.emplace_back(2.f * Point::UnitX(), Point::UnitX(), Point::UnitY());
  tests.emplace_back(Point{2.f, 0.4f}, Point{1.f, 0.4f}, Point::UnitY());
  tests.emplace_back(Point{2.f, 0.6f}, Point{1.f, 0.6f}, Point::Zero());
  tests.emplace_back(Point::UnitY(), Point::UnitY(), Point::UnitX());
  tests.emplace_back(0.4f * Point::UnitY(), 0.4f * Point::UnitY(),
                     Point::Ones());
  tests.emplace_back(0.6f * Point::UnitY(), 0.6f * Point::UnitY(),
                     Point::UnitX());
  tests.emplace_back(2.f * Point::UnitY(), Point::UnitY(), Point::UnitX());
  tests.emplace_back(Point{0.4f, 2.f}, Point{0.4f, 1.f}, Point::UnitX());
  tests.emplace_back(Point{0.6f, 2.f}, Point{0.6f, 1.f}, Point::Zero());

  for (const QueryAndExpectedResults& test : tests) {
    if ((test.query_point - test.closest_point).norm() <= 0) {
      EXPECT_TRUE(unit_aabb.containsPoint(test.query_point))
          << test.getDescription();
    }

    const Point returned_closest_point =
        unit_aabb.closestPointTo(test.query_point);
    EXPECT_NEAR(returned_closest_point.x(), test.closest_point.x(), kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(returned_closest_point.y(), test.closest_point.y(), kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(unit_aabb.minDistanceTo(test.query_point),
                (test.query_point - test.closest_point).norm(), kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(unit_aabb.minSquaredDistanceTo(test.query_point),
                (test.query_point - test.closest_point).squaredNorm(), kEpsilon)
        << test.getDescription();

    const Point returned_furthest_point =
        unit_aabb.furthestPointFrom(test.query_point);
    EXPECT_NEAR(returned_furthest_point.x(), test.furthest_point.x(), kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(returned_furthest_point.y(), test.furthest_point.y(), kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(unit_aabb.maxDistanceTo(test.query_point),
                (test.query_point - test.furthest_point).norm(), kEpsilon)
        << test.getDescription();
    EXPECT_NEAR(unit_aabb.maxSquaredDistanceTo(test.query_point),
                (test.query_point - test.furthest_point).squaredNorm(),
                kEpsilon)
        << test.getDescription();
  }
}
}  // namespace wavemap_2d

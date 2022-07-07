#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/test/fixture_base.h>
#include <wavemap_common/utils/eigen_format.h>

#include "wavemap_2d/iterator/ray_iterator.h"

namespace wavemap {
using RayIteratorTest = FixtureBase;

TEST_F(RayIteratorTest, IterationOrderAndCompleteness) {
  constexpr int kNumTestRays = 100;
  struct TestRay {
    Point2D origin;
    Vector2D translation;
  };

  // Create zero length, perfectly horizontal/vertical, and random test rays
  std::vector<TestRay> test_rays(kNumTestRays);
  test_rays[0] = {getRandomPoint<2>(), {0.f, 0.f}};
  test_rays[1] = {getRandomPoint<2>(), {getRandomSignedDistance(), 0.f}};
  test_rays[2] = {getRandomPoint<2>(), {0.f, getRandomSignedDistance()}};
  std::generate(test_rays.begin() + 3, test_rays.end(), [this]() {
    return TestRay{getRandomPoint<2>(), getRandomTranslation<2>()};
  });

  for (const auto& test_ray : test_rays) {
    const Point2D start_point = test_ray.origin;
    const Point2D end_point = test_ray.origin + test_ray.translation;
    const Vector2D t_start_end = end_point - start_point;
    const FloatingPoint ray_length = t_start_end.norm();

    const FloatingPoint min_cell_width = getRandomMinCellWidth();
    const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
    const Index2D start_point_index =
        convert::pointToNearestIndex(start_point, min_cell_width_inv);
    const Index2D end_point_index =
        convert::pointToNearestIndex(end_point, min_cell_width_inv);
    const Index2D direction =
        (end_point_index - start_point_index).cwiseSign().cast<IndexElement>();

    Ray ray(start_point, end_point, min_cell_width);
    EXPECT_EQ(*ray.begin(), start_point_index);
    EXPECT_EQ(*ray.end(), end_point_index);

    Index2D last_index;
    size_t step_idx = 0u;
    for (const Index2D& index : ray) {
      if (step_idx == 0u) {
        EXPECT_EQ(index, start_point_index)
            << "Ray iterator did not start at start index.";
      } else {
        EXPECT_NE(index, last_index)
            << "Ray iterator updated the same index twice.";
        const Index2D index_diff = index - last_index;
        EXPECT_EQ(index_diff.cwiseAbs().sum(), 1)
            << "Ray iterator skipped a cell.";
        EXPECT_TRUE((index_diff.array() == direction.array()).any())
            << "Ray iterator stepped into an unexpected direction.";

        const Point2D current_point =
            convert::indexToCenterPoint(index, min_cell_width);
        const Vector2D t_start_current = current_point - start_point;
        const FloatingPoint distance =
            std::abs(t_start_end.x() * t_start_current.y() -
                     t_start_current.x() * t_start_end.y()) /
            ray_length;
        constexpr FloatingPoint kMaxAcceptableMultiplicativeError = 1.f + 1e-4f;
        EXPECT_LE(distance, kMaxAcceptableMultiplicativeError *
                                std::sqrt(0.5f) * min_cell_width)
            << "Ray iterator updated cell that is not traversed by the ray, "
               "for cell with index"
            << EigenFormat::oneLine(index) << " and center point"
            << EigenFormat::oneLine(current_point) << " on ray from"
            << EigenFormat::oneLine(start_point) << " to"
            << EigenFormat::oneLine(end_point) << " at min cell width "
            << min_cell_width << ".";
      }
      last_index = index;
      ++step_idx;
    }
    EXPECT_TRUE(last_index == end_point_index);
    EXPECT_GE(step_idx, 1)
        << "Ray iterator should at least take one step, to update the cell "
           "that contains the start (and end) point.";
  }
}
}  // namespace wavemap

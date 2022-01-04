#include <gtest/gtest.h>

#include "wavemap_2d/integrator/grid_iterator.h"
#include "wavemap_2d/integrator/ray_iterator.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
using IteratorTest = FixtureBase;

TEST_F(IteratorTest, GridIterator) {
  constexpr int kNumTestGrids = 20;
  for (int i = 0; i < kNumTestGrids; ++i) {
    const Index bottom_left_idx = -getRandomIndex().cwiseAbs();
    const Index top_right_idx = getRandomIndex().cwiseAbs();

    Grid grid(bottom_left_idx, top_right_idx);
    Grid::Iterator grid_it = grid.begin();
    const Grid::Iterator grid_it_end = grid.end();

    size_t count = 0u;
    for (Index index = bottom_left_idx; index.x() <= top_right_idx.x();
         ++index.x()) {
      for (index.y() = bottom_left_idx.y(); index.y() <= top_right_idx.y();
           ++index.y()) {
        EXPECT_NE(grid_it, grid_it_end);
        EXPECT_EQ(*grid_it, index);
        ++grid_it;
        ++count;
      }
    }
    const size_t num_grid_indices =
        (top_right_idx - bottom_left_idx + Index::Ones()).array().prod();
    EXPECT_EQ(count, num_grid_indices);
    EXPECT_EQ(grid_it, grid_it_end);
  }
}

TEST_F(IteratorTest, RayIterator) {
  constexpr int kNumTestRays = 100;
  struct TestRay {
    Point origin;
    Vector translation;
  };

  // Create zero length, perfectly horizontal/vertical, and random test rays
  std::vector<TestRay> test_rays(kNumTestRays);
  test_rays[0] = {getRandomPoint(), {0.f, 0.f}};
  test_rays[1] = {getRandomPoint(), {getRandomSignedDistance(), 0.f}};
  test_rays[2] = {getRandomPoint(), {0.f, getRandomSignedDistance()}};
  std::generate(test_rays.begin() + 3, test_rays.end(), [this]() {
    return TestRay{getRandomPoint(), getRandomTranslation()};
  });

  for (const auto& test_ray : test_rays) {
    const Point start_point = test_ray.origin;
    const Point end_point = test_ray.origin + test_ray.translation;
    const Vector t_start_end = end_point - start_point;
    const FloatingPoint ray_length = t_start_end.norm();

    const FloatingPoint resolution = getRandomResolution();
    const FloatingPoint resolution_inv = 1.f / resolution;
    const Index start_point_index =
        (start_point * resolution_inv).array().round().cast<IndexElement>();
    const Index end_point_index =
        (end_point * resolution_inv).array().round().cast<IndexElement>();
    const Index direction =
        (end_point_index - start_point_index).cwiseSign().cast<IndexElement>();

    Ray ray(start_point, end_point, resolution);
    EXPECT_EQ(*ray.begin(), start_point_index);
    EXPECT_EQ(*ray.end(), end_point_index);

    Index last_index;
    size_t step_idx = 0u;
    for (const Index& index : ray) {
      if (step_idx == 0u) {
        EXPECT_EQ(index, start_point_index)
            << "Ray iterator did not start at start index.";
      } else {
        EXPECT_NE(index, last_index)
            << "Ray iterator updated the same index twice.";
        const Index index_diff = index - last_index;
        EXPECT_EQ(index_diff.cwiseAbs().sum(), 1)
            << "Ray iterator skipped a cell.";
        EXPECT_TRUE((index_diff.array() == direction.array()).any())
            << "Ray iterator stepped into an unexpected direction.";

        const Point current_point = resolution * index.cast<FloatingPoint>();
        const Vector t_start_current = current_point - start_point;
        const FloatingPoint distance =
            std::abs(t_start_end.x() * t_start_current.y() -
                     t_start_current.x() * t_start_end.y()) /
            ray_length;
        EXPECT_LE(distance, std::sqrt(0.5f) * resolution)
            << "Ray iterator updated cell that is not traversed by the ray";
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
}  // namespace wavemap_2d

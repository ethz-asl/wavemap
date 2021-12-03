#include <gtest/gtest.h>

#include "wavemap_2d/integrator/beam_model.h"
#include "wavemap_2d/integrator/grid_iterator.h"
#include "wavemap_2d/integrator/ray_iterator.h"
#include "wavemap_2d/utils/random_number_generator.h"

namespace wavemap_2d {
class IteratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    random_number_generator_ = std::make_unique<RandomNumberGenerator>();
  }

  FloatingPoint getRandomResolution() const {
    constexpr FloatingPoint kMinResolution = 1e-3;
    constexpr FloatingPoint kMaxResolution = 1e0;
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
  static Point getRandomPoint() {
    constexpr FloatingPoint kMaxCoordinate = 1e3;
    return kMaxCoordinate * Point::Random();
  }
  FloatingPoint getRandomSignedDistance() {
    constexpr FloatingPoint kMinDistance = -4e1;
    constexpr FloatingPoint kMaxDistance = 4e1;
    return random_number_generator_->getRandomRealNumber(kMinDistance,
                                                         kMaxDistance);
  }
  Translation getRandomTranslation() {
    return {getRandomSignedDistance(), getRandomSignedDistance()};
  }

 private:
  std::unique_ptr<RandomNumberGenerator> random_number_generator_;
};

TEST_F(IteratorTest, GridIterator) {
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

TEST_F(IteratorTest, RayIterator) {
  // Create zero length, perfectly horizontal/vertical, and random test rays
  constexpr int kNumTestRays = 10;
  struct TestRay {
    Point origin;
    Translation translation;
  };
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
    const Translation t_start_end = end_point - start_point;
    const FloatingPoint ray_length = t_start_end.norm();

    const FloatingPoint resolution = getRandomResolution();
    const FloatingPoint resolution_inv = 1.f / resolution;
    const Index start_point_index =
        (start_point * resolution_inv).array().round().cast<IndexElement>();
    const Index end_point_index =
        (end_point * resolution_inv).array().round().cast<IndexElement>();
    const Index direction =
        (end_point_index - start_point_index).cwiseSign().cast<IndexElement>();

    Ray ray(start_point, end_point, resolution_inv);
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
        const Translation t_start_current = current_point - start_point;
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

class MeasurementModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    random_number_generator_ = std::make_unique<RandomNumberGenerator>();
  }

  FloatingPoint getRandomResolution() const {
    constexpr FloatingPoint kMinResolution = 1e-3;
    constexpr FloatingPoint kMaxResolution = 2e-1;
    return random_number_generator_->getRandomRealNumber(kMinResolution,
                                                         kMaxResolution);
  }
  static Point getRandomPoint() {
    constexpr FloatingPoint kMaxCoordinate = 1e3;
    return kMaxCoordinate * Point::Random();
  }
  FloatingPoint getRandomSignedDistance() {
    constexpr FloatingPoint kMinDistance = -4e1;
    constexpr FloatingPoint kMaxDistance = 4e1;
    return random_number_generator_->getRandomRealNumber(kMinDistance,
                                                         kMaxDistance);
  }
  Translation getRandomTranslation() {
    return {getRandomSignedDistance(), getRandomSignedDistance()};
  }

 private:
  std::unique_ptr<RandomNumberGenerator> random_number_generator_;
};

TEST_F(MeasurementModelTest, BeamModel) {
  const FloatingPoint resolution = getRandomResolution();
  BeamModel beam_model(resolution);

  const Point W_start_point = getRandomPoint();
  const Point W_end_point = W_start_point + getRandomTranslation();
  beam_model.setStartPoint(W_start_point);
  beam_model.setEndPoint(W_end_point);

  const Point C_end_point = W_end_point - W_start_point;
  const FloatingPoint measured_distance = C_end_point.norm();
  const Point C_end_point_normalized = C_end_point / measured_distance;

  const Index bottom_left_idx = beam_model.getBottomLeftUpdateIndex();
  const Index top_right_idx = beam_model.getTopRightUpdateIndex();

  constexpr IndexElement kIndexPadding = 10;
  const Index padded_bottom_left_idx =
      bottom_left_idx - Index::Constant(kIndexPadding);
  const Index padded_top_right_idx =
      top_right_idx + Index::Constant(kIndexPadding);

  const Grid grid(padded_bottom_left_idx, padded_top_right_idx);
  for (const auto& index : grid) {
    const FloatingPoint update = beam_model.computeUpdateAt(index);

    if ((index.array() < bottom_left_idx.array()).any() ||
        (top_right_idx.array() < index.array()).any()) {
      EXPECT_FLOAT_EQ(update, 0.f)
          << "Update queries outside the bounding box should return 0.";
    }

    const Point W_cell_center = index.cast<FloatingPoint>() * resolution;
    const Point C_cell_center = W_cell_center - W_start_point;
    const FloatingPoint distance = C_cell_center.norm();
    const FloatingPoint dot_prod_normalized =
        C_cell_center.dot(C_end_point_normalized) / distance;
    const FloatingPoint angle = std::acos(dot_prod_normalized);

    const bool within_range =
        distance <= measured_distance + BeamModel::kRangeDeltaThresh;
    const bool within_angle = angle <= BeamModel::kAngleThresh;
    if (within_range && within_angle) {
      if (distance < measured_distance) {
        EXPECT_LE(update, 0.f)
            << "Logodds occupancy updates in front of the sensor should be "
               "non-positive (free or unknown space)";
      } else {
        EXPECT_GE(update, 0.f)
            << "Logodds occupancy updates in front of the sensor should be "
               "non-negative (occupied or unknown space)";
      }
    } else {
      EXPECT_FLOAT_EQ(update, 0.f)
          << "Encountered non-zero update for cell that is not within the "
             "beam's "
          << (!within_range ? "range" : "")
          << (!within_range && !within_angle ? " and " : "")
          << (!within_angle ? "angle" : "");
    }
  }
}
}  // namespace wavemap_2d

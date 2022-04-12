#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/integrator/measurement_model/beam_model.h"
#include "wavemap_2d/integrator/measurement_model/fixed_logodds_model.h"
#include "wavemap_2d/iterator/grid_iterator.h"
#include "wavemap_2d/iterator/ray_iterator.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
using MeasurementModelTest = FixtureBase;

TEST_F(MeasurementModelTest, BeamModel) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
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

      const Point W_cell_center = computeCenterFromIndex(index, resolution);
      const Point C_cell_center = W_cell_center - W_start_point;
      const FloatingPoint distance = C_cell_center.norm();
      const FloatingPoint dot_prod_normalized =
          C_cell_center.dot(C_end_point_normalized) / distance;
      const FloatingPoint angle =
          std::acos(std::clamp(dot_prod_normalized, 0.f, 1.f));

      const FloatingPoint non_zero_range =
          measured_distance + BeamModel::kRangeDeltaThresh;
      const bool within_range = distance <= non_zero_range;
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
            << (!within_range ? "range (" + std::to_string(distance) + " !<= " +
                                    std::to_string(non_zero_range) + ") "
                              : "")
            << (!within_range && !within_angle ? " and " : "")
            << (!within_angle
                    ? "angle (" + std::to_string(angle) + " !<= " +
                          std::to_string(BeamModel::kAngleThresh) + ") "
                    : "");
      }
    }
  }
}

TEST_F(MeasurementModelTest, FixedLogOddsModel) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    const Point start_point = getRandomPoint();
    const Point end_point = start_point + getRandomTranslation();
    const FloatingPoint resolution = getRandomResolution();
    const FloatingPoint resolution_inv = 1.f / resolution;

    FixedLogOddsModel fixed_log_odds_model(resolution);
    fixed_log_odds_model.setStartPoint(start_point);
    fixed_log_odds_model.setEndPoint(end_point);
    const Index start_point_index =
        computeNearestIndexForPoint(start_point, resolution_inv);
    const Index end_point_index =
        computeNearestIndexForPoint(end_point, resolution_inv);

    const Index bottom_left_idx = start_point_index.cwiseMin(end_point_index);
    const Index top_right_idx = start_point_index.cwiseMin(end_point_index);
    const Index model_bottom_left_idx =
        fixed_log_odds_model.getBottomLeftUpdateIndex();
    const Index model_top_right_idx =
        fixed_log_odds_model.getTopRightUpdateIndex();
    EXPECT_TRUE(
        (model_bottom_left_idx.array() <= bottom_left_idx.array()).all());
    EXPECT_TRUE((top_right_idx.array() <= model_top_right_idx.array()).all());

    size_t step_idx = 0u;
    bool updated_end_point = false;
    const Ray ray(start_point, end_point, resolution);
    for (const auto& index : ray) {
      const FloatingPoint update = fixed_log_odds_model.computeUpdateAt(index);
      if (index == end_point_index) {
        EXPECT_FLOAT_EQ(update, FixedLogOddsModel::kLogOddsOccupied);
        EXPECT_FALSE(updated_end_point)
            << "The cell containing the end point should only be updated once.";
        updated_end_point = true;
      } else {
        EXPECT_FLOAT_EQ(update, FixedLogOddsModel::kLogOddsFree);
      }
      ++step_idx;
    }
    EXPECT_GE(step_idx, 1);
    EXPECT_TRUE(updated_end_point);
  }
}
}  // namespace wavemap_2d

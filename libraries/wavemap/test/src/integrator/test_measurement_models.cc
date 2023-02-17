#include <gtest/gtest.h>
#include <wavemap/common.h>
#include <wavemap/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap/integrator/measurement_model/range_only/constant_1d_log_odds.h>
#include <wavemap/iterator/grid_iterator.h>
#include <wavemap/iterator/ray_iterator.h>
#include <wavemap/test/fixture_base.h>

namespace wavemap {
using MeasurementModelTest = FixtureBase;

TEST_F(MeasurementModelTest, ContinuousVolumetricLogOddsModel) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    const FloatingPoint min_cell_width = getRandomMinCellWidth();
    const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
    ContinuousVolumetricLogOddsConfig config;
    config.angle_sigma =
        random_number_generator_->getRandomRealNumber(1e-3f, 5e-2f);
    config.range_sigma =
        random_number_generator_->getRandomRealNumber(1e-3f, 5e-2f);
    const ContinuousVolumetricLogOdds<2> model(config);

    const Point2D W_start_point = getRandomPoint<2>();
    const Point2D W_end_point = W_start_point + getRandomTranslation<2>();

    const Point2D W_t_end_point_start_point = W_end_point - W_start_point;
    const FloatingPoint beam_length = W_t_end_point_start_point.norm();
    const FloatingPoint beam_angle = std::atan2(W_t_end_point_start_point.y(),
                                                W_t_end_point_start_point.x());

    const Index2D bottom_left_idx = model.getBottomLeftUpdateIndex(
        W_start_point, W_end_point, beam_length, min_cell_width_inv);
    const Index2D top_right_idx = model.getTopRightUpdateIndex(
        W_start_point, W_end_point, beam_length, min_cell_width_inv);

    constexpr IndexElement kIndexPadding = 10;
    const Index2D padded_bottom_left_idx =
        bottom_left_idx - Index2D::Constant(kIndexPadding);
    const Index2D padded_top_right_idx =
        top_right_idx + Index2D::Constant(kIndexPadding);

    const Grid grid(padded_bottom_left_idx, padded_top_right_idx);
    for (const auto& index : grid) {
      const Point2D W_cell_center =
          convert::indexToCenterPoint(index, min_cell_width);
      const Point2D W_t_cell_center_end_point = W_cell_center - W_start_point;
      const FloatingPoint cell_distance = W_t_cell_center_end_point.norm();

      const FloatingPoint cell_angle = std::atan2(
          W_t_cell_center_end_point.y(), W_t_cell_center_end_point.x());
      const FloatingPoint cell_to_beam_angle =
          std::abs(cell_angle - beam_angle);

      const FloatingPoint update =
          model.computeUpdate(cell_distance, cell_to_beam_angle, beam_length);

      if ((index.array() < bottom_left_idx.array()).any() ||
          (top_right_idx.array() < index.array()).any()) {
        EXPECT_FLOAT_EQ(update, 0.f)
            << "Update queries outside the bounding box should return 0. "
               "At current index "
            << EigenFormat::oneLine(index) << ", bottom_left_idx "
            << EigenFormat::oneLine(bottom_left_idx) << ", top_right_index"
            << EigenFormat::oneLine(top_right_idx) << ", cell_distance "
            << cell_distance << ", cell_to_beam_angle " << cell_to_beam_angle
            << ", beam_angle " << beam_angle << ", cell_angle " << cell_angle
            << " and beam_length " << beam_length;
      }

      const FloatingPoint non_zero_range =
          beam_length + model.getRangeThresholdBehindSurface();
      const bool within_range = cell_distance <= non_zero_range;
      const bool within_angle = cell_to_beam_angle <= model.getAngleThreshold();
      if (within_range && within_angle) {
        if (cell_distance < beam_length) {
          EXPECT_LE(update, kEpsilon)
              << "Logodds occupancy updates in front of the sensor should be "
                 "non-positive (free or unknown space)";
        } else {
          EXPECT_GE(update, -kEpsilon)
              << "Logodds occupancy updates in front of the sensor should be "
                 "non-negative (occupied or unknown space)";
        }
      } else {
        EXPECT_NEAR(update, 0.f, kEpsilon)
            << "Encountered non-zero update for cell that is not within the "
               "beam's "
            << (!within_range
                    ? "range (" + std::to_string(cell_distance) +
                          " !<= " + std::to_string(non_zero_range) + ") "
                    : "")
            << (!within_range && !within_angle ? " and " : "")
            << (!within_angle
                    ? "angle (" + std::to_string(cell_to_beam_angle) + " !<= " +
                          std::to_string(model.getAngleThreshold()) + ") "
                    : "");
      }

      if (cell_distance <
          beam_length - model.getRangeThresholdInFrontOfSurface()) {
        EXPECT_NEAR(model.computeFreeSpaceUpdate(cell_to_beam_angle), update,
                    kEpsilon)
            << "In free space (update <= 0), the regular update computation "
               "computeUpdate(...) and computeFreeSpaceUpdate(...) should "
               "yield the same results.";
      }
    }
  }
}

TEST_F(MeasurementModelTest, Constant1DLogOddsModel) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
    const Point2D start_point = getRandomPoint<2>();
    const Point2D end_point = start_point + getRandomTranslation<2>();
    const FloatingPoint min_cell_width = getRandomMinCellWidth();
    const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;

    Constant1DLogOdds<2> model(min_cell_width);
    model.setStartPoint(start_point);
    model.setEndPoint(end_point);
    const Index2D start_point_index =
        convert::pointToNearestIndex(start_point, min_cell_width_inv);
    const Index2D end_point_index =
        convert::pointToNearestIndex(end_point, min_cell_width_inv);

    const Index2D bottom_left_idx = start_point_index.cwiseMin(end_point_index);
    const Index2D top_right_idx = start_point_index.cwiseMin(end_point_index);
    const Index2D model_bottom_left_idx = model.getBottomLeftUpdateIndex();
    const Index2D model_top_right_idx = model.getTopRightUpdateIndex();
    EXPECT_TRUE(
        (model_bottom_left_idx.array() <= bottom_left_idx.array()).all());
    EXPECT_TRUE((top_right_idx.array() <= model_top_right_idx.array()).all());

    size_t step_idx = 0u;
    bool updated_end_point = false;
    const Ray ray(start_point, end_point, min_cell_width);
    for (const auto& index : ray) {
      const FloatingPoint update = model.computeUpdate(index);
      if (index == end_point_index) {
        EXPECT_FLOAT_EQ(update, Constant1DLogOdds<2>::kLogOddsOccupied);
        EXPECT_FALSE(updated_end_point)
            << "The cell containing the end point should only be updated once.";
        updated_end_point = true;
      } else {
        EXPECT_FLOAT_EQ(update, Constant1DLogOdds<2>::kLogOddsFree);
      }
      ++step_idx;
    }
    EXPECT_GE(step_idx, 1);
    EXPECT_TRUE(updated_end_point);
  }
}
}  // namespace wavemap

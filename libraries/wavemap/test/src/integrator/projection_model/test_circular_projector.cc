#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/integrator/projection_model/circular_projector.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
class CircularProjectorTest : public FixtureBase, public GeometryGenerator {};

TEST_F(CircularProjectorTest, InitializationAndAccessors) {
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const FloatingPoint min_angle = getRandomAngle(-kPi, kHalfPi);
    const FloatingPoint max_angle = getRandomAngle(min_angle + kEpsilon, kPi);
    const IndexElement num_cells = getRandomIndexElement(2, 1000);
    CircularProjector circular_projector(
        CircularProjectorConfig{min_angle, max_angle, num_cells});
    EXPECT_EQ(circular_projector.getMinAngle(), min_angle);
    EXPECT_EQ(circular_projector.getMaxAngle(), max_angle);
    EXPECT_EQ(circular_projector.getNumCells(), num_cells);
  }
}

TEST_F(CircularProjectorTest, Conversions) {
  for (int repetition = 0; repetition < 1000; ++repetition) {
    // Test bearing <-> angle
    {
      const Point2D point_original = getRandomPoint<2>();
      const FloatingPoint range = point_original.norm();
      const FloatingPoint angle =
          CircularProjector::bearingToAngle(point_original);
      const Point2D point_roundtrip =
          range * CircularProjector::angleToBearing(angle);
      EXPECT_NEAR(point_roundtrip.x(), point_original.x(), 1e-6 * range);
      EXPECT_NEAR(point_roundtrip.y(), point_original.y(), 1e-6 * range);
    }
    {
      const FloatingPoint angle_original = getRandomAngle();
      const Vector2D bearing =
          CircularProjector::angleToBearing(angle_original);
      const FloatingPoint angle_roundtrip =
          CircularProjector::bearingToAngle(bearing);
      EXPECT_FLOAT_EQ(angle_roundtrip, angle_original);
    }

    // Instantiate projection model with random params
    constexpr FloatingPoint kMinAngleIntervalWidth = kPi / 16.f;
    const FloatingPoint min_angle = getRandomAngle(-kPi, kHalfPi);
    const FloatingPoint max_angle =
        getRandomAngle(min_angle + kMinAngleIntervalWidth, kPi);
    const IndexElement num_beams = getRandomIndexElement(2, 2048);
    CircularProjector circular_projector(
        CircularProjectorConfig{min_angle, max_angle, num_beams});

    // Precompute commonly used values
    const auto max_index = static_cast<IndexElement>((num_beams - 1));
    const FloatingPoint mid_angle = min_angle + (max_angle - min_angle) / 2.f;
    const IndexElement mid_index_floor = max_index / 2;
    // If the max_index is uneven, the ceil of the middle index is rounded up
    const IndexElement mid_index_ceil = mid_index_floor + (max_index & 0b1);

    // Test angle <-> index
    {
      std::string error_msg = "For min_angle " + std::to_string(min_angle) +
                              ", max_angle " + std::to_string(max_angle) +
                              " and num_beams " + std::to_string(num_beams);
      EXPECT_EQ(circular_projector.angleToNearestIndex(min_angle), 0)
          << error_msg;
      EXPECT_EQ(circular_projector.angleToFloorIndex(min_angle + kEpsilon), 0)
          << error_msg;
      EXPECT_FLOAT_EQ(circular_projector.indexToAngle(0), min_angle)
          << error_msg;

      EXPECT_EQ(circular_projector.angleToNearestIndex(max_angle), max_index)
          << error_msg;
      EXPECT_EQ(circular_projector.angleToCeilIndex(max_angle - kEpsilon),
                max_index)
          << error_msg;
      EXPECT_NEAR(circular_projector.indexToAngle(max_index), max_angle, 1e-6)
          << error_msg;

      EXPECT_EQ(circular_projector.angleToFloorIndex(mid_angle + kEpsilon),
                mid_index_floor)
          << error_msg;
      EXPECT_EQ(circular_projector.angleToCeilIndex(mid_angle - kEpsilon),
                mid_index_ceil)
          << error_msg;
    }

    // Test bearing <-> index
    {
      const Vector2D min_bearing = CircularProjector::angleToBearing(min_angle);
      EXPECT_EQ(circular_projector.bearingToNearestIndex(min_bearing), 0);
      const Vector2D bearing_min_index = circular_projector.indexToBearing(0);
      EXPECT_NEAR(bearing_min_index.x(), min_bearing.x(), 1e-6);
      EXPECT_NEAR(bearing_min_index.y(), min_bearing.y(), 1e-6);

      const Vector2D max_bearing = CircularProjector::angleToBearing(max_angle);
      EXPECT_EQ(circular_projector.bearingToNearestIndex(max_bearing),
                max_index);
      const Vector2D bearing_max_index =
          circular_projector.indexToBearing(max_index);
      EXPECT_NEAR(bearing_max_index.x(), max_bearing.x(), 1e-6);
      EXPECT_NEAR(bearing_max_index.y(), max_bearing.y(), 1e-6);

      const Vector2D mid_bearing = CircularProjector::angleToBearing(mid_angle);
      EXPECT_GE(circular_projector.bearingToNearestIndex(mid_bearing),
                mid_index_floor);
      EXPECT_LE(circular_projector.bearingToNearestIndex(mid_bearing),
                mid_index_ceil);
    }
  }
}
}  // namespace wavemap

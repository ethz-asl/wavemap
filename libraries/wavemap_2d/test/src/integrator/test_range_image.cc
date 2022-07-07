#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/test/fixture_base.h>

#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap {
using RangeImageTest = FixtureBase;

TEST_F(RangeImageTest, ConstructorAndAccessors) {
  for (int idx = 0; idx < 10; ++idx) {
    const FloatingPoint min_angle = getRandomAngle(-kPi, kHalfPi);
    const FloatingPoint max_angle = getRandomAngle(min_angle + kEpsilon, kPi);
    const Eigen::Index num_beams = getRandomIndexElement(1, 2048);
    RangeImage range_image(min_angle, max_angle, num_beams);

    EXPECT_FALSE(range_image.empty());
    EXPECT_EQ(range_image.size(), num_beams);

    EXPECT_EQ(range_image.getMinAngle(), min_angle);
    EXPECT_EQ(range_image.getMaxAngle(), max_angle);
    EXPECT_EQ(range_image.getNumBeams(), num_beams);

    range_image.clear();
    EXPECT_TRUE(range_image.empty());
    EXPECT_EQ(range_image.size(), 0);
    EXPECT_EQ(range_image.getNumBeams(), 0);

    const Eigen::Index new_num_beams = getRandomIndexElement(1, 1024);
    range_image.resize(new_num_beams);
    EXPECT_EQ(range_image.size(), new_num_beams);
    EXPECT_EQ(range_image.getNumBeams(), new_num_beams);
  }
}

TEST_F(RangeImageTest, IndexConversions) {
  for (int idx = 0; idx < 1000; ++idx) {
    // Test bearing <-> angle
    {
      const Point2D point_original = getRandomPoint<2>();
      const FloatingPoint range = point_original.norm();
      const FloatingPoint angle = RangeImage::bearingToAngle(point_original);
      const Point2D point_roundtrip = range * RangeImage::angleToBearing(angle);
      EXPECT_NEAR(point_roundtrip.x(), point_original.x(), 1e-6 * range);
      EXPECT_NEAR(point_roundtrip.y(), point_original.y(), 1e-6 * range);

      const FloatingPoint angle_original = getRandomAngle();
      const Vector2D bearing = RangeImage::angleToBearing(angle_original);
      const FloatingPoint angle_roundtrip = RangeImage::bearingToAngle(bearing);
      EXPECT_FLOAT_EQ(angle_roundtrip, angle_original);
    }

    // Construct a random range image
    const FloatingPoint min_angle = getRandomAngle(-kPi, kHalfPi);
    const FloatingPoint max_angle = getRandomAngle(min_angle + kEpsilon, kPi);
    const Eigen::Index num_beams = getRandomIndexElement(2, 2048);
    RangeImage range_image(min_angle, max_angle, num_beams);

    // Precompute commonly used values
    const auto max_index = static_cast<RangeImageIndex>((num_beams - 1));
    const FloatingPoint mid_angle = min_angle + (max_angle - min_angle) / 2.f;
    const RangeImageIndex mid_index_floor = max_index / 2;
    // If the max_index is uneven, the ceil of the middle index is rounded up
    const RangeImageIndex mid_index_ceil = mid_index_floor + (max_index & 0b1);

    // Test angle <-> index
    {
      EXPECT_EQ(range_image.angleToNearestIndex(min_angle), 0);
      EXPECT_EQ(range_image.angleToFloorIndex(min_angle + kEpsilon), 0);
      EXPECT_FLOAT_EQ(range_image.indexToAngle(0), min_angle);

      EXPECT_EQ(range_image.angleToNearestIndex(max_angle), max_index);
      EXPECT_EQ(range_image.angleToCeilIndex(max_angle - kEpsilon), max_index);
      EXPECT_NEAR(range_image.indexToAngle(max_index), max_angle, 1e-6);

      EXPECT_EQ(range_image.angleToFloorIndex(mid_angle + kEpsilon),
                mid_index_floor);
      EXPECT_EQ(range_image.angleToCeilIndex(mid_angle - kEpsilon),
                mid_index_ceil);
    }

    // Test bearing <-> index
    {
      const Vector2D min_bearing = RangeImage::angleToBearing(min_angle);
      EXPECT_EQ(range_image.bearingToNearestIndex(min_bearing), 0);
      const Vector2D bearing_min_index = range_image.indexToBearing(0);
      EXPECT_NEAR(bearing_min_index.x(), min_bearing.x(), 1e-6);
      EXPECT_NEAR(bearing_min_index.y(), min_bearing.y(), 1e-6);

      const Vector2D max_bearing = RangeImage::angleToBearing(max_angle);
      EXPECT_EQ(range_image.bearingToNearestIndex(max_bearing), max_index);
      const Vector2D bearing_max_index = range_image.indexToBearing(max_index);
      EXPECT_NEAR(bearing_max_index.x(), max_bearing.x(), 1e-6);
      EXPECT_NEAR(bearing_max_index.y(), max_bearing.y(), 1e-6);

      const Vector2D mid_bearing = RangeImage::angleToBearing(mid_angle);
      EXPECT_GE(range_image.bearingToNearestIndex(mid_bearing),
                mid_index_floor);
      EXPECT_LE(range_image.bearingToNearestIndex(mid_bearing), mid_index_ceil);
    }
  }
}
}  // namespace wavemap

#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/integrator/projection_model/spherical_projector.h"
#include "wavemap/test/eigen_utils.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"
#include "wavemap/utils/print/eigen.h"

namespace wavemap {
class SphericalProjectorTest : public FixtureBase, public GeometryGenerator {
 protected:
  SphericalProjector getRandomProjectionModel() {
    const FloatingPoint min_elevation_angle = getRandomAngle(-kQuarterPi, 0.f);
    const FloatingPoint max_elevation_angle =
        getRandomAngle(min_elevation_angle + kPi / 8.f, kQuarterPi);
    const FloatingPoint min_azimuth_angle = -kPi;
    const FloatingPoint max_azimuth_angle = kPi;
    const int num_rows = int_math::exp2(getRandomIndexElement(4, 6));
    const int num_cols = int_math::exp2(getRandomIndexElement(7, 10));
    return SphericalProjector(SphericalProjectorConfig{
        {min_elevation_angle, max_elevation_angle, num_rows},
        {min_azimuth_angle, max_azimuth_angle, num_cols}});
  }
};

TEST_F(SphericalProjectorTest, InitializationAndAccessors) {
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const FloatingPoint min_elevation_angle =
        getRandomAngle(-kHalfPi, kQuarterPi);
    const FloatingPoint max_elevation_angle =
        getRandomAngle(min_elevation_angle + kEpsilon, kHalfPi);
    const IndexElement num_rows = getRandomIndexElement(2, 1000);

    const FloatingPoint min_azimuth_angle = getRandomAngle(-kPi, kHalfPi);
    const FloatingPoint max_azimuth_angle =
        getRandomAngle(min_azimuth_angle + kEpsilon, kPi);
    const IndexElement num_columns = getRandomIndexElement(2, 1000);

    const Index2D dimensions{num_rows, num_columns};

    SphericalProjector spherical_projector(
        {{min_elevation_angle, max_elevation_angle, num_rows},
         {min_azimuth_angle, max_azimuth_angle, num_columns}});
    EXPECT_EQ(spherical_projector.getNumRows(), num_rows);
    EXPECT_EQ(spherical_projector.getNumColumns(), num_columns);
    EXPECT_EQ(spherical_projector.getDimensions(), dimensions);
  }
}

TEST_F(SphericalProjectorTest, CellToBeamAngles) {
  constexpr FloatingPoint kNoiseTolerance = 1e-5f;
  constexpr FloatingPoint kMaxAcceptableAngleError = 2.f * kNoiseTolerance;
  const auto projector = getRandomProjectionModel();
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const Point3D C_point = getRandomPoint<3>();
    const auto sensor_coordinates = projector.cartesianToSensor(C_point);
    const auto [index, offset] =
        projector.imageToNearestIndexAndOffset(sensor_coordinates.image);
    if (sensor_coordinates.depth < 1e-1f || (index.array() < 0).any() ||
        (projector.getDimensions().array() <= index.array()).any()) {
      --repetition;
      continue;
    }

    const Point3D C_point_round_trip =
        projector.sensorToCartesian(sensor_coordinates);
    ASSERT_LE((C_point - C_point_round_trip).norm(),
              kNoiseTolerance * (1.f + sensor_coordinates.depth));

    // Compute "ground truth" using double precision
    const Point3D C_from_closest_pixel = projector.sensorToCartesian(
        {projector.indexToImage(index), sensor_coordinates.depth});
    const double projected_double =
        C_point.cast<double>().dot(C_from_closest_pixel.cast<double>()) /
        (C_point.cast<double>().norm() *
         C_from_closest_pixel.cast<double>().norm());
    const FloatingPoint angle =
        (projected_double < 1.f)
            ? static_cast<FloatingPoint>(std::acos(projected_double))
            : 0.f;

    // Compute based on the offsets
    const FloatingPoint angle_from_offset =
        projector.imageOffsetToErrorNorm(sensor_coordinates.image, offset);
    EXPECT_NEAR(angle_from_offset, angle, kMaxAcceptableAngleError)
        << "For C_point " << print::eigen::oneLine(C_point)
        << ", C_from_closest_pixel "
        << print::eigen::oneLine(C_from_closest_pixel)
        << " and intermediate sensor coordinates "
        << print::eigen::oneLine(sensor_coordinates.image) << ", "
        << sensor_coordinates.depth;
  }
}

// TODO(victorr): Test boundaries
}  // namespace wavemap

#include <gtest/gtest.h>

#include "wavemap_common/common.h"
#include "wavemap_common/integrator/projection_model/image_2d/spherical_projector.h"
#include "wavemap_common/test/eigen_utils.h"
#include "wavemap_common/test/fixture_base.h"
#include "wavemap_common/utils/eigen_format.h"

namespace wavemap {
class SphericalProjectorTest : public FixtureBase {
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
  constexpr FloatingPoint kMaxAcceptableAngleError = 2.f * kEpsilon;
  const auto projector = getRandomProjectionModel();
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const Point3D C_point = getRandomPoint<3>();
    const Vector3D sensor_coordinates = projector.cartesianToSensor(C_point);
    const auto [index, offset] =
        projector.imageToNearestIndexAndOffset(sensor_coordinates.head<2>());
    if (sensor_coordinates[2] < 1e-1f || (index.array() < 0).any() ||
        (projector.getDimensions().array() <= index.array()).any()) {
      --repetition;
      continue;
    }

    const Point3D C_point_round_trip =
        projector.sensorToCartesian(sensor_coordinates);
    ASSERT_LE((C_point - C_point_round_trip).norm(),
              kEpsilon * (1.f + sensor_coordinates[2]));

    // Compute "ground truth" using double precision
    const Point3D C_from_closest_pixel = projector.sensorToCartesian(
        projector.indexToImage(index), sensor_coordinates[2]);
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
        projector.imageOffsetToErrorNorm(sensor_coordinates.head<2>(), offset);
    EXPECT_NEAR(angle_from_offset, angle, kMaxAcceptableAngleError)
        << "For C_point " << EigenFormat::oneLine(C_point)
        << ", C_from_closest_pixel "
        << EigenFormat::oneLine(C_from_closest_pixel)
        << " and intermediate sensor coordinates "
        << EigenFormat::oneLine(sensor_coordinates);
  }
}

// TODO(victorr): Test boundaries
}  // namespace wavemap

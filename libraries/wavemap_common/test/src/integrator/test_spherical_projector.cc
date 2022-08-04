#include <gtest/gtest.h>

#include "wavemap_common/common.h"
#include "wavemap_common/integrator/spherical_projector.h"
#include "wavemap_common/test/fixture_base.h"
#include "wavemap_common/utils/eigen_format.h"

namespace wavemap {
using SphericalProjectorTest = FixtureBase;

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

    SphericalProjector spherical_projector(
        min_elevation_angle, max_elevation_angle, num_rows, min_azimuth_angle,
        max_azimuth_angle, num_columns);
    EXPECT_EQ(spherical_projector.getMinElevationAngle(), min_elevation_angle);
    EXPECT_EQ(spherical_projector.getMaxElevationAngle(), max_elevation_angle);
    EXPECT_EQ(spherical_projector.getNumRows(), num_rows);
    EXPECT_EQ(spherical_projector.getMinAzimuthAngle(), min_azimuth_angle);
    EXPECT_EQ(spherical_projector.getMaxAzimuthAngle(), max_azimuth_angle);
    EXPECT_EQ(spherical_projector.getNumColumns(), num_columns);
  }
}

TEST_F(SphericalProjectorTest, Conversions) {
  for (int repetition = 0; repetition < 1000; ++repetition) {
    // Test bearing <-> spherical
    {
      const Point3D point_original = getRandomPoint<3>();
      const FloatingPoint norm = point_original.norm();
      const Vector2D spherical_coordinates =
          SphericalProjector::bearingToSpherical(point_original);
      const Point3D point_roundtrip =
          norm * SphericalProjector::sphericalToBearing(spherical_coordinates);
      EXPECT_LE((point_roundtrip - point_original).norm(),
                kEpsilon * (1.f + norm))
          << "Original point was " << EigenFormat::oneLine(point_original)
          << " with norm " << norm << ", but after round trip it became "
          << EigenFormat::oneLine(point_roundtrip)
          << ". Intermediate spherical coordinates were "
          << EigenFormat::oneLine(spherical_coordinates) << ".";
    }
    {
      const Vector2D spherical_coords_original =
          getRandomPoint<2>().normalized();
      const Vector3D bearing =
          SphericalProjector::sphericalToBearing(spherical_coords_original);
      const Vector2D spherical_coords_roundtrip =
          SphericalProjector::bearingToSpherical(bearing);
      EXPECT_LE((spherical_coords_roundtrip - spherical_coords_original).norm(),
                kEpsilon)
          << "Original (normalized) spherical coordinates were "
          << EigenFormat::oneLine(spherical_coords_original)
          << ", but after round trip they became "
          << EigenFormat::oneLine(spherical_coords_roundtrip)
          << ". Intermediate bearing vector was "
          << EigenFormat::oneLine(bearing) << ".";
    }
  }
}
}  // namespace wavemap

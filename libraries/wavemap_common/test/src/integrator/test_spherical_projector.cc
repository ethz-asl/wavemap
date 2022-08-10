#include <gtest/gtest.h>

#include "wavemap_common/common.h"
#include "wavemap_common/integrator/projection_model/spherical_projector.h"
#include "wavemap_common/test/eigen_utils.h"
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

    // Instantiate a random projection model
    constexpr FloatingPoint kOneDegree = 0.0174533f;
    const Vector2D min_angles = {getRandomAngle(-kHalfPi, kQuarterPi),
                                 getRandomAngle(-kPi, kHalfPi)};
    const Vector2D max_angles = {
        getRandomAngle(min_angles[0] + kOneDegree, kHalfPi),
        getRandomAngle(min_angles[1] + kOneDegree, kPi)};
    const Index2D size =
        getRandomIndex<2>(Index2D::Constant(2), Index2D::Constant(1000));
    SphericalProjector spherical_projector(min_angles[0], max_angles[0],
                                           size[0], min_angles[1],
                                           max_angles[1], size[1]);

    // Precompute commonly used values
    const Index2D max_index = size - Index2D::Ones();
    const Vector2D mid_angles = min_angles + (max_angles - min_angles) / 2.f;
    const Index2D mid_index_floor = max_index / 2;
    const Index2D mid_index_ceil = (max_index.cast<FloatingPoint>() / 2.f)
                                       .array()
                                       .ceil()
                                       .cast<IndexElement>();
    const Vector2D epsilon_2d = Vector2D::Constant(kEpsilon);

    // Test angle <-> index
    {
      EXPECT_EIGEN_EQ(spherical_projector.sphericalToNearestIndex(min_angles),
                      Index2D::Zero());
      EXPECT_EIGEN_EQ(
          spherical_projector.sphericalToFloorIndex(min_angles + epsilon_2d),
          Index2D::Zero());
      EXPECT_EIGEN_NEAR(spherical_projector.indexToSpherical(Index2D::Zero()),
                        min_angles, kEpsilon);

      EXPECT_EIGEN_EQ(spherical_projector.sphericalToNearestIndex(max_angles),
                      max_index);
      EXPECT_EIGEN_EQ(
          spherical_projector.sphericalToCeilIndex(max_angles - epsilon_2d),
          max_index);
      EXPECT_EIGEN_NEAR(spherical_projector.indexToSpherical(max_index),
                        max_angles, kEpsilon);

      EXPECT_EIGEN_EQ(
          spherical_projector.sphericalToFloorIndex(mid_angles + epsilon_2d),
          mid_index_floor);
      EXPECT_EIGEN_EQ(
          spherical_projector.sphericalToCeilIndex(mid_angles - epsilon_2d),
          mid_index_ceil);
    }

    // Test bearing <-> index
    {
      const Vector3D min_bearing =
          SphericalProjector::sphericalToBearing(min_angles);
      EXPECT_EIGEN_EQ(spherical_projector.bearingToNearestIndex(min_bearing),
                      Index2D::Zero());
      const Vector3D bearing_min_index =
          spherical_projector.indexToBearing(Index2D::Zero());
      EXPECT_NEAR(bearing_min_index.x(), min_bearing.x(), kEpsilon);
      EXPECT_NEAR(bearing_min_index.y(), min_bearing.y(), kEpsilon);

      const Vector3D max_bearing =
          SphericalProjector::sphericalToBearing(max_angles);
      EXPECT_EIGEN_EQ(spherical_projector.bearingToNearestIndex(max_bearing),
                      max_index);
      const Vector3D bearing_max_index =
          spherical_projector.indexToBearing(max_index);
      EXPECT_NEAR(bearing_max_index.x(), max_bearing.x(), kEpsilon);
      EXPECT_NEAR(bearing_max_index.y(), max_bearing.y(), kEpsilon);

      const Vector3D mid_bearing =
          SphericalProjector::sphericalToBearing(mid_angles);
      EXPECT_EIGEN_LE(mid_index_floor,
                      spherical_projector.bearingToNearestIndex(mid_bearing));
      EXPECT_EIGEN_LE(spherical_projector.bearingToNearestIndex(mid_bearing),
                      mid_index_ceil.array());
    }
  }
}
}  // namespace wavemap

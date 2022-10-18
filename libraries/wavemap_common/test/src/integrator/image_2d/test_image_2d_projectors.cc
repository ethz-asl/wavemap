#include <gtest/gtest.h>

#include "wavemap_common/common.h"
#include "wavemap_common/integrator/projection_model/image_2d/ouster_projector.h"
#include "wavemap_common/integrator/projection_model/image_2d/pinhole_camera_projector.h"
#include "wavemap_common/integrator/projection_model/image_2d/spherical_projector.h"
#include "wavemap_common/test/eigen_utils.h"
#include "wavemap_common/test/fixture_base.h"
#include "wavemap_common/utils/eigen_format.h"

namespace wavemap {
template <typename PointT>
class Image2DProjectorTest : public FixtureBase {
 protected:
  static constexpr FloatingPoint kMinAngleIntervalWidth = kPi / 16.f;

  void getRandomProjectorConfig(SphericalProjectorConfig& config) {
    config.elevation.min_angle = getRandomAngle(-kQuarterPi, 0.f);
    config.elevation.max_angle = getRandomAngle(
        config.elevation.min_angle + kMinAngleIntervalWidth, kQuarterPi);
    config.elevation.num_cells = int_math::exp2(getRandomIndexElement(4, 7));

    config.azimuth.min_angle = getRandomAngle(-kPi, kHalfPi);
    config.azimuth.max_angle =
        getRandomAngle(config.azimuth.min_angle + kMinAngleIntervalWidth, kPi);
    config.azimuth.num_cells = int_math::exp2(getRandomIndexElement(7, 11));
  }

  void getRandomProjectorConfig(OusterProjectorConfig& config) {
    config.elevation.min_angle = getRandomAngle(-kQuarterPi, 0.f);
    config.elevation.max_angle = getRandomAngle(
        config.elevation.min_angle + kMinAngleIntervalWidth, kQuarterPi);
    config.elevation.num_cells = int_math::exp2(getRandomIndexElement(4, 7));

    config.azimuth.min_angle = getRandomAngle(-kPi, kHalfPi);
    config.azimuth.max_angle =
        getRandomAngle(config.azimuth.min_angle + kMinAngleIntervalWidth, kPi);
    config.azimuth.num_cells = int_math::exp2(getRandomIndexElement(7, 11));

    config.lidar_origin_to_beam_origin = getRandomSignedDistance(0.f, 0.03f);
    config.lidar_origin_to_sensor_origin_z_offset =
        getRandomSignedDistance(-0.03f, 0.03f);
  }

  void getRandomProjectorConfig(PinholeCameraProjectorConfig& config) {
    config.width = getRandomIndexElement(2, 2048);
    config.height = getRandomIndexElement(2, 2048);
    config.fx = getRandomSignedDistance(2.f, 100.f);
    config.fy = getRandomSignedDistance(2.f, 100.f);
    config.cx = getRandomSignedDistance(2.f, 2048.f);
    config.cy = getRandomSignedDistance(2.f, 2048.f);
  }
};

using ProjectorTypes = ::testing::Types<SphericalProjector, OusterProjector,
                                        PinholeCameraProjector>;
TYPED_TEST_SUITE(Image2DProjectorTest, ProjectorTypes, );

TYPED_TEST(Image2DProjectorTest, Conversions) {
  constexpr int kNumRepetitions = 10000;
  constexpr FloatingPoint min_range = 1e-2f;
  constexpr FloatingPoint max_range = 1e2f;

  // Create a projector with random params
  typename TypeParam::Config projector_config;
  TestFixture::getRandomProjectorConfig(projector_config);
  const TypeParam projector(projector_config);

  // Test Cartesian -> sensor -> Cartesian round trips
  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    // Get a random point in Cartesian space and ensure it's in the FoV
    const Point3D C_point =
        FixtureBase::getRandomPoint<3>(min_range, max_range);
    const Vector3D sensor_coordinates = projector.cartesianToSensor(C_point);
    const Vector2D image_coordinates = sensor_coordinates.head<2>();
    const FloatingPoint range_or_depth = sensor_coordinates[2];
    const Index2D image_index =
        projector.imageToNearestIndex(image_coordinates);
    if ((image_index.array() < 0 ||
         projector.getDimensions().array() <= image_index.array())
            .any() ||
        range_or_depth < 1e-1f) {
      --repetition;
      continue;
    }

    const Point3D C_point_roundtrip =
        projector.sensorToCartesian(sensor_coordinates);
    const FloatingPoint range = C_point.norm();

    EXPECT_LE((C_point_roundtrip - C_point).norm(), kEpsilon * (1.f + range))
        << "Original point was " << EigenFormat::oneLine(C_point)
        << " with norm " << range << ", but after round trip it became "
        << EigenFormat::oneLine(C_point_roundtrip)
        << ". Intermediate sensor coordinates were "
        << EigenFormat::oneLine(sensor_coordinates) << ".";
  }

  // Test sensor -> Cartesian -> sensor round trips
  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    // Get a random point in sensor coordinates and ensure it's in the FoV
    const Index2D image_index = FixtureBase::getRandomIndex<2>(
        Index2D::Zero(), projector.getDimensions());
    const Vector2D image_coordinates = projector.indexToImage(image_index);
    const Point3D C_point = projector.sensorToCartesian(image_coordinates, 1.f);
    const Index2D image_index_roundtrip =
        projector.cartesianToNearestIndex(C_point);

    EXPECT_EQ(image_index_roundtrip, image_index)
        << "Original image index was " << EigenFormat::oneLine(image_index)
        << ", but after round trip it became "
        << EigenFormat::oneLine(image_index_roundtrip)
        << ". Intermediate image and Cartesian coordinates were "
        << EigenFormat::oneLine(image_coordinates) << " and "
        << EigenFormat::oneLine(C_point) << ".";
  }
}
}  // namespace wavemap

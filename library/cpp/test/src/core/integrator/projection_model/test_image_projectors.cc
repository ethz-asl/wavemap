#include <algorithm>
#include <iostream>
#include <list>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "wavemap/core/common.h"
#include "wavemap/core/integrator/projection_model/ouster_projector.h"
#include "wavemap/core/integrator/projection_model/pinhole_camera_projector.h"
#include "wavemap/core/integrator/projection_model/spherical_projector.h"
#include "wavemap/core/utils/math/angle_normalization.h"
#include "wavemap/core/utils/print/container.h"
#include "wavemap/core/utils/print/eigen.h"
#include "wavemap/test/eigen_utils.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
class Image2DProjectorTest : public FixtureBase, public GeometryGenerator {
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

  struct AABBAndPose {
    AABB<Point3D> W_aabb;
    Transformation3D T_W_C;

    AABBAndPose(AABB<Point3D> W_aabb, const Transformation3D& T_W_C)
        : W_aabb(std::move(W_aabb)), T_W_C(T_W_C) {}
  };

  std::vector<AABBAndPose> getTestAABBsAndPoses() {
    // Manually define initial AABBs
    std::list<AABB<Point3D>> aabbs{
        {Point3D::Constant(kEpsilon), Point3D::Ones()},
        {Point3D::Constant(kEpsilon), {0.5f, 1.f, 1.f}},
        {Point3D::Constant(kEpsilon), {1.f, 0.5f, 1.f}},
        {Point3D::Constant(kEpsilon), {1.f, 1.f, 0.5f}}};
    // Insert copies of all above AABBs flipped along the X-axis
    std::generate_n(
        std::back_inserter(aabbs), aabbs.size(),
        [aabbs_it = aabbs.cbegin()]() mutable {
          AABB<Point3D> aabb_flipped{
              {-aabbs_it->max.x(), aabbs_it->min.y(), aabbs_it->min.z()},
              {-aabbs_it->min.x(), aabbs_it->max.y(), aabbs_it->max.z()}};
          ++aabbs_it;
          return aabb_flipped;
        });
    // Insert copies of the initial AABBs flipped along the Y-axis
    std::generate_n(
        std::back_inserter(aabbs), aabbs.size(),
        [aabbs_it = aabbs.cbegin()]() mutable {
          AABB<Point3D> aabb_flipped{
              {aabbs_it->min.x(), -aabbs_it->max.y(), aabbs_it->min.z()},
              {aabbs_it->max.x(), -aabbs_it->min.y(), aabbs_it->max.z()}};
          ++aabbs_it;
          return aabb_flipped;
        });
    // Insert copies of the initial AABBs flipped along the Z-axis
    std::generate_n(
        std::back_inserter(aabbs), aabbs.size(),
        [aabbs_it = aabbs.cbegin()]() mutable {
          AABB<Point3D> aabb_flipped{
              {aabbs_it->min.x(), aabbs_it->min.y(), -aabbs_it->max.z()},
              {aabbs_it->max.x(), aabbs_it->max.y(), -aabbs_it->min.z()}};
          ++aabbs_it;
          return aabb_flipped;
        });
    // NOTE: The AABB set now consists of the initial AABBs and copies for all
    //       combinations of flips (X, Y, Z, XY, XZ, ..., XYZ).

    // Create tests by combining the above AABBs (incl. random rescaling and
    // translations) with identity and random sensor poses
    std::vector<AABBAndPose> aabbs_and_poses;
    for (const auto& aabb : aabbs) {
      for (int i = 0; i < 10; ++i) {
        const FloatingPoint random_scale = 1.f / getRandomMinCellWidth();
        for (const Vector3D& t_random :
             {getRandomTranslation<3>(),
              Vector3D{getRandomSignedDistance(), 0.f, 0.f},
              Vector3D{0.f, getRandomSignedDistance(), 0.f},
              Vector3D{0.f, 0.f, getRandomSignedDistance()}}) {
          const AABB<Point3D> aabb_scaled{random_scale * aabb.min,
                                          random_scale * aabb.max};
          const AABB<Point3D> aabb_translated{aabb.min + t_random,
                                              aabb.max + t_random};
          const AABB<Point3D> aabb_scaled_translated{
              aabb_scaled.min + t_random, aabb_scaled.max + t_random};
          const Transformation3D T_W_C_random = getRandomTransformation();
          aabbs_and_poses.emplace_back(aabb_scaled, Transformation3D());
          aabbs_and_poses.emplace_back(aabb_translated, Transformation3D());
          aabbs_and_poses.emplace_back(aabb_scaled_translated,
                                       Transformation3D());
          aabbs_and_poses.emplace_back(aabb, T_W_C_random);
          aabbs_and_poses.emplace_back(aabb_scaled, T_W_C_random);
          aabbs_and_poses.emplace_back(aabb_translated, T_W_C_random);
          aabbs_and_poses.emplace_back(aabb_scaled_translated, T_W_C_random);
        }
      }
    }
    return aabbs_and_poses;
  }
};

template <typename T>
using Image2DProjectorTypedTest = Image2DProjectorTest;

using ProjectorTypes = ::testing::Types<SphericalProjector, OusterProjector,
                                        PinholeCameraProjector>;
TYPED_TEST_SUITE(Image2DProjectorTypedTest, ProjectorTypes, );

TYPED_TEST(Image2DProjectorTypedTest, Conversions) {
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
        GeometryGenerator::getRandomPoint<3>(min_range, max_range);
    const auto sensor_coordinates = projector.cartesianToSensor(C_point);
    const Vector2D image_coordinates = sensor_coordinates.image;
    const FloatingPoint range_or_depth = sensor_coordinates.depth;
    if (range_or_depth < 1e-1f) {
      --repetition;
      continue;
    }
    const Index2D image_index =
        projector.imageToNearestIndex(image_coordinates);
    if ((image_index.array() < 0 ||
         projector.getDimensions().array() <= image_index.array())
            .any()) {
      --repetition;
      continue;
    }

    const Point3D C_point_roundtrip =
        projector.sensorToCartesian(sensor_coordinates);
    const FloatingPoint range = C_point.norm();

    constexpr FloatingPoint kNoiseTolerance = 1e-5f;
    EXPECT_LE((C_point_roundtrip - C_point).norm(),
              kNoiseTolerance * (1.f + range))
        << "Original point was " << print::eigen::oneLine(C_point)
        << " with norm " << range << ", but after round trip it became "
        << print::eigen::oneLine(C_point_roundtrip)
        << ". Intermediate sensor coordinates were "
        << print::eigen::oneLine(sensor_coordinates.image) << ", "
        << sensor_coordinates.depth << ".";
  }

  // Test sensor -> Cartesian -> sensor round trips
  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    // Get a random point in sensor coordinates and ensure it's in the FoV
    const Index2D image_index = GeometryGenerator::getRandomIndex<2>(
        Index2D::Zero(), projector.getDimensions());
    const Vector2D image_coordinates = projector.indexToImage(image_index);
    const Point3D C_point =
        projector.sensorToCartesian({image_coordinates, 1.f});
    const Index2D image_index_roundtrip =
        projector.cartesianToNearestIndex(C_point);

    EXPECT_EQ(image_index_roundtrip, image_index)
        << "Original image index was " << print::eigen::oneLine(image_index)
        << ", but after round trip it became "
        << print::eigen::oneLine(image_index_roundtrip)
        << ". Intermediate image and Cartesian coordinates were "
        << print::eigen::oneLine(image_coordinates) << " and "
        << print::eigen::oneLine(C_point) << ".";
  }

  // Test index -> image -> index round trips
  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    // Get a random point in sensor coordinates and ensure it's in the FoV
    const Index2D image_index = GeometryGenerator::getRandomIndex<2>(
        Index2D::Zero(), projector.getDimensions());
    const Vector2D image_coordinates = projector.indexToImage(image_index);
    const Index2D image_index_roundtrip =
        projector.imageToNearestIndex(image_coordinates);

    EXPECT_EQ(image_index_roundtrip, image_index)
        << "Original image index was " << print::eigen::oneLine(image_index)
        << ", but after round trip it became "
        << print::eigen::oneLine(image_index_roundtrip)
        << ". Intermediate image coordinates were "
        << print::eigen::oneLine(image_coordinates) << " and "
        << print::eigen::oneLine(image_coordinates) << ".";
  }
}

TYPED_TEST(Image2DProjectorTypedTest, SensorCoordinateAABBs) {
  constexpr int kNumRandomProjectorConfigs = 10;
  for (int config_idx = 0; config_idx < kNumRandomProjectorConfigs;
       ++config_idx) {
    // Create a projector with random params
    typename TypeParam::Config projector_config;
    Image2DProjectorTest::getRandomProjectorConfig(projector_config);
    const TypeParam projector(projector_config);

    // Run tests
    int error_count = 0;
    const auto tests = TestFixture::getTestAABBsAndPoses();
    for (const auto& test : tests) {
      // The Projector::cartesianToSensorAABB(...) method is only valid if the
      // AABB does not contain the sensor's origin
      if (test.W_aabb.contains(test.T_W_C.getPosition())) {
        continue;
      }

      AABB<Vector3D> reference_aabb;
      std::array<FloatingPoint, AABB<Vector3D>::kNumCorners> corners_x{};
      std::array<FloatingPoint, AABB<Vector3D>::kNumCorners> corners_y{};
      std::array<FloatingPoint, AABB<Vector3D>::kNumCorners> corners_z{};
      // Include all four corners
      for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
           ++corner_idx) {
        const Point3D C_t_C_corner =
            test.T_W_C.inverse() * test.W_aabb.corner_point(corner_idx);
        const auto corner_sensor_coordinates =
            projector.cartesianToSensor(C_t_C_corner);
        corners_x[corner_idx] = corner_sensor_coordinates.image.x();
        corners_y[corner_idx] = corner_sensor_coordinates.image.y();
        corners_z[corner_idx] = corner_sensor_coordinates.depth;
      }
      // Find the min/max corner coordinates
      for (auto [axis, coordinates] :
           {std::pair{0, corners_x}, {1, corners_y}, {2, corners_z}}) {
        std::sort(coordinates.begin(), coordinates.end());
        const FloatingPoint min_angle = coordinates.front();
        const FloatingPoint max_angle = coordinates.back();
        const bool angle_range_wraps_around =
            projector.sensorAxisCouldBePeriodic()[axis] &&
            kPi < max_angle - min_angle;
        if (angle_range_wraps_around) {
          const FloatingPoint smallest_angle_above_zero =
              *std::upper_bound(coordinates.cbegin(), coordinates.cend(), 0.f);
          const FloatingPoint greatest_angle_below_zero = *std::prev(
              std::upper_bound(coordinates.cbegin(), coordinates.cend(), 0.f));
          reference_aabb.min[axis] = smallest_angle_above_zero;
          reference_aabb.max[axis] = greatest_angle_below_zero;
        } else {
          reference_aabb.min[axis] = min_angle;
          reference_aabb.max[axis] = max_angle;
        }
      }
      // Also include the Z-coordinates of the AABB's closest and furthest pts
      {
        const Point3D C_closest_point =
            test.T_W_C.inverse() *
            test.W_aabb.closestPointTo(test.T_W_C.getPosition());
        reference_aabb.min.z() =
            std::min(reference_aabb.min.z(),
                     projector.cartesianToSensorZ(C_closest_point));
        const Point3D C_furthest_point =
            test.T_W_C.inverse() *
            test.W_aabb.furthestPointFrom(test.T_W_C.getPosition());
        reference_aabb.max.z() =
            std::max(reference_aabb.max.z(),
                     projector.cartesianToSensorZ(C_furthest_point));
      }

      // For pinhole cameras, AABBs that (partially or fully) lie behind the
      // camera are handled in a special way and not tested here
      // TODO(victorr): Add dedicated tests for pinhole cameras
      if (reference_aabb.min.z() < 0.5f) {
        continue;
      }

      const auto returned_angle_pair = projector.cartesianToSensorAABB(
          test.W_aabb, test.T_W_C.getRotation().inverse().getRotationMatrix(),
          test.T_W_C.getPosition());
      // TODO(victorr): Properly define all 'acceptable errors' in pixel space
      const Vector2D image_stride = projector.indexToImage(Index2D::Ones()) -
                                    projector.indexToImage(Index2D::Zero());
      constexpr FloatingPoint kAcceptableNoise = 0.1f;
      constexpr FloatingPoint kAcceptablePadding = 0.4f;

      bool check_failed = false;
      auto canary_token = [&check_failed]() {
        check_failed = true;
        return "";
      };
      for (const int axis : {0, 1}) {
        const bool angle_range_wraps_around =
            projector.sensorAxisCouldBePeriodic()[axis] &&
            kPi < reference_aabb.max[axis] - reference_aabb.min[axis];
        if (angle_range_wraps_around) {
          EXPECT_GE(angle_math::normalize(returned_angle_pair.min[axis] -
                                          reference_aabb.min[axis]),
                    0.f)
              << " for axis " << axis << canary_token();
          EXPECT_LE(angle_math::normalize(returned_angle_pair.min[axis] -
                                          reference_aabb.min[axis]),
                    kAcceptablePadding * image_stride[axis])
              << " for axis " << axis << canary_token();
          EXPECT_LE(angle_math::normalize(returned_angle_pair.max[axis] -
                                          reference_aabb.max[axis]),
                    0.f)
              << " for axis " << axis << canary_token();
          EXPECT_GE(angle_math::normalize(returned_angle_pair.max[axis] -
                                          reference_aabb.max[axis]),
                    -kAcceptablePadding * image_stride[axis])
              << " for axis " << axis << canary_token();
        } else {
          EXPECT_LE(angle_math::normalize(returned_angle_pair.min[axis] -
                                          reference_aabb.min[axis]),
                    kAcceptableNoise * image_stride[axis])
              << " for axis " << axis << canary_token();
          EXPECT_GE(angle_math::normalize(returned_angle_pair.min[axis] -
                                          reference_aabb.min[axis]),
                    -kAcceptablePadding * image_stride[axis])
              << " for axis " << axis << canary_token();
          EXPECT_GE(angle_math::normalize(returned_angle_pair.max[axis] -
                                          reference_aabb.max[axis]),
                    -kAcceptableNoise * image_stride[axis])
              << " for axis " << axis << canary_token();
          EXPECT_LE(angle_math::normalize(returned_angle_pair.max[axis] -
                                          reference_aabb.max[axis]),
                    kAcceptablePadding * image_stride[axis])
              << " for axis " << axis << canary_token();
        }
      }

      if (check_failed) {
        const AABB<Point3D>::Corners C_t_C_corners =
            test.T_W_C.inverse().transformVectorized(
                test.W_aabb.corner_matrix());
        std::cerr << "For\n-W_aabb: " << test.W_aabb.toString() << "\n-T_W_C:\n"
                  << test.T_W_C << "\nWith C_cell_corners:\n"
                  << C_t_C_corners << "\nsensor X-coordinates:\n"
                  << print::sequence(corners_x) << "\nsensor Y-coordinates:\n"
                  << print::sequence(corners_y) << print::sequence(corners_x)
                  << "\nsensor Z-coordinates:\n"
                  << print::sequence(corners_z)
                  << "\nand reference min/max sensor coordinates: "
                  << print::eigen::oneLine(reference_aabb.min) << ", "
                  << print::eigen::oneLine(reference_aabb.max)
                  << "\nWe got min/max sensor coordinates: "
                  << print::eigen::oneLine(returned_angle_pair.min) << ", "
                  << print::eigen::oneLine(returned_angle_pair.max)
                  << "\nThis is error nr " << ++error_count << "\n"
                  << std::endl;
      }
    }
  }
}

TYPED_TEST(Image2DProjectorTypedTest, imageToNearestIndicesAndOffsets) {
  constexpr int kNumRandomProjectorConfigs = 10;
  for (int config_idx = 0; config_idx < kNumRandomProjectorConfigs;
       ++config_idx) {
    // Create a projector with random params
    typename TypeParam::Config projector_config;
    Image2DProjectorTest::getRandomProjectorConfig(projector_config);
    const TypeParam projector(projector_config);

    // Test single and batched computation equivalence
    const Vector2D image_coordinates = Vector2D::Random();
    const auto indices = projector.imageToNearestIndices(image_coordinates);
    const auto indices_and_offsets =
        projector.imageToNearestIndicesAndOffsets(image_coordinates);

    const Vector2D index_real = projector.imageToIndexReal(image_coordinates);
    const Vector2D index_lower = index_real.array().floor();
    const Vector2D index_upper = index_real.array().ceil();

    for (int neighbox_idx = 0; neighbox_idx < 4; ++neighbox_idx) {
      const Vector2D index_rounded{
          neighbox_idx & 0b01 ? index_upper[0] : index_lower[0],
          neighbox_idx & 0b10 ? index_upper[1] : index_lower[1]};
      const Index2D index_expected = index_rounded.cast<IndexElement>();
      const Vector2D offset_expected =
          projector.index_to_image_scale_factor_.cwiseProduct(index_rounded -
                                                              index_real);

      EXPECT_EQ(indices(0, neighbox_idx), index_expected[0]);
      EXPECT_EQ(indices(1, neighbox_idx), index_expected[1]);

      EXPECT_EQ(indices_and_offsets.first(0, neighbox_idx), index_expected[0]);
      EXPECT_EQ(indices_and_offsets.first(1, neighbox_idx), index_expected[1]);
      EXPECT_NEAR(indices_and_offsets.second(0, neighbox_idx),
                  offset_expected[0], kEpsilon);
      EXPECT_NEAR(indices_and_offsets.second(1, neighbox_idx),
                  offset_expected[1], kEpsilon);
    }
  }
}

TYPED_TEST(Image2DProjectorTypedTest, ImageOffsetErrorNorms) {
  constexpr int kNumRandomProjectorConfigs = 10;
  for (int config_idx = 0; config_idx < kNumRandomProjectorConfigs;
       ++config_idx) {
    // Create a projector with random params
    typename TypeParam::Config projector_config;
    Image2DProjectorTest::getRandomProjectorConfig(projector_config);
    const TypeParam projector(projector_config);

    // Test single and batched computation equivalence
    const Vector2D linearization_point = Vector2D::Random();
    const ProjectorBase::CellToBeamOffsetArray offsets =
        ProjectorBase::CellToBeamOffsetArray::Random();
    const auto error_norms =
        projector.imageOffsetsToErrorNorms(linearization_point, offsets);
    for (int offset_idx = 0; offset_idx < 4; ++offset_idx) {
      const FloatingPoint error_norm = projector.imageOffsetToErrorNorm(
          linearization_point, offsets.col(offset_idx));
      EXPECT_NEAR(error_norms[offset_idx], error_norm, kEpsilon);
    }
  }
}
}  // namespace wavemap

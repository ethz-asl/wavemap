#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/integrator/pointcloud_integrator.h>
#include <wavemap_common/test/fixture_base.h>
#include <wavemap_common/utils/angle_utils.h>
#include <wavemap_common/utils/container_print_utils.h>
#include <wavemap_common/utils/eigen_format.h>

#include "wavemap_3d/integrator/projective/coarse_to_fine/range_image_2d_intersector.h"

namespace wavemap {
class RangeImage2DIntersectorTest : public FixtureBase {
 protected:
  PointcloudIntegratorConfig getRandomPointcloudIntegratorConfig() {
    const FloatingPoint min_range = getRandomSignedDistance(1e-1f, 3.f);
    const FloatingPoint max_range = getRandomSignedDistance(min_range, 40.f);
    return PointcloudIntegratorConfig{min_range, max_range};
  }

  SphericalProjector getRandomProjectionModel() {
    const FloatingPoint min_elevation_angle = getRandomAngle(-kQuarterPi, 0.f);
    const FloatingPoint max_elevation_angle =
        getRandomAngle(min_elevation_angle, kQuarterPi);
    const FloatingPoint min_azimuth_angle = -kPi;
    const FloatingPoint max_azimuth_angle = kPi;
    const int num_rows = int_math::exp2(getRandomIndexElement(4, 7));
    const int num_cols = int_math::exp2(getRandomIndexElement(7, 11));
    return SphericalProjector(SphericalProjectorConfig{
        {min_elevation_angle, max_elevation_angle, num_rows},
        {min_azimuth_angle, max_azimuth_angle, num_cols}});
  }

  ContinuousVolumetricLogOdds<3> getRandomMeasurementModel(
      const SphericalProjector& projection_model) {
    ContinuousVolumetricLogOddsConfig measurement_model_config;
    const FloatingPoint max_angle_sigma_without_overlap =
        (projection_model.getMaxImageCoordinates() -
         projection_model.getMinImageCoordinates())
            .cwiseQuotient(
                projection_model.getDimensions().cast<FloatingPoint>())
            .minCoeff() /
        (2.f * 6.f);
    measurement_model_config.angle_sigma =
        random_number_generator_->getRandomRealNumber(
            max_angle_sigma_without_overlap / 10.f,
            max_angle_sigma_without_overlap);
    measurement_model_config.range_sigma =
        random_number_generator_->getRandomRealNumber(1e-3f, 5e-2f);
    return ContinuousVolumetricLogOdds<3>(measurement_model_config);
  }

  PosedRangeImage2D getRandomPosedRangeImage(IndexElement num_rows,
                                             IndexElement num_cols,
                                             FloatingPoint min_range,
                                             FloatingPoint max_range) {
    CHECK_LT(min_range, max_range);

    PosedRangeImage2D posed_range_image(num_rows, num_cols);
    for (const Index2D& index :
         Grid<2>(Index2D::Zero(), {num_rows - 1, num_cols - 1})) {
      const FloatingPoint range = getRandomSignedDistance(min_range, max_range);
      posed_range_image.getRange(index) = range;
    }
    posed_range_image.setPose(getRandomTransformation<3>());

    return posed_range_image;
  }

  struct AABBAndPose {
    AABB<Point3D> W_aabb;
    Transformation3D T_W_C;

    AABBAndPose(AABB<Point3D> W_aabb, const Transformation3D& T_W_C)
        : W_aabb(std::move(W_aabb)), T_W_C(T_W_C) {}
  };

  std::vector<AABBAndPose> getTestAABBsAndPoses() const {
    // Manually define initial AABBs
    std::list<AABB<Point3D>> aabbs{{Point3D::Zero(), Point3D::Ones()},
                                   {Point3D::Zero(), {0.5f, 1.f, 1.f}},
                                   {Point3D::Zero(), {1.f, 0.5f, 1.f}},
                                   {Point3D::Zero(), {1.f, 1.f, 0.5f}}};
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
      for (int i = 0; i < 1000; ++i) {
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
          const Transformation3D T_W_C_random = getRandomTransformation<3>();
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

TEST_F(RangeImage2DIntersectorTest, AabbMinMaxProjectedAngle) {
  // Generate test set
  std::vector<AABBAndPose> tests = getTestAABBsAndPoses();
  const auto projection_model = getRandomProjectionModel();

  // Run tests
  int error_count = 0;
  for (const auto& test : tests) {
    RangeImage2DIntersector::MinMaxAnglePair reference_angle_pair;
    using AngleCornerArray =
        std::array<FloatingPoint, AABB<Point3D>::kNumCorners>;
    AngleCornerArray elevation_angles{};
    AngleCornerArray azimuth_angles{};
    if (test.W_aabb.containsPoint(test.T_W_C.getPosition())) {
      reference_angle_pair.min_spherical_coordinates = -Vector2D::Constant(kPi);
      reference_angle_pair.max_spherical_coordinates = Vector2D::Constant(kPi);
    } else {
      for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
           ++corner_idx) {
        const Point3D C_t_C_corner =
            test.T_W_C.inverse() * test.W_aabb.corner_point(corner_idx);
        const Vector2D angles = projection_model.cartesianToImage(C_t_C_corner);
        elevation_angles[corner_idx] = angles[0];
        azimuth_angles[corner_idx] = angles[1];
      }
      for (auto [axis, angles] :
           {std::pair{0, &elevation_angles}, {1, &azimuth_angles}}) {
        std::sort(angles->begin(), angles->end());
        const FloatingPoint min_angle = angles->front();
        const FloatingPoint max_angle = angles->back();
        const bool angle_range_wraps_around = kPi < max_angle - min_angle;
        if (angle_range_wraps_around) {
          const FloatingPoint smallest_angle_above_zero =
              *std::upper_bound(angles->cbegin(), angles->cend(), 0.f);
          const FloatingPoint greatest_angle_below_zero = *std::prev(
              std::upper_bound(angles->cbegin(), angles->cend(), 0.f));
          reference_angle_pair.min_spherical_coordinates[axis] =
              smallest_angle_above_zero;
          reference_angle_pair.max_spherical_coordinates[axis] =
              greatest_angle_below_zero;
        } else {
          reference_angle_pair.min_spherical_coordinates[axis] = min_angle;
          reference_angle_pair.max_spherical_coordinates[axis] = max_angle;
        }
      }
    }

    const RangeImage2DIntersector::MinMaxAnglePair returned_angle_pair =
        RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
            test.T_W_C, test.W_aabb, projection_model);
    constexpr FloatingPoint kOneAndAHalfDegree = 0.0261799f;

    bool check_failed = false;
    auto canary_token = [&check_failed]() {
      check_failed = true;
      return "";
    };
    for (const int axis : {0, 1}) {
      const bool angle_range_wraps_around =
          kPi < reference_angle_pair.max_spherical_coordinates[axis] -
                    reference_angle_pair.min_spherical_coordinates[axis];
      if (angle_range_wraps_around) {
        EXPECT_GE(angle_math::normalize(
                      returned_angle_pair.min_spherical_coordinates[axis] -
                      reference_angle_pair.min_spherical_coordinates[axis]),
                  0.f)
            << canary_token();
        EXPECT_LE(angle_math::normalize(
                      returned_angle_pair.min_spherical_coordinates[axis] -
                      reference_angle_pair.min_spherical_coordinates[axis]),
                  kOneAndAHalfDegree);
        EXPECT_LE(angle_math::normalize(
                      returned_angle_pair.max_spherical_coordinates[axis] -
                      reference_angle_pair.max_spherical_coordinates[axis]),
                  0.f)
            << canary_token();
        EXPECT_GE(angle_math::normalize(
                      returned_angle_pair.max_spherical_coordinates[axis] -
                      reference_angle_pair.max_spherical_coordinates[axis]),
                  -kOneAndAHalfDegree)
            << canary_token();
      } else {
        EXPECT_LE(angle_math::normalize(
                      returned_angle_pair.min_spherical_coordinates[axis] -
                      reference_angle_pair.min_spherical_coordinates[axis]),
                  0.f)
            << canary_token();
        EXPECT_GE(angle_math::normalize(
                      returned_angle_pair.min_spherical_coordinates[axis] -
                      reference_angle_pair.min_spherical_coordinates[axis]),
                  -kOneAndAHalfDegree)
            << canary_token();
        EXPECT_GE(angle_math::normalize(
                      returned_angle_pair.max_spherical_coordinates[axis] -
                      reference_angle_pair.max_spherical_coordinates[axis]),
                  0.f)
            << canary_token();
        EXPECT_LE(angle_math::normalize(
                      returned_angle_pair.max_spherical_coordinates[axis] -
                      reference_angle_pair.max_spherical_coordinates[axis]),
                  kOneAndAHalfDegree)
            << canary_token();
      }
    }

    if (check_failed) {
      const AABB<Point3D>::Corners C_t_C_corners =
          test.T_W_C.inverse().transformVectorized(test.W_aabb.corner_matrix());
      std::cerr
          << "For\n-W_aabb: " << test.W_aabb.toString() << "\n-T_W_C:\n"
          << test.T_W_C << "\nWith C_cell_corners:\n"
          << C_t_C_corners << "\nelevation angles:\n"
          << ToString(elevation_angles) << "\nazimuth angles:\n"
          << ToString(azimuth_angles)
          << "\nand reference min/max spherical coordinates: "
          << EigenFormat::oneLine(
                 reference_angle_pair.min_spherical_coordinates)
          << ", "
          << EigenFormat::oneLine(
                 reference_angle_pair.max_spherical_coordinates)
          << "\nWe got min/max spherical coordinates: "
          << EigenFormat::oneLine(returned_angle_pair.min_spherical_coordinates)
          << ", "
          << EigenFormat::oneLine(returned_angle_pair.max_spherical_coordinates)
          << "\nThis is error nr " << ++error_count << "\n"
          << std::endl;
    }
  }
}

TEST_F(RangeImage2DIntersectorTest, RangeImageIntersectionType) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random pointcloud
    const FloatingPoint min_cell_width = getRandomMinCellWidth();
    const auto integrator_config = getRandomPointcloudIntegratorConfig();
    const auto projection_model = getRandomProjectionModel();
    const auto measurement_model = getRandomMeasurementModel(projection_model);
    constexpr FloatingPoint kMaxRange = 60.f;
    const auto posed_range_image =
        std::make_shared<PosedRangeImage2D>(getRandomPosedRangeImage(
            projection_model.getNumRows(), projection_model.getNumColumns(),
            0.f, kMaxRange));

    // Create the hierarchical range image
    RangeImage2DIntersector range_image_intersector(
        posed_range_image, projection_model, integrator_config.min_range,
        integrator_config.max_range, measurement_model.getAngleThreshold(),
        measurement_model.getRangeThresholdInFrontOfSurface(),
        measurement_model.getRangeThresholdBehindSurface());

    const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
    constexpr NdtreeIndexElement kMaxHeight = 6;
    const Index3D min_index = convert::pointToCeilIndex<3>(
        posed_range_image->getPose().getPosition() -
            Vector3D::Constant(kMaxRange),
        min_cell_width_inv);
    const Index3D max_index = convert::pointToCeilIndex<3>(
        posed_range_image->getPose().getPosition() +
            Vector3D::Constant(kMaxRange),
        min_cell_width_inv);
    for (const Index3D& index :
         getRandomIndexVector(min_index, max_index, 50, 100)) {
      const NdtreeIndexElement height =
          getRandomNdtreeIndexHeight(2, kMaxHeight);
      const OctreeIndex query_index =
          convert::indexAndHeightToNodeIndex(index, height);
      const Index3D min_reference_index =
          convert::nodeIndexToMinCornerIndex(query_index);
      const Index3D max_reference_index =
          convert::nodeIndexToMaxCornerIndex(query_index);
      bool has_free = false;
      bool has_occupied = false;
      bool has_unknown = false;
      const Transformation3D T_C_W = posed_range_image->getPoseInverse();
      for (const Index3D& reference_index :
           Grid(min_reference_index, max_reference_index)) {
        const Point3D W_cell_center =
            convert::indexToCenterPoint(reference_index, min_cell_width);
        const Point3D C_cell_center = T_C_W * W_cell_center;
        const FloatingPoint d_C_cell = C_cell_center.norm();
        if (integrator_config.max_range < d_C_cell) {
          has_unknown = true;
          continue;
        }

        const Index2D range_image_index =
            projection_model.cartesianToNearestIndex(C_cell_center);
        if ((range_image_index.array() < 0).any() ||
            (posed_range_image->getDimensions().array() <=
             range_image_index.array())
                .any()) {
          has_unknown = true;
          continue;
        }

        const FloatingPoint range_image_distance =
            posed_range_image->getRange(range_image_index);
        if (d_C_cell <
            range_image_distance -
                measurement_model.getRangeThresholdInFrontOfSurface()) {
          has_free = true;
        } else if (d_C_cell <=
                   range_image_distance +
                       measurement_model.getRangeThresholdBehindSurface()) {
          has_occupied = true;
        } else {
          has_unknown = true;
        }
      }
      ASSERT_TRUE(has_free || has_occupied || has_unknown);
      IntersectionType reference_intersection_type;
      if (has_occupied) {
        reference_intersection_type = IntersectionType::kPossiblyOccupied;
      } else if (has_free) {
        reference_intersection_type = IntersectionType::kFreeOrUnknown;
      } else {
        reference_intersection_type = IntersectionType::kFullyUnknown;
      }

      const FloatingPoint node_width =
          convert::heightToCellWidth(min_cell_width, query_index.height);
      const Point3D W_node_center =
          convert::indexToCenterPoint(query_index.position, node_width);

      const Point3D W_node_bottom_left =
          W_node_center - Vector3D::Constant(node_width / 2.f);
      const AABB<Point3D> W_cell_aabb{
          W_node_bottom_left,
          W_node_bottom_left + Vector3D::Constant(node_width)};
      const IntersectionType returned_intersection_type =
          range_image_intersector.determineIntersectionType(
              posed_range_image->getPose(), W_cell_aabb);
      EXPECT_TRUE(reference_intersection_type <= returned_intersection_type)
          << "Expected " << getIntersectionTypeStr(reference_intersection_type)
          << " but got " << getIntersectionTypeStr(returned_intersection_type);
    }
  }
}
}  // namespace wavemap

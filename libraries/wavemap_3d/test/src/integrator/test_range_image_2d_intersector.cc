#include <gtest/gtest.h>
#include <wavemap/common.h>
#include <wavemap/integrator/pointcloud_integrator.h>
#include <wavemap/integrator/projection_model/image_2d/spherical_projector.h>
#include <wavemap/test/fixture_base.h>
#include <wavemap/utils/container_print_utils.h>
#include <wavemap/utils/eigen_format.h>

#include "wavemap_3d/integrator/projective/coarse_to_fine/range_image_2d_intersector.h"

namespace wavemap {
class RangeImage2DIntersectorTest : public FixtureBase {
 protected:
  PointcloudIntegratorConfig getRandomPointcloudIntegratorConfig() {
    const FloatingPoint min_range = getRandomSignedDistance(1e-1f, 3.f);
    const FloatingPoint max_range = getRandomSignedDistance(min_range, 40.f);
    return PointcloudIntegratorConfig{min_range, max_range};
  }

  std::shared_ptr<SphericalProjector> getRandomProjectionModel() {
    const FloatingPoint min_elevation_angle = getRandomAngle(-kQuarterPi, 0.f);
    const FloatingPoint max_elevation_angle =
        getRandomAngle(min_elevation_angle, kQuarterPi);
    const FloatingPoint min_azimuth_angle = -kPi;
    const FloatingPoint max_azimuth_angle = kPi;
    const int num_rows = int_math::exp2(getRandomIndexElement(4, 7));
    const int num_cols = int_math::exp2(getRandomIndexElement(7, 11));
    return std::make_shared<SphericalProjector>(SphericalProjectorConfig{
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
};

TEST_F(RangeImage2DIntersectorTest, RangeImageUpdateType) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random pointcloud
    const FloatingPoint min_cell_width = getRandomMinCellWidth();
    const auto integrator_config = getRandomPointcloudIntegratorConfig();
    const auto projection_model = getRandomProjectionModel();
    const auto measurement_model = getRandomMeasurementModel(*projection_model);
    constexpr FloatingPoint kMaxRange = 60.f;
    const auto posed_range_image =
        std::make_shared<PosedRangeImage2D>(getRandomPosedRangeImage(
            projection_model->getNumRows(), projection_model->getNumColumns(),
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
            projection_model->cartesianToNearestIndex(C_cell_center);
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
      UpdateType reference_update_type;
      if (has_occupied) {
        reference_update_type = UpdateType::kPossiblyOccupied;
      } else if (has_free) {
        reference_update_type = UpdateType::kFreeOrUnobserved;
      } else {
        reference_update_type = UpdateType::kFullyUnobserved;
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
      const UpdateType returned_update_type =
          range_image_intersector.determineUpdateType(
              W_cell_aabb,
              posed_range_image->getPose().inverse().getRotationMatrix(),
              posed_range_image->getPose().getPosition());
      EXPECT_TRUE(reference_update_type <= returned_update_type)
          << "Expected " << getUpdateTypeStr(reference_update_type)
          << " but got " << getUpdateTypeStr(returned_update_type);
    }
  }
}
}  // namespace wavemap

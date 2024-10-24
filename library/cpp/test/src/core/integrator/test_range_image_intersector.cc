#include <memory>

#include <gtest/gtest.h>

#include "wavemap/core/common.h"
#include "wavemap/core/integrator/measurement_model/continuous_beam.h"
#include "wavemap/core/integrator/projection_model/spherical_projector.h"
#include "wavemap/core/integrator/projective/coarse_to_fine/range_image_intersector.h"
#include "wavemap/core/integrator/projective/projective_integrator.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
class RangeImage2DIntersectorTest : public FixtureBase,
                                    public GeometryGenerator {
 protected:
  ProjectiveIntegratorConfig getRandomPointcloudIntegratorConfig() {
    const FloatingPoint min_range = getRandomSignedDistance(1e-1f, 3.f);
    const FloatingPoint max_range = getRandomSignedDistance(min_range, 40.f);
    return ProjectiveIntegratorConfig{min_range, max_range};
  }

  SphericalProjectorConfig getRandomProjectionModelConfig() {
    SphericalProjectorConfig config;
    config.elevation.min_angle = getRandomAngle(-kQuarterPi, 0.f);
    config.elevation.max_angle =
        getRandomAngle(config.elevation.min_angle, kQuarterPi);
    config.elevation.num_cells = int_math::exp2(getRandomIndexElement(4, 7));
    config.azimuth.min_angle = -kPi;
    config.azimuth.max_angle = kPi;
    config.azimuth.num_cells = int_math::exp2(getRandomIndexElement(7, 11));
    return config;
  }

  ContinuousBeamConfig getRandomMeasurementModelConfig(
      const SphericalProjector& projection_model) {
    ContinuousBeamConfig config;
    const FloatingPoint max_angle_sigma_without_overlap =
        (projection_model.getMaxImageCoordinates() -
         projection_model.getMinImageCoordinates())
            .cwiseQuotient(
                projection_model.getDimensions().cast<FloatingPoint>())
            .minCoeff() /
        (2.f * 6.f);
    config.angle_sigma = getRandomFloat(max_angle_sigma_without_overlap / 10.f,
                                        max_angle_sigma_without_overlap);
    config.range_sigma = getRandomFloat(1e-3f, 5e-2f);
    return config;
  }

  PosedImage<> getRandomPosedRangeImage(IndexElement num_rows,
                                        IndexElement num_cols,
                                        FloatingPoint min_range,
                                        FloatingPoint max_range) {
    CHECK_LT(min_range, max_range);

    PosedImage<> posed_range_image(num_rows, num_cols);
    for (const Index2D& index :
         Grid<2>(Index2D::Zero(), {num_rows - 1, num_cols - 1})) {
      const FloatingPoint range = getRandomSignedDistance(min_range, max_range);
      posed_range_image.at(index) = range;
    }
    posed_range_image.setPose(getRandomTransformation());

    return posed_range_image;
  }
};

TEST_F(RangeImage2DIntersectorTest, RangeImageUpdateType) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random pointcloud
    const FloatingPoint min_cell_width = getRandomMinCellWidth();
    const auto integrator_config = getRandomPointcloudIntegratorConfig();
    const auto projection_model =
        std::make_shared<SphericalProjector>(getRandomProjectionModelConfig());
    const auto range_image =
        std::make_shared<Image<>>(projection_model->getDimensions());
    const auto beam_offset_image =
        std::make_shared<Image<Vector2D>>(projection_model->getDimensions());
    const auto measurement_model =
        ContinuousBeam(getRandomMeasurementModelConfig(*projection_model),
                       projection_model, range_image, beam_offset_image);
    constexpr FloatingPoint kMaxRange = 60.f;
    const auto posed_range_image =
        std::make_shared<PosedImage<>>(getRandomPosedRangeImage(
            projection_model->getNumRows(), projection_model->getNumColumns(),
            0.f, kMaxRange));

    // Create the hierarchical range image
    RangeImageIntersector range_image_intersector(
        posed_range_image, projection_model, measurement_model,
        integrator_config.min_range, integrator_config.max_range);

    const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
    constexpr IndexElement kMaxHeight = 6;
    const Index3D min_index = convert::pointToCeilIndex<3>(
        posed_range_image->getOrigin() - Vector3D::Constant(kMaxRange),
        min_cell_width_inv);
    const Index3D max_index = convert::pointToCeilIndex<3>(
        posed_range_image->getOrigin() + Vector3D::Constant(kMaxRange),
        min_cell_width_inv);
    for (const Index3D& index :
         getRandomIndexVector(min_index, max_index, 50, 100)) {
      const IndexElement height = getRandomNdtreeIndexHeight(2, kMaxHeight);
      const OctreeIndex query_index =
          convert::indexAndHeightToNodeIndex(index, height);
      const Index3D min_reference_index =
          convert::nodeIndexToMinCornerIndex(query_index);
      const Index3D max_reference_index =
          convert::nodeIndexToMaxCornerIndex(query_index);
      bool has_free = false;
      bool has_occupied = false;
      bool has_unobserved = false;
      const Transformation3D T_C_W = posed_range_image->getPoseInverse();
      for (const Index3D& reference_index :
           Grid(min_reference_index, max_reference_index)) {
        const Point3D W_cell_center =
            convert::indexToCenterPoint(reference_index, min_cell_width);
        const Point3D C_cell_center = T_C_W * W_cell_center;
        const FloatingPoint d_C_cell = C_cell_center.norm();
        if (integrator_config.max_range < d_C_cell) {
          has_unobserved = true;
          continue;
        }

        const Index2D range_image_index =
            projection_model->cartesianToNearestIndex(C_cell_center);
        if ((range_image_index.array() < 0).any() ||
            (posed_range_image->getDimensions().array() <=
             range_image_index.array())
                .any()) {
          has_unobserved = true;
          continue;
        }

        const FloatingPoint range_image_distance =
            posed_range_image->at(range_image_index);
        if (d_C_cell <
            range_image_distance - measurement_model.getPaddingSurfaceFront()) {
          has_free = true;
        } else if (d_C_cell <= range_image_distance +
                                   measurement_model.getPaddingSurfaceBack()) {
          has_occupied = true;
        } else {
          has_unobserved = true;
        }
      }
      ASSERT_TRUE(has_free || has_occupied || has_unobserved);
      UpdateType reference_update_type;
      if (has_occupied) {
        reference_update_type = UpdateType::kPossiblyOccupied;
      } else if (has_free && has_unobserved) {
        reference_update_type = UpdateType::kFreeOrUnobserved;
      } else if (has_free) {
        reference_update_type = UpdateType::kFullyFree;
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
              posed_range_image->getOrigin());
      EXPECT_TRUE(reference_update_type.toTypeId() <=
                  returned_update_type.toTypeId())
          << "Expected " << reference_update_type.toStr() << " but got "
          << returned_update_type.toStr();
    }
  }
}
}  // namespace wavemap

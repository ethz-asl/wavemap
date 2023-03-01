#include <unordered_set>

#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/data_structure/volumetric/hashed_blocks.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/data_structure/volumetric/volumetric_octree.h"
#include "wavemap/data_structure/volumetric/wavelet_octree.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/integrator/integrator_base.h"
#include "wavemap/integrator/projection_model/spherical_projector.h"
#include "wavemap/integrator/projective/coarse_to_fine/coarse_to_fine_integrator.h"
#include "wavemap/integrator/projective/coarse_to_fine/wavelet_integrator.h"
#include "wavemap/integrator/projective/fixed_resolution/fixed_resolution_integrator.h"
#include "wavemap/integrator/ray_tracing/ray_tracing_integrator.h"
#include "wavemap/iterator/grid_iterator.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/utils/eigen_format.h"

namespace wavemap {
class PointcloudIntegrator3DTest : public FixtureBase {
 protected:
  RayTracingIntegratorConfig getRandomRayTracingIntegratorConfig() {
    RayTracingIntegratorConfig config;
    config.min_range = getRandomSignedDistance(0.2f, 3.f);
    config.max_range = getRandomSignedDistance(config.min_range, 20.f);
    return config;
  }

  ProjectiveIntegratorConfig getRandomProjectiveIntegratorConfig() {
    ProjectiveIntegratorConfig config;
    config.min_range = getRandomSignedDistance(0.2f, 3.f);
    config.max_range = getRandomSignedDistance(config.min_range, 20.f);
    return config;
  }

  VolumetricDataStructureConfig getRandomVolumetricDataStructureConfig() {
    const FloatingPoint min_cell_width = getRandomMinCellWidth(0.05f, 1.f);
    return VolumetricDataStructureConfig{min_cell_width};
  }

  std::shared_ptr<SphericalProjector> getRandomProjectionModel() {
    const FloatingPoint min_elevation_angle = getRandomAngle(-kQuarterPi, 0.f);
    const FloatingPoint max_elevation_angle =
        getRandomAngle(min_elevation_angle + kPi / 8.f, kQuarterPi);
    const FloatingPoint min_azimuth_angle = -kPi;
    const FloatingPoint max_azimuth_angle = kPi;
    const int num_rows = int_math::exp2(getRandomIndexElement(4, 6));
    const int num_cols = int_math::exp2(getRandomIndexElement(7, 10));
    return std::make_shared<SphericalProjector>(SphericalProjectorConfig{
        {min_elevation_angle, max_elevation_angle, num_rows},
        {min_azimuth_angle, max_azimuth_angle, num_cols}});
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
    config.angle_sigma = random_number_generator_->getRandomRealNumber(
        max_angle_sigma_without_overlap / 10.f,
        max_angle_sigma_without_overlap);
    config.range_sigma =
        random_number_generator_->getRandomRealNumber(5e-3f, 1e-1f);
    return config;
  }

  PosedPointcloud<> getRandomPointcloud(
      const SphericalProjector& projection_model,
      FloatingPoint min_distance = 0.f,
      FloatingPoint max_distance = 30.f) const {
    CHECK_LT(min_distance, max_distance);

    Pointcloud<> pointcloud;
    pointcloud.resize(projection_model.getNumRows() *
                      projection_model.getNumColumns());

    for (int pointcloud_index = 0;
         pointcloud_index < static_cast<int>(pointcloud.size());
         ++pointcloud_index) {
      const FloatingPoint range =
          getRandomSignedDistance(min_distance, max_distance);
      const Index2D image_index{
          pointcloud_index % projection_model.getNumRows(),
          pointcloud_index / projection_model.getNumRows()};
      pointcloud[pointcloud_index] =
          range * projection_model.sensorToCartesian(
                      projection_model.indexToImage(image_index), 1.f);
    }

    return PosedPointcloud<>(getRandomTransformation<3>(), pointcloud);
  }
};

TEST_F(PointcloudIntegrator3DTest, RayTracingIntegrator) {
  for (int idx = 0; idx < 3; ++idx) {
    const auto ray_tracing_integrator_config =
        getRandomRayTracingIntegratorConfig();
    const auto data_structure_config = getRandomVolumetricDataStructureConfig();
    const auto projection_model = getRandomProjectionModel();
    const FloatingPoint min_cell_width_inv =
        1.f / data_structure_config.min_cell_width;

    // Set the min distance s.t. no cells are updated by more than 1 beam
    // NOTE: Otherwise one beam's free space update could erase another beam's
    //       endpoint and this situation is not covered in this unit test.
    constexpr FloatingPoint kMaxDistance = 200.f;
    const FloatingPoint min_distance =
        data_structure_config.min_cell_width *
        projection_model->getDimensions()
            .cast<FloatingPoint>()
            .cwiseQuotient(projection_model->getMaxImageCoordinates() -
                           projection_model->getMinImageCoordinates())
            .maxCoeff();
    if (kMaxDistance <= min_distance) {
      --idx;
      continue;
    }

    // Generate a random point cloud and save its end points in a hashed set
    const PosedPointcloud<> random_pointcloud =
        getRandomPointcloud(*projection_model, min_distance, kMaxDistance);
    std::unordered_set<Index3D, VoxbloxIndexHash<3>> ray_end_points;
    for (const auto& end_point : random_pointcloud.getPointsGlobal()) {
      const Index3D index =
          convert::pointToNearestIndex<3>(end_point, min_cell_width_inv);
      ray_end_points.emplace(index);
    }

    // Set up the occupancy map, integrator and integrate the point cloud
    VolumetricDataStructureBase::Ptr occupancy_map =
        std::make_shared<HashedBlocks>(data_structure_config);
    IntegratorBase::Ptr pointcloud_integrator =
        std::make_shared<RayTracingIntegrator>(ray_tracing_integrator_config,
                                               occupancy_map);
    pointcloud_integrator->integratePointcloud(random_pointcloud);

    // Check the map
    // NOTE: This test may generate false positives if the point cloud contains
    //       overlapping rays which erase each other's endpoints. This is most
    //       likely to happen when the map resolution is low w.r.t. the spacing
    //       between the point cloud's beams.
    for (const Index3D& index :
         Grid(occupancy_map->getMinIndex(), occupancy_map->getMaxIndex())) {
      const bool cell_occupied_in_map =
          (0.f < occupancy_map->getCellValue(index));
      const bool cell_contains_ray_end_point = ray_end_points.count(index);
      EXPECT_EQ(cell_occupied_in_map, cell_contains_ray_end_point)
          << "for index " << EigenFormat::oneLine(index) << ", min cell width "
          << data_structure_config.min_cell_width << " and point cloud size "
          << random_pointcloud.getPointsLocal().size();
    }
  }
}

TEST_F(PointcloudIntegrator3DTest,
       FixedResolutionAndCoarseToFineIntegratorEquivalence) {
  constexpr int kNumRepetitions = 3;
  for (int idx = 0; idx < kNumRepetitions; ++idx) {
    const auto projective_integrator_config =
        getRandomProjectiveIntegratorConfig();
    const auto data_structure_config = getRandomVolumetricDataStructureConfig();
    const auto projection_model = getRandomProjectionModel();
    const auto posed_range_image =
        std::make_shared<PosedImage<>>(projection_model->getDimensions());
    const auto beam_offset_image =
        std::make_shared<Image<Vector2D>>(projection_model->getDimensions());
    const auto measurement_model = std::make_shared<ContinuousBeam>(
        getRandomMeasurementModelConfig(*projection_model), projection_model,
        posed_range_image, beam_offset_image);
    const PosedPointcloud<> random_pointcloud =
        getRandomPointcloud(*projection_model);

    VolumetricDataStructureBase::Ptr reference_occupancy_map =
        std::make_shared<HashedBlocks>(data_structure_config);
    IntegratorBase::Ptr reference_integrator =
        std::make_shared<FixedResolutionIntegrator>(
            projective_integrator_config, projection_model, posed_range_image,
            beam_offset_image, measurement_model, reference_occupancy_map);
    reference_integrator->integratePointcloud(random_pointcloud);

    VolumetricOctree::Ptr evaluated_occupancy_map =
        std::make_shared<VolumetricOctree>(data_structure_config);
    IntegratorBase::Ptr evaluated_integrator =
        std::make_shared<CoarseToFineIntegrator>(
            projective_integrator_config, projection_model, posed_range_image,
            beam_offset_image, measurement_model, evaluated_occupancy_map);
    evaluated_integrator->integratePointcloud(random_pointcloud);

    evaluated_occupancy_map->prune();
    const Index3D min_index = reference_occupancy_map->getMinIndex().cwiseMin(
        evaluated_occupancy_map->getMinIndex());
    const Index3D max_index = reference_occupancy_map->getMaxIndex().cwiseMax(
        evaluated_occupancy_map->getMaxIndex());

    for (const Index3D& index : Grid(min_index, max_index)) {
      const FloatingPoint cell_value_in_reference_map =
          reference_occupancy_map->getCellValue(index);
      const FloatingPoint cell_value_in_evaluated_map =
          evaluated_occupancy_map->getCellValue(index);
      EXPECT_NEAR(cell_value_in_evaluated_map, cell_value_in_reference_map,
                  projective_integrator_config.termination_update_error)
          << "For cell index " << EigenFormat::oneLine(index);
    }
  }
}

TEST_F(PointcloudIntegrator3DTest,
       FixedResolutionAndWaveletIntegratorEquivalence) {
  constexpr int kNumRepetitions = 3;
  for (int idx = 0; idx < kNumRepetitions; ++idx) {
    const auto projective_integrator_config =
        getRandomProjectiveIntegratorConfig();
    const auto data_structure_config = getRandomVolumetricDataStructureConfig();
    const auto projection_model = getRandomProjectionModel();
    const auto posed_range_image =
        std::make_shared<PosedImage<>>(projection_model->getDimensions());
    const auto beam_offset_image =
        std::make_shared<Image<Vector2D>>(projection_model->getDimensions());
    const auto measurement_model = std::make_shared<ContinuousBeam>(
        getRandomMeasurementModelConfig(*projection_model), projection_model,
        posed_range_image, beam_offset_image);
    const PosedPointcloud<> random_pointcloud =
        getRandomPointcloud(*projection_model);

    VolumetricDataStructureBase::Ptr reference_occupancy_map =
        std::make_shared<HashedBlocks>(data_structure_config);
    IntegratorBase::Ptr reference_integrator =
        std::make_shared<FixedResolutionIntegrator>(
            projective_integrator_config, projection_model, posed_range_image,
            beam_offset_image, measurement_model, reference_occupancy_map);
    reference_integrator->integratePointcloud(random_pointcloud);

    WaveletOctree::Ptr evaluated_occupancy_map =
        std::make_shared<WaveletOctree>(data_structure_config);
    IntegratorBase::Ptr evaluated_integrator =
        std::make_shared<WaveletIntegrator>(
            projective_integrator_config, projection_model, posed_range_image,
            beam_offset_image, measurement_model, evaluated_occupancy_map);
    evaluated_integrator->integratePointcloud(random_pointcloud);

    evaluated_occupancy_map->prune();
    const Index3D min_index = reference_occupancy_map->getMinIndex().cwiseMin(
        evaluated_occupancy_map->getMinIndex());
    const Index3D max_index = reference_occupancy_map->getMaxIndex().cwiseMax(
        evaluated_occupancy_map->getMaxIndex());

    for (const Index3D& index : Grid(min_index, max_index)) {
      const FloatingPoint cell_value_in_reference_map =
          reference_occupancy_map->getCellValue(index);
      const FloatingPoint cell_value_in_evaluated_map =
          evaluated_occupancy_map->getCellValue(index);
      EXPECT_NEAR(cell_value_in_evaluated_map, cell_value_in_reference_map,
                  projective_integrator_config.termination_update_error)
          << "For cell index " << EigenFormat::oneLine(index);
    }
  }
}
}  // namespace wavemap
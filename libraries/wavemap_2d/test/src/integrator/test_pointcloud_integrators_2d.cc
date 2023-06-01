#include <unordered_set>

#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>
#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/indexing/index_hashes.h>
#include <wavemap_common/iterator/grid_iterator.h>
#include <wavemap_common/test/fixture_base.h>
#include <wavemap_common/utils/eigen_format.h>

#include "wavemap_2d/data_structure/dense_grid.h"
#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"
#include "wavemap_2d/data_structure/volumetric_quadtree.h"
#include "wavemap_2d/data_structure/wavelet_quadtree.h"
#include "wavemap_2d/integrator/pointcloud_integrator_2d.h"
#include "wavemap_2d/integrator/projective/beamwise_integrator_2d.h"
#include "wavemap_2d/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_2d.h"
#include "wavemap_2d/integrator/projective/coarse_to_fine/wavelet_integrator_2d.h"
#include "wavemap_2d/integrator/projective/fixed_resolution/fixed_resolution_integrator_2d.h"
#include "wavemap_2d/integrator/ray_tracing/ray_integrator_2d.h"

namespace wavemap {
class PointcloudIntegrator2DTest : public FixtureBase {
 protected:
  PointcloudIntegratorConfig getRandomPointcloudIntegratorConfig() {
    const FloatingPoint min_range = getRandomSignedDistance(1e-2f, 3.f);
    const FloatingPoint max_range = getRandomSignedDistance(min_range, 60.f);
    return PointcloudIntegratorConfig{min_range, max_range};
  }

  VolumetricDataStructureConfig getRandomVolumetricDataStructureConfig() {
    const FloatingPoint min_cell_width = getRandomMinCellWidth(0.02f, 0.5f);
    return VolumetricDataStructureConfig{min_cell_width};
  }

  CircularProjector getRandomProjectionModel() {
    const FloatingPoint min_angle = getRandomAngle(-kPi, 0.f);
    const FloatingPoint max_angle = getRandomAngle(min_angle, kPi);
    const IndexElement num_beams = getRandomIndexElement(200, 2048);
    return CircularProjector(
        CircularProjectorConfig{min_angle, max_angle, num_beams});
  }

  ContinuousVolumetricLogOdds<2> getRandomMeasurementModel(
      const CircularProjector& projection_model) {
    ContinuousVolumetricLogOddsConfig measurement_model_config;
    const FloatingPoint max_angle_sigma_without_overlap =
        (projection_model.getMaxAngle() - projection_model.getMinAngle()) /
        static_cast<FloatingPoint>(projection_model.getNumCells()) /
        (2.f * 6.f);
    measurement_model_config.angle_sigma =
        random_number_generator_->getRandomRealNumber(
            max_angle_sigma_without_overlap / 10.f,
            max_angle_sigma_without_overlap);
    measurement_model_config.range_sigma =
        random_number_generator_->getRandomRealNumber(1e-3f, 5e-2f);
    return ContinuousVolumetricLogOdds<2>(measurement_model_config);
  }

  PosedPointcloud<Point2D> getRandomPointcloud(
      const CircularProjector& projection_model,
      FloatingPoint min_distance = 0.f,
      FloatingPoint max_distance = 30.f) const {
    CHECK_LT(min_distance, max_distance);

    Pointcloud<Point2D> pointcloud;
    pointcloud.resize(projection_model.getNumCells());

    for (int index = 0; index < projection_model.getNumCells(); ++index) {
      const FloatingPoint range =
          getRandomSignedDistance(min_distance, max_distance);
      pointcloud[index] = range * projection_model.indexToBearing(index);
    }

    return {Transformation2D(), pointcloud};
  }
};

TEST_F(PointcloudIntegrator2DTest, RayIntegrator) {
  for (int idx = 0; idx < 3; ++idx) {
    const auto pointcloud_integrator_config =
        getRandomPointcloudIntegratorConfig();
    const auto data_structure_config = getRandomVolumetricDataStructureConfig();
    const auto projection_model = getRandomProjectionModel();
    const FloatingPoint min_cell_width_inv =
        1.f / data_structure_config.min_cell_width;

    // Set the min distance s.t. no cells are updated by more than 1 beam
    // NOTE: Otherwise one beam's free space update could erase another beam's
    //       endpoint and this situation is not covered in this unit test.
    constexpr FloatingPoint kMaxDistance = 400.f;
    const FloatingPoint min_distance =
        data_structure_config.min_cell_width *
        static_cast<FloatingPoint>(projection_model.getNumCells()) /
        (projection_model.getMaxAngle() - projection_model.getMinAngle());
    if (kMaxDistance <= min_distance) {
      --idx;
      continue;
    }

    // Generate a random pointcloud and save its endpoints in a hashed set
    const auto random_pointcloud =
        getRandomPointcloud(projection_model, min_distance, kMaxDistance);
    std::unordered_set<Index2D, VoxbloxIndexHash<2>> ray_end_points;
    for (const auto& end_point : random_pointcloud.getPointsGlobal()) {
      const Index2D index =
          convert::pointToNearestIndex<2>(end_point, min_cell_width_inv);
      ray_end_points.emplace(index);
    }

    // Setup the occupancy map, integrator and integrate the point cloud
    VolumetricDataStructure2D::Ptr occupancy_map =
        std::make_shared<DenseGrid<SaturatingOccupancyCell>>(
            data_structure_config);
    PointcloudIntegrator2D::Ptr pointcloud_integrator =
        std::make_shared<RayIntegrator2D>(pointcloud_integrator_config,
                                          occupancy_map);
    pointcloud_integrator->integratePointcloud(random_pointcloud);

    // Check the map
    // NOTE: This test may generate false positives if the point cloud contains
    //       overlapping rays which erase each other's endpoints. This is most
    //       likely to happen when the map resolution is low w.r.t. the spacing
    //       between the point cloud's beams.
    for (const Index2D& index :
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

TEST_F(PointcloudIntegrator2DTest,
       BeamAndFixedResolutionIntegratorEquivalence) {
  constexpr bool kShowVisuals = false;
  constexpr int kNumRepetitions = 10;
  for (int idx = 0; idx < kNumRepetitions; ++idx) {
    const auto pointcloud_integrator_config =
        getRandomPointcloudIntegratorConfig();
    const auto data_structure_config = getRandomVolumetricDataStructureConfig();
    const auto projection_model = getRandomProjectionModel();
    const auto measurement_model = getRandomMeasurementModel(projection_model);
    const auto random_pointcloud = getRandomPointcloud(projection_model);

    VolumetricDataStructure2D::Ptr beam_occupancy_map =
        std::make_shared<DenseGrid<UnboundedOccupancyCell>>(
            data_structure_config);
    PointcloudIntegrator2D::Ptr beam_integrator =
        std::make_shared<BeamwiseIntegrator2D>(pointcloud_integrator_config,
                                               measurement_model,
                                               beam_occupancy_map);
    beam_integrator->integratePointcloud(random_pointcloud);
    if (kShowVisuals) {
      beam_occupancy_map->showImage(true, 1000);
    }

    VolumetricDataStructure2D::Ptr scan_occupancy_map =
        std::make_shared<DenseGrid<UnboundedOccupancyCell>>(
            data_structure_config);
    PointcloudIntegrator2D::Ptr scan_integrator =
        std::make_shared<FixedResolutionIntegrator2D>(
            pointcloud_integrator_config, projection_model, measurement_model,
            scan_occupancy_map);
    scan_integrator->integratePointcloud(random_pointcloud);
    if (kShowVisuals) {
      scan_occupancy_map->showImage(true, 1000);
    }

    const Index2D min_index = beam_occupancy_map->getMinIndex().cwiseMin(
        scan_occupancy_map->getMinIndex());
    const Index2D max_index = beam_occupancy_map->getMaxIndex().cwiseMax(
        scan_occupancy_map->getMaxIndex());

    constexpr FloatingPoint kTolerableError = 5e-2f;
    VolumetricDataStructure2D::Ptr error_grid;
    if (kShowVisuals) {
      error_grid = std::make_shared<DenseGrid<SaturatingOccupancyCell>>(
          data_structure_config);
    }
    for (const Index2D& index : Grid(min_index, max_index)) {
      const FloatingPoint cell_value_in_beam_map =
          beam_occupancy_map->getCellValue(index);
      const FloatingPoint cell_value_in_scan_map =
          scan_occupancy_map->getCellValue(index);
      EXPECT_NEAR(cell_value_in_scan_map, cell_value_in_beam_map,
                  kTolerableError);
      if (error_grid) {
        error_grid->setCellValue(
            index, (cell_value_in_scan_map - cell_value_in_beam_map) /
                       kTolerableError);
      }
    }
    if (error_grid) {
      error_grid->showImage(true, 2000);
    }
  }
}

TEST_F(PointcloudIntegrator2DTest, BeamAndCoarseToFineIntegratorEquivalence) {
  constexpr bool kShowVisuals = false;
  constexpr int kNumRepetitions = 10;
  for (int idx = 0; idx < kNumRepetitions; ++idx) {
    const auto pointcloud_integrator_config =
        getRandomPointcloudIntegratorConfig();
    const auto data_structure_config = getRandomVolumetricDataStructureConfig();
    const auto projection_model = getRandomProjectionModel();
    const auto measurement_model = getRandomMeasurementModel(projection_model);
    const auto random_pointcloud = getRandomPointcloud(projection_model);

    VolumetricDataStructure2D::Ptr beam_occupancy_map =
        std::make_shared<DenseGrid<UnboundedOccupancyCell>>(
            data_structure_config);
    PointcloudIntegrator2D::Ptr beam_integrator =
        std::make_shared<BeamwiseIntegrator2D>(pointcloud_integrator_config,
                                               measurement_model,
                                               beam_occupancy_map);
    beam_integrator->integratePointcloud(random_pointcloud);
    if (kShowVisuals) {
      beam_occupancy_map->showImage(true, 1000);
    }

    VolumetricDataStructure2D::Ptr scan_occupancy_map =
        std::make_shared<VolumetricQuadtree<UnboundedOccupancyCell>>(
            data_structure_config);
    PointcloudIntegrator2D::Ptr scan_integrator =
        std::make_shared<CoarseToFineIntegrator2D>(
            pointcloud_integrator_config, projection_model, measurement_model,
            scan_occupancy_map);
    scan_integrator->integratePointcloud(random_pointcloud);
    if (kShowVisuals) {
      scan_occupancy_map->showImage(true, 1000);
    }

    scan_occupancy_map->prune();
    const Index2D min_index = beam_occupancy_map->getMinIndex().cwiseMin(
        scan_occupancy_map->getMinIndex());
    const Index2D max_index = beam_occupancy_map->getMaxIndex().cwiseMax(
        scan_occupancy_map->getMaxIndex());

    VolumetricDataStructure2D::Ptr error_grid;
    if (kShowVisuals) {
      error_grid = std::make_shared<DenseGrid<SaturatingOccupancyCell>>(
          data_structure_config);
    }
    for (const Index2D& index : Grid(min_index, max_index)) {
      const FloatingPoint cell_value_in_beam_map =
          beam_occupancy_map->getCellValue(index);
      const FloatingPoint cell_value_in_scan_map =
          scan_occupancy_map->getCellValue(index);
      EXPECT_NEAR(cell_value_in_scan_map, cell_value_in_beam_map,
                  CoarseToFineIntegrator2D::kMaxAcceptableUpdateError);
      if (error_grid) {
        error_grid->setCellValue(
            index, (cell_value_in_scan_map - cell_value_in_beam_map) /
                       CoarseToFineIntegrator2D::kMaxAcceptableUpdateError);
      }
    }
    if (error_grid) {
      error_grid->showImage(true, 2000);
    }
  }
}

TEST_F(PointcloudIntegrator2DTest, BeamAndWaveletIntegratorEquivalence) {
  constexpr bool kShowVisuals = false;
  constexpr int kNumRepetitions = 10;
  for (int idx = 0; idx < kNumRepetitions; ++idx) {
    const auto pointcloud_integrator_config =
        getRandomPointcloudIntegratorConfig();
    const auto data_structure_config = getRandomVolumetricDataStructureConfig();
    const auto projection_model = getRandomProjectionModel();
    const auto measurement_model = getRandomMeasurementModel(projection_model);
    const auto random_pointcloud = getRandomPointcloud(projection_model);

    VolumetricDataStructure2D::Ptr beam_occupancy_map =
        std::make_shared<DenseGrid<UnboundedOccupancyCell>>(
            data_structure_config);
    PointcloudIntegrator2D::Ptr beam_integrator =
        std::make_shared<BeamwiseIntegrator2D>(pointcloud_integrator_config,
                                               measurement_model,
                                               beam_occupancy_map);
    beam_integrator->integratePointcloud(random_pointcloud);
    if (kShowVisuals) {
      beam_occupancy_map->showImage(true, 1000);
    }

    VolumetricDataStructure2D::Ptr scan_occupancy_map =
        std::make_shared<WaveletQuadtree<UnboundedOccupancyCell>>(
            data_structure_config);
    PointcloudIntegrator2D::Ptr scan_integrator =
        std::make_shared<WaveletIntegrator2D>(
            pointcloud_integrator_config, projection_model, measurement_model,
            scan_occupancy_map);
    scan_integrator->integratePointcloud(random_pointcloud);
    if (kShowVisuals) {
      scan_occupancy_map->showImage(true, 1000);
    }

    scan_occupancy_map->prune();
    const Index2D min_index = beam_occupancy_map->getMinIndex().cwiseMin(
        scan_occupancy_map->getMinIndex());
    const Index2D max_index = beam_occupancy_map->getMaxIndex().cwiseMax(
        scan_occupancy_map->getMaxIndex());

    VolumetricDataStructure2D::Ptr error_grid;
    if (kShowVisuals) {
      error_grid = std::make_shared<DenseGrid<SaturatingOccupancyCell>>(
          data_structure_config);
    }
    for (const Index2D& index : Grid(min_index, max_index)) {
      const FloatingPoint cell_value_in_beam_map =
          beam_occupancy_map->getCellValue(index);
      const FloatingPoint cell_value_in_scan_map =
          scan_occupancy_map->getCellValue(index);
      EXPECT_NEAR(cell_value_in_scan_map, cell_value_in_beam_map,
                  WaveletIntegrator2D::kMaxAcceptableUpdateError);
      if (error_grid) {
        error_grid->setCellValue(
            index, (cell_value_in_scan_map - cell_value_in_beam_map) /
                       WaveletIntegrator2D::kMaxAcceptableUpdateError);
      }
    }
    if (error_grid) {
      error_grid->showImage(true, 2000);
    }
  }
}
}  // namespace wavemap

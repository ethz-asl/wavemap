#include <unordered_set>

#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>
#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/test/fixture_base.h>
#include <wavemap_common/utils/eigen_format.h>

#include "wavemap_2d/data_structure/dense_grid.h"
#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"
#include "wavemap_2d/data_structure/volumetric_quadtree.h"
#include "wavemap_2d/data_structure/wavelet_tree.h"
#include "wavemap_2d/indexing/index_hashes.h"
#include "wavemap_2d/integrator/point_integrator/beam_integrator.h"
#include "wavemap_2d/integrator/point_integrator/ray_integrator.h"
#include "wavemap_2d/integrator/pointcloud_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/coarse_to_fine_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/wavelet_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/fixed_resolution/fixed_resolution_integrator.h"
#include "wavemap_2d/iterator/grid_iterator.h"

namespace wavemap {
class PointcloudIntegratorTest : public FixtureBase {
 protected:
  PosedPointcloud<Point2D, Transformation2D> getRandomPointcloud(
      FloatingPoint min_angle, FloatingPoint max_angle, int num_beams,
      FloatingPoint min_distance, FloatingPoint max_distance) const {
    CHECK_LT(min_angle, max_angle);
    CHECK_LT(min_distance, max_distance);

    const FloatingPoint angle_increment =
        (max_angle - min_angle) / static_cast<FloatingPoint>(num_beams - 1);

    Pointcloud<Point2D> pointcloud;
    pointcloud.resize(num_beams);
    for (int index = 0; index < num_beams; ++index) {
      const FloatingPoint range =
          getRandomSignedDistance(min_distance, max_distance);
      const FloatingPoint angle =
          min_angle + static_cast<FloatingPoint>(index) * angle_increment;

      pointcloud[index] = range * RangeImage::angleToBearing(angle);
    }

    return {Transformation2D(), pointcloud};
  }
};

TEST_F(PointcloudIntegratorTest, RayIntegrator) {
  for (int idx = 0; idx < 3; ++idx) {
    const FloatingPoint min_cell_width = getRandomMinCellWidth();
    const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;

    const FloatingPoint min_angle = -kPi;
    const FloatingPoint max_angle = kPi;
    const int num_beams = getRandomIndexElement(10, 100);
    const FloatingPoint min_distance = static_cast<FloatingPoint>(num_beams) *
                                       min_cell_width / (max_angle - min_angle);
    constexpr FloatingPoint kMaxDistance = 400.f;

    // Generate a random point cloud and save its end points in a hashed set
    const PosedPointcloud<Point2D, Transformation2D> random_pointcloud =
        getRandomPointcloud(min_angle, max_angle, num_beams, min_distance,
                            kMaxDistance);
    std::unordered_set<Index2D, VoxbloxIndexHash> ray_end_points;
    for (const auto& end_point : random_pointcloud.getPointsGlobal()) {
      const Index2D index =
          convert::pointToNearestIndex<2>(end_point, min_cell_width_inv);
      ray_end_points.emplace(index);
    }

    // Setup the occupancy map, integrator and integrate the point cloud
    VolumetricDataStructure2D::Ptr occupancy_map =
        std::make_shared<DenseGrid<SaturatingOccupancyCell>>(min_cell_width);
    PointcloudIntegrator::Ptr pointcloud_integrator =
        std::make_shared<RayIntegrator>(occupancy_map);
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
          << min_cell_width << " and point cloud size "
          << random_pointcloud.getPointsLocal().size();
    }
  }
}

TEST_F(PointcloudIntegratorTest, BeamAndFixedResolutionIntegratorEquivalence) {
  constexpr bool kShowVisuals = false;
  for (int idx = 0; idx < 10; ++idx) {
    const FloatingPoint min_cell_width = getRandomMinCellWidth(0.02f, 0.5f);
    // TODO(victorr): Use random FoVs and numbers of beams once these are
    //                configurable
    constexpr FloatingPoint kMinAngle = -kHalfPi;
    constexpr FloatingPoint kMaxAngle = kHalfPi;
    constexpr int kNumBeams = 400;
    constexpr FloatingPoint kMinDistance = 0.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    const PosedPointcloud<Point2D, Transformation2D> random_pointcloud =
        getRandomPointcloud(kMinAngle, kMaxAngle, kNumBeams, kMinDistance,
                            kMaxDistance);

    VolumetricDataStructure2D::Ptr beam_occupancy_map =
        std::make_shared<DenseGrid<UnboundedOccupancyCell>>(min_cell_width);
    PointcloudIntegrator::Ptr beam_integrator =
        std::make_shared<BeamIntegrator>(beam_occupancy_map);
    beam_integrator->integratePointcloud(random_pointcloud);
    if (kShowVisuals) {
      beam_occupancy_map->showImage(true, 1000);
    }

    VolumetricDataStructure2D::Ptr scan_occupancy_map =
        std::make_shared<DenseGrid<UnboundedOccupancyCell>>(min_cell_width);
    PointcloudIntegrator::Ptr scan_integrator =
        std::make_shared<FixedResolutionIntegrator>(scan_occupancy_map);
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
      error_grid =
          std::make_shared<DenseGrid<SaturatingOccupancyCell>>(min_cell_width);
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

TEST_F(PointcloudIntegratorTest, BeamAndCoarseToFineIntegratorEquivalence) {
  constexpr bool kShowVisuals = false;
  for (int idx = 0; idx < 10; ++idx) {
    const FloatingPoint min_cell_width = getRandomMinCellWidth(0.02f, 0.5f);
    // TODO(victorr): Use random FoVs and numbers of beams once these are
    //                configurable
    constexpr FloatingPoint kMinAngle = -kHalfPi;
    constexpr FloatingPoint kMaxAngle = kHalfPi;
    constexpr int kNumBeams = 400;
    constexpr FloatingPoint kMinDistance = 0.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    const PosedPointcloud<Point2D, Transformation2D> random_pointcloud =
        getRandomPointcloud(kMinAngle, kMaxAngle, kNumBeams, kMinDistance,
                            kMaxDistance);

    VolumetricDataStructure2D::Ptr beam_occupancy_map =
        std::make_shared<DenseGrid<UnboundedOccupancyCell>>(min_cell_width);
    PointcloudIntegrator::Ptr beam_integrator =
        std::make_shared<BeamIntegrator>(beam_occupancy_map);
    beam_integrator->integratePointcloud(random_pointcloud);
    if (kShowVisuals) {
      beam_occupancy_map->showImage(true, 1000);
    }

    VolumetricDataStructure2D::Ptr scan_occupancy_map =
        std::make_shared<VolumetricQuadtree<UnboundedOccupancyCell>>(
            min_cell_width);
    PointcloudIntegrator::Ptr scan_integrator =
        std::make_shared<CoarseToFineIntegrator>(scan_occupancy_map);
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
      error_grid =
          std::make_shared<DenseGrid<SaturatingOccupancyCell>>(min_cell_width);
    }
    for (const Index2D& index : Grid(min_index, max_index)) {
      const FloatingPoint cell_value_in_beam_map =
          beam_occupancy_map->getCellValue(index);
      const FloatingPoint cell_value_in_scan_map =
          scan_occupancy_map->getCellValue(index);
      EXPECT_NEAR(cell_value_in_scan_map, cell_value_in_beam_map,
                  CoarseToFineIntegrator::kMaxAcceptableUpdateError);
      if (error_grid) {
        error_grid->setCellValue(
            index, (cell_value_in_scan_map - cell_value_in_beam_map) /
                       CoarseToFineIntegrator::kMaxAcceptableUpdateError);
      }
    }
    if (error_grid) {
      error_grid->showImage(true, 2000);
    }
  }
}

TEST_F(PointcloudIntegratorTest, BeamAndWaveletIntegratorEquivalence) {
  constexpr bool kShowVisuals = false;
  for (int idx = 0; idx < 10; ++idx) {
    const FloatingPoint min_cell_width = getRandomMinCellWidth(0.02f, 0.5f);
    // TODO(victorr): Use random FoVs and numbers of beams once these are
    //                configurable
    constexpr FloatingPoint kMinAngle = -kHalfPi;
    constexpr FloatingPoint kMaxAngle = kHalfPi;
    constexpr int kNumBeams = 400;
    constexpr FloatingPoint kMinDistance = 0.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    const PosedPointcloud<Point2D, Transformation2D> random_pointcloud =
        getRandomPointcloud(kMinAngle, kMaxAngle, kNumBeams, kMinDistance,
                            kMaxDistance);

    VolumetricDataStructure2D::Ptr beam_occupancy_map =
        std::make_shared<DenseGrid<UnboundedOccupancyCell>>(min_cell_width);
    PointcloudIntegrator::Ptr beam_integrator =
        std::make_shared<BeamIntegrator>(beam_occupancy_map);
    beam_integrator->integratePointcloud(random_pointcloud);
    if (kShowVisuals) {
      beam_occupancy_map->showImage(true, 1000);
    }

    VolumetricDataStructure2D::Ptr scan_occupancy_map =
        std::make_shared<WaveletTree<UnboundedOccupancyCell>>(min_cell_width);
    PointcloudIntegrator::Ptr scan_integrator =
        std::make_shared<WaveletIntegrator>(scan_occupancy_map);
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
      error_grid =
          std::make_shared<DenseGrid<SaturatingOccupancyCell>>(min_cell_width);
    }
    for (const Index2D& index : Grid(min_index, max_index)) {
      const FloatingPoint cell_value_in_beam_map =
          beam_occupancy_map->getCellValue(index);
      const FloatingPoint cell_value_in_scan_map =
          scan_occupancy_map->getCellValue(index);
      EXPECT_NEAR(cell_value_in_scan_map, cell_value_in_beam_map,
                  CoarseToFineIntegrator::kMaxAcceptableUpdateError);
      if (error_grid) {
        error_grid->setCellValue(
            index, (cell_value_in_scan_map - cell_value_in_beam_map) /
                       CoarseToFineIntegrator::kMaxAcceptableUpdateError);
      }
    }
    if (error_grid) {
      error_grid->showImage(true, 2000);
    }
  }
}
}  // namespace wavemap

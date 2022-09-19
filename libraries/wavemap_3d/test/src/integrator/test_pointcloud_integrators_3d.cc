#include <unordered_set>

#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>
#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/indexing/index_hashes.h>
#include <wavemap_common/integrator/ray_tracing/ray_integrator.h>
#include <wavemap_common/iterator/grid_iterator.h>
#include <wavemap_common/test/fixture_base.h>
#include <wavemap_common/utils/eigen_format.h>

#include "wavemap_3d/data_structure/hashed_blocks_3d.h"
#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"
#include "wavemap_3d/data_structure/volumetric_octree.h"
#include "wavemap_3d/data_structure/wavelet_octree.h"
#include "wavemap_3d/integrator/pointcloud_integrator_3d.h"
#include "wavemap_3d/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_3d.h"
#include "wavemap_3d/integrator/projective/coarse_to_fine/wavelet_integrator_3d.h"
#include "wavemap_3d/integrator/projective/fixed_resolution/fixed_resolution_integrator_3d.h"

namespace wavemap {
class PointcloudIntegrator3DTest : public FixtureBase {
 protected:
  PosedPointcloud<Point3D> getRandomPointcloud(
      FloatingPoint min_elevation_angle, FloatingPoint max_elevation_angle,
      int num_rows, FloatingPoint min_azimuth_angle,
      FloatingPoint max_azimuth_angle, int num_cols, FloatingPoint min_distance,
      FloatingPoint max_distance) const {
    CHECK_LT(min_elevation_angle, max_elevation_angle);
    CHECK_LT(min_azimuth_angle, max_azimuth_angle);
    CHECK_LT(min_distance, max_distance);

    Pointcloud<Point3D> pointcloud;
    pointcloud.resize(num_rows * num_cols);

    const SphericalProjector spherical_projector(
        min_elevation_angle, max_elevation_angle, num_rows, min_azimuth_angle,
        max_azimuth_angle, num_cols);
    for (int pointcloud_index = 0;
         pointcloud_index < static_cast<int>(pointcloud.size());
         ++pointcloud_index) {
      const FloatingPoint range =
          getRandomSignedDistance(min_distance, max_distance);
      const Index2D image_index{pointcloud_index % num_rows,
                                pointcloud_index / num_rows};
      pointcloud[pointcloud_index] =
          range * spherical_projector.indexToBearing(image_index);
    }

    return {getRandomTransformation<3>(), pointcloud};
  }
};

TEST_F(PointcloudIntegrator3DTest, RayIntegrator) {
  for (int idx = 0; idx < 3; ++idx) {
    const FloatingPoint min_cell_width = getRandomMinCellWidth();
    const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;

    constexpr FloatingPoint kMinElevationAngle = -kQuarterPi;
    constexpr FloatingPoint kMaxElevationAngle = kQuarterPi;
    constexpr FloatingPoint kMinAzimuthAngle = -kPi;
    constexpr FloatingPoint kMaxAzimuthAngle = kPi;

    const int num_rows = getRandomIndexElement(10, 200);
    const int num_cols = getRandomIndexElement(10, 1000);

    const FloatingPoint min_distance =
        std::max(static_cast<FloatingPoint>(num_rows) * min_cell_width /
                     (kMaxElevationAngle - kMinElevationAngle),
                 static_cast<FloatingPoint>(num_cols) * min_cell_width /
                     (kMaxAzimuthAngle - kMinAzimuthAngle));
    constexpr FloatingPoint kMaxDistance = 200.f;
    CHECK_LE(min_distance, kMaxDistance);

    // Generate a random point cloud and save its end points in a hashed set
    const PosedPointcloud<Point3D> random_pointcloud = getRandomPointcloud(
        kMinElevationAngle, kMaxElevationAngle, num_rows, kMinAzimuthAngle,
        kMaxAzimuthAngle, num_cols, min_distance, kMaxDistance);
    std::unordered_set<Index3D, VoxbloxIndexHash<3>> ray_end_points;
    for (const auto& end_point : random_pointcloud.getPointsGlobal()) {
      const Index3D index =
          convert::pointToNearestIndex<3>(end_point, min_cell_width_inv);
      ray_end_points.emplace(index);
    }

    // Set up the occupancy map, integrator and integrate the point cloud
    VolumetricDataStructure3D::Ptr occupancy_map =
        std::make_shared<HashedBlocks3D<SaturatingOccupancyCell>>(
            min_cell_width);
    PointcloudIntegrator3D::Ptr pointcloud_integrator =
        std::make_shared<RayIntegrator<3>>(occupancy_map);
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
          << min_cell_width << " and point cloud size "
          << random_pointcloud.getPointsLocal().size();
    }
  }
}

TEST_F(PointcloudIntegrator3DTest,
       FixedResolutionAndCoarseToFineIntegratorEquivalence) {
  constexpr int kNumRepetitions = 3;
  for (int idx = 0; idx < kNumRepetitions; ++idx) {
    const FloatingPoint min_cell_width = getRandomMinCellWidth(0.02f, 0.5f);
    // TODO(victorr): Use random FoVs and numbers of beams once these are
    //                configurable
    constexpr FloatingPoint kMinElevationAngle = -0.3926991f;
    constexpr FloatingPoint kMaxElevationAngle = 0.3926991f;
    constexpr FloatingPoint kMinAzimuthAngle = -kPi;
    constexpr FloatingPoint kMaxAzimuthAngle = kPi;
    constexpr int kNumRows = 64;
    constexpr int kNumCols = 1024;
    constexpr FloatingPoint kMinDistance = 0.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    const PosedPointcloud<Point3D> random_pointcloud = getRandomPointcloud(
        kMinElevationAngle, kMaxElevationAngle, kNumRows, kMinAzimuthAngle,
        kMaxAzimuthAngle, kNumCols, kMinDistance, kMaxDistance);

    VolumetricDataStructure3D::Ptr beam_occupancy_map =
        std::make_shared<HashedBlocks3D<UnboundedOccupancyCell>>(
            min_cell_width);
    PointcloudIntegrator3D::Ptr fixed_resolution_integrator =
        std::make_shared<FixedResolutionIntegrator3D>(beam_occupancy_map);
    fixed_resolution_integrator->integratePointcloud(random_pointcloud);

    VolumetricDataStructure3D::Ptr scan_occupancy_map =
        std::make_shared<VolumetricOctree<UnboundedOccupancyCell>>(
            min_cell_width);
    PointcloudIntegrator3D::Ptr scan_integrator =
        std::make_shared<CoarseToFineIntegrator3D>(scan_occupancy_map);
    scan_integrator->integratePointcloud(random_pointcloud);

    scan_occupancy_map->prune();
    const Index3D min_index = beam_occupancy_map->getMinIndex().cwiseMin(
        scan_occupancy_map->getMinIndex());
    const Index3D max_index = beam_occupancy_map->getMaxIndex().cwiseMax(
        scan_occupancy_map->getMaxIndex());

    for (const Index3D& index : Grid(min_index, max_index)) {
      const FloatingPoint cell_value_in_beam_map =
          beam_occupancy_map->getCellValue(index);
      const FloatingPoint cell_value_in_scan_map =
          scan_occupancy_map->getCellValue(index);
      EXPECT_NEAR(cell_value_in_scan_map, cell_value_in_beam_map,
                  CoarseToFineIntegrator3D::kMaxAcceptableUpdateError);
    }
  }
}

TEST_F(PointcloudIntegrator3DTest,
       FixedResolutionAndWaveletIntegratorEquivalence) {
  constexpr int kNumRepetitions = 3;
  for (int idx = 0; idx < kNumRepetitions; ++idx) {
    const FloatingPoint min_cell_width = getRandomMinCellWidth(0.02f, 0.5f);
    // TODO(victorr): Use random FoVs and numbers of beams once these are
    //                configurable
    constexpr FloatingPoint kMinElevationAngle = -0.3926991f;
    constexpr FloatingPoint kMaxElevationAngle = 0.3926991f;
    constexpr FloatingPoint kMinAzimuthAngle = -kPi;
    constexpr FloatingPoint kMaxAzimuthAngle = kPi;
    constexpr int kNumRows = 64;
    constexpr int kNumCols = 1024;
    constexpr FloatingPoint kMinDistance = 0.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    const PosedPointcloud<Point3D> random_pointcloud = getRandomPointcloud(
        kMinElevationAngle, kMaxElevationAngle, kNumRows, kMinAzimuthAngle,
        kMaxAzimuthAngle, kNumCols, kMinDistance, kMaxDistance);

    VolumetricDataStructure3D::Ptr beam_occupancy_map =
        std::make_shared<HashedBlocks3D<UnboundedOccupancyCell>>(
            min_cell_width);
    PointcloudIntegrator3D::Ptr fixed_resolution_integrator =
        std::make_shared<FixedResolutionIntegrator3D>(beam_occupancy_map);
    fixed_resolution_integrator->integratePointcloud(random_pointcloud);

    VolumetricDataStructure3D::Ptr scan_occupancy_map =
        std::make_shared<WaveletOctree<UnboundedOccupancyCell>>(min_cell_width);
    PointcloudIntegrator3D::Ptr scan_integrator =
        std::make_shared<WaveletIntegrator3D>(scan_occupancy_map);
    scan_integrator->integratePointcloud(random_pointcloud);

    scan_occupancy_map->prune();
    const Index3D min_index = beam_occupancy_map->getMinIndex().cwiseMin(
        scan_occupancy_map->getMinIndex());
    const Index3D max_index = beam_occupancy_map->getMaxIndex().cwiseMax(
        scan_occupancy_map->getMaxIndex());

    for (const Index3D& index : Grid(min_index, max_index)) {
      const FloatingPoint cell_value_in_beam_map =
          beam_occupancy_map->getCellValue(index);
      const FloatingPoint cell_value_in_scan_map =
          scan_occupancy_map->getCellValue(index);
      EXPECT_NEAR(cell_value_in_scan_map, cell_value_in_beam_map,
                  CoarseToFineIntegrator3D::kMaxAcceptableUpdateError);
    }
  }
}
}  // namespace wavemap

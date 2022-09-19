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

    const FloatingPoint min_elevation_angle = -kPi;
    const FloatingPoint max_elevation_angle = kPi;
    const FloatingPoint min_azimuth_angle = -kPi;
    const FloatingPoint max_azimuth_angle = kPi;

    const int num_rows = getRandomIndexElement(10, 200);
    const int num_cols = getRandomIndexElement(10, 1000);

    const FloatingPoint min_distance =
        std::max(static_cast<FloatingPoint>(num_rows) * min_cell_width /
                     (max_elevation_angle - min_elevation_angle),
                 static_cast<FloatingPoint>(num_cols) * min_cell_width /
                     (max_azimuth_angle - min_azimuth_angle));
    constexpr FloatingPoint kMaxDistance = 200.f;
    CHECK_LE(min_distance, kMaxDistance);

    // Generate a random point cloud and save its end points in a hashed set
    const PosedPointcloud<Point3D> random_pointcloud = getRandomPointcloud(
        min_elevation_angle, max_elevation_angle, num_rows, min_azimuth_angle,
        max_azimuth_angle, num_cols, min_distance, kMaxDistance);
    std::unordered_set<Index3D, VoxbloxIndexHash<3>> ray_end_points;
    for (const auto& end_point : random_pointcloud.getPointsGlobal()) {
      const Index3D index =
          convert::pointToNearestIndex<3>(end_point, min_cell_width_inv);
      ray_end_points.emplace(index);
    }

    // Set up the occupancy map, integrator and integrate the point cloud
    VolumetricDataStructure3D::Ptr occupancy_map =
        std::make_shared<HashedBlocks<SaturatingOccupancyCell, 3>>(
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
}  // namespace wavemap

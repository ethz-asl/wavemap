#include <unordered_set>

#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/integrator/integrator_base.h"
#include "wavemap/integrator/projection_model/spherical_projector.h"
#include "wavemap/integrator/projective/coarse_to_fine/coarse_to_fine_integrator.h"
#include "wavemap/integrator/projective/coarse_to_fine/hashed_chunked_wavelet_integrator.h"
#include "wavemap/integrator/projective/coarse_to_fine/hashed_wavelet_integrator.h"
#include "wavemap/integrator/projective/coarse_to_fine/wavelet_integrator.h"
#include "wavemap/integrator/projective/fixed_resolution/fixed_resolution_integrator.h"
#include "wavemap/integrator/ray_tracing/ray_tracing_integrator.h"
#include "wavemap/map/hashed_blocks.h"
#include "wavemap/map/hashed_wavelet_octree.h"
#include "wavemap/map/volumetric_data_structure_base.h"
#include "wavemap/map/volumetric_octree.h"
#include "wavemap/map/wavelet_octree.h"
#include "wavemap/test/config_generator.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"
#include "wavemap/utils/iterate/grid_iterator.h"
#include "wavemap/utils/print/eigen.h"

namespace wavemap {
class PointcloudIntegratorTest : public FixtureBase,
                                 public GeometryGenerator,
                                 public ConfigGenerator {
 protected:
  PosedPointcloud<> getRandomPointcloud(
      const SphericalProjector& projection_model,
      FloatingPoint min_distance = 0.f, FloatingPoint max_distance = 30.f) {
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
                      {projection_model.indexToImage(image_index), 1.f});
    }

    return PosedPointcloud<>(getRandomTransformation<3>(), pointcloud);
  }
};

template <typename T>
using PointcloudIntegratorTypedTest = PointcloudIntegratorTest;

template <typename IntegratorT, typename DataStructureT>
struct IntegratorDataStructurePair {
  using IntegratorType = IntegratorT;
  using DataStructureType = DataStructureT;
};

using IntegratorTypes = ::testing::Types<
    IntegratorDataStructurePair<CoarseToFineIntegrator, VolumetricOctree>,
    IntegratorDataStructurePair<WaveletIntegrator, WaveletOctree>,
    IntegratorDataStructurePair<HashedWaveletIntegrator, HashedWaveletOctree>,
    IntegratorDataStructurePair<HashedChunkedWaveletIntegrator,
                                HashedChunkedWaveletOctree>>;
TYPED_TEST_SUITE(PointcloudIntegratorTypedTest, IntegratorTypes, );

TYPED_TEST(PointcloudIntegratorTypedTest,
           EquivalenceToFixedResolutionIntegrator) {
  constexpr int kNumRepetitions = 3;
  constexpr int kNumPointclouds = 10;
  for (int idx = 0; idx < kNumRepetitions; ++idx) {
    const auto projective_integrator_config =
        ConfigGenerator::getRandomConfig<ProjectiveIntegratorConfig>();
    const auto data_structure_config = ConfigGenerator::getRandomConfig<
        typename TypeParam::DataStructureType::Config>();
    const auto projection_model = std::make_shared<SphericalProjector>(
        ConfigGenerator::getRandomConfig<SphericalProjectorConfig>());
    const auto posed_range_image =
        std::make_shared<PosedImage<>>(projection_model->getDimensions());
    const auto beam_offset_image =
        std::make_shared<Image<Vector2D>>(projection_model->getDimensions());
    const auto measurement_model = std::make_shared<ContinuousBeam>(
        ConfigGenerator::getRandomConfig<ContinuousBeamConfig>(
            *projection_model),
        projection_model, posed_range_image, beam_offset_image);

    VolumetricDataStructureBase::Ptr reference_occupancy_map =
        std::make_shared<HashedBlocks>(data_structure_config);
    IntegratorBase::Ptr reference_integrator =
        std::make_shared<FixedResolutionIntegrator>(
            projective_integrator_config, projection_model, posed_range_image,
            beam_offset_image, measurement_model, reference_occupancy_map);

    auto evaluated_occupancy_map =
        std::make_shared<typename TypeParam::DataStructureType>(
            data_structure_config);
    auto evaluated_integrator =
        std::make_shared<typename TypeParam::IntegratorType>(
            projective_integrator_config, projection_model, posed_range_image,
            beam_offset_image, measurement_model, evaluated_occupancy_map);

    for (int cloud_idx = 0; cloud_idx < kNumPointclouds; ++cloud_idx) {
      const PosedPointcloud<> random_pointcloud =
          TestFixture::getRandomPointcloud(*projection_model);
      reference_integrator->integratePointcloud(random_pointcloud);
      evaluated_integrator->integratePointcloud(random_pointcloud);
      evaluated_occupancy_map->prune();
    }

    // Compare the evaluated and fixed resolution reference map
    // Make sure all cells of the reference map are accounted for
    reference_occupancy_map->forEachLeaf([&](const OctreeIndex& node_index,
                                             FloatingPoint reference_value) {
      const Index3D min_index = convert::nodeIndexToMinCornerIndex(node_index);
      const Index3D max_index = convert::nodeIndexToMinCornerIndex(node_index);
      for (const auto& index : Grid(min_index, max_index)) {
        const FloatingPoint evaluated_value =
            evaluated_occupancy_map->getCellValue(index);
        EXPECT_NEAR(evaluated_value, reference_value,
                    projective_integrator_config.termination_update_error)
            << "For cell index " << node_index.toString();
      }
    });
    // Make sure the evaluated map contains no spurious cells
    evaluated_occupancy_map->forEachLeaf([&](const OctreeIndex& node_index,
                                             FloatingPoint evaluated_value) {
      const Index3D min_index = convert::nodeIndexToMinCornerIndex(node_index);
      const Index3D max_index = convert::nodeIndexToMinCornerIndex(node_index);
      for (const auto& index : Grid(min_index, max_index)) {
        const FloatingPoint reference_value =
            reference_occupancy_map->getCellValue(index);
        EXPECT_NEAR(evaluated_value, reference_value,
                    projective_integrator_config.termination_update_error)
            << "For cell index " << print::eigen::oneLine(index);
      }
    });
  }
}

TEST_F(PointcloudIntegratorTest, RayTracingIntegrator) {
  for (int idx = 0; idx < 3; ++idx) {
    const auto ray_tracing_integrator_config =
        getRandomConfig<RayTracingIntegratorConfig>();
    const auto data_structure_config =
        getRandomConfig<VolumetricDataStructureConfig>();
    const auto projection_model =
        SphericalProjector(getRandomConfig<SphericalProjectorConfig>());
    const FloatingPoint min_cell_width_inv =
        1.f / data_structure_config.min_cell_width;

    // Set the min distance s.t. no cells are updated by more than 1 beam
    // NOTE: Otherwise one beam's free space update could erase another beam's
    //       endpoint and this situation is not covered in this unit test.
    constexpr FloatingPoint kMaxDistance = 200.f;
    const FloatingPoint min_distance =
        data_structure_config.min_cell_width *
        projection_model.getDimensions()
            .cast<FloatingPoint>()
            .cwiseQuotient(projection_model.getMaxImageCoordinates() -
                           projection_model.getMinImageCoordinates())
            .maxCoeff();
    if (kMaxDistance <= min_distance) {
      --idx;
      continue;
    }

    // Generate a random pointcloud and save its end points in a hashed set
    const PosedPointcloud<> random_pointcloud =
        getRandomPointcloud(projection_model, min_distance, kMaxDistance);
    std::unordered_set<Index3D, IndexHash<3>> ray_end_points;
    for (const auto& end_point : random_pointcloud.getPointsGlobal()) {
      const Index3D index =
          convert::pointToNearestIndex<3>(end_point, min_cell_width_inv);
      ray_end_points.emplace(index);
    }

    // Set up the occupancy map, integrator and integrate the pointcloud
    VolumetricDataStructureBase::Ptr occupancy_map =
        std::make_shared<HashedBlocks>(data_structure_config);
    IntegratorBase::Ptr pointcloud_integrator =
        std::make_shared<RayTracingIntegrator>(ray_tracing_integrator_config,
                                               occupancy_map);
    pointcloud_integrator->integratePointcloud(random_pointcloud);

    // Check the map
    // NOTE: This test may generate false positives if the pointcloud contains
    //       overlapping rays which erase each other's endpoints. This is most
    //       likely to happen when the map resolution is low w.r.t. the spacing
    //       between the pointcloud's beams.
    for (const Index3D& index :
         Grid(occupancy_map->getMinIndex(), occupancy_map->getMaxIndex())) {
      const bool cell_occupied_in_map =
          (0.f < occupancy_map->getCellValue(index));
      const bool cell_contains_ray_end_point = ray_end_points.count(index);
      EXPECT_EQ(cell_occupied_in_map, cell_contains_ray_end_point)
          << "for index " << print::eigen::oneLine(index) << ", min cell width "
          << data_structure_config.min_cell_width << " and pointcloud size "
          << random_pointcloud.getPointsLocal().size();
    }
  }
}
}  // namespace wavemap

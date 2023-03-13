#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_WAVELET_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_WAVELET_INTEGRATOR_H_

#include <memory>
#include <utility>
#include <vector>

#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/integrator/projective/coarse_to_fine/range_image_intersector.h"
#include "wavemap/integrator/projective/projective_integrator.h"
#include "wavemap/utils/thread_pool.h"

namespace wavemap {
class HashedWaveletIntegrator : public ProjectiveIntegrator {
 public:
  HashedWaveletIntegrator(const ProjectiveIntegratorConfig& config,
                          ProjectorBase::ConstPtr projection_model,
                          PosedImage<>::Ptr posed_range_image,
                          Image<Vector2D>::Ptr beam_offset_image,
                          MeasurementModelBase::ConstPtr measurement_model,
                          HashedWaveletOctree::Ptr occupancy_map)
      : ProjectiveIntegrator(
            config, std::move(projection_model), std::move(posed_range_image),
            std::move(beam_offset_image), std::move(measurement_model)),
        occupancy_map_(std::move(CHECK_NOTNULL(occupancy_map))) {}

 private:
  const HashedWaveletOctree::Ptr occupancy_map_;
  const FloatingPoint min_cell_width_ = occupancy_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv_ = 1.f / min_cell_width_;
  ThreadPool thread_pool_{2};

  static constexpr FloatingPoint kNoiseThreshold = 1e-4f;
  const FloatingPoint min_log_odds_ = occupancy_map_->getConfig().min_log_odds;
  const FloatingPoint max_log_odds_ = occupancy_map_->getConfig().max_log_odds;
  const IndexElement tree_height_ = occupancy_map_->getTreeHeight();

  std::shared_ptr<RangeImageIntersector> range_image_intersector_;
  static constexpr auto kUnitCubeHalfDiagonal =
      constants<FloatingPoint>::kSqrt3 / 2.f;

  std::pair<OctreeIndex, OctreeIndex> getFovMinMaxIndices(
      const Point3D& sensor_origin) const;

  using BlockList = std::vector<OctreeIndex>;
  void recursiveTester(const OctreeIndex& node_index, BlockList& job_list);

  HashedWaveletOctree::Coefficients::Scale recursiveSamplerCompressor(
      const OctreeIndex& node_index, FloatingPoint node_value,
      HashedWaveletOctree::NodeType& parent_node,
      OctreeIndex ::RelativeChild relative_child_index);

  void updateMap() override;
};
}  // namespace wavemap

#include "wavemap/integrator/projective/coarse_to_fine/impl/hashed_wavelet_integrator_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_WAVELET_INTEGRATOR_H_

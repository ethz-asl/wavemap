#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_CHUNKED_WAVELET_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_CHUNKED_WAVELET_INTEGRATOR_H_

#include <memory>
#include <utility>
#include <vector>

#include "wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/integrator/projective/coarse_to_fine/range_image_intersector.h"
#include "wavemap/integrator/projective/projective_integrator.h"
#include "wavemap/utils/thread_pool.h"

namespace wavemap {
class HashedChunkedWaveletIntegrator : public ProjectiveIntegrator {
 public:
  HashedChunkedWaveletIntegrator(
      const ProjectiveIntegratorConfig& config,
      ProjectorBase::ConstPtr projection_model,
      PosedImage<>::Ptr posed_range_image,
      Image<Vector2D>::Ptr beam_offset_image,
      MeasurementModelBase::ConstPtr measurement_model,
      HashedChunkedWaveletOctree::Ptr occupancy_map)
      : ProjectiveIntegrator(
            config, std::move(projection_model), std::move(posed_range_image),
            std::move(beam_offset_image), std::move(measurement_model)),
        occupancy_map_(std::move(CHECK_NOTNULL(occupancy_map))) {}

 private:
  using BlockList = std::vector<OctreeIndex>;

  const HashedChunkedWaveletOctree::Ptr occupancy_map_;
  ThreadPool thread_pool_;
  std::shared_ptr<RangeImageIntersector> range_image_intersector_;

  // Cache/pre-computed commonly used values
  static constexpr FloatingPoint kNoiseThreshold = 1e-4f;
  static constexpr auto kUnitCubeHalfDiagonal =
      constants<FloatingPoint>::kSqrt3 / 2.f;
  const FloatingPoint min_cell_width_ = occupancy_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv_ = 1.f / min_cell_width_;
  const FloatingPoint min_log_odds_ = occupancy_map_->getMinLogOdds();
  const FloatingPoint max_log_odds_ = occupancy_map_->getMaxLogOdds();
  const FloatingPoint min_log_odds_padded_ = min_log_odds_ - kNoiseThreshold;
  const FloatingPoint min_log_odds_shrunk_ = min_log_odds_ + kNoiseThreshold;
  const FloatingPoint max_log_odds_padded_ = max_log_odds_ + kNoiseThreshold;
  const IndexElement tree_height_ = occupancy_map_->getTreeHeight();
  const IndexElement chunk_height_ = occupancy_map_->getChunkHeight();

  std::pair<OctreeIndex, OctreeIndex> getFovMinMaxIndices(
      const Point3D& sensor_origin) const;
  void recursiveTester(const OctreeIndex& node_index,
                       BlockList& update_job_list);

  void updateMap() override;
  void updateBlock(HashedChunkedWaveletOctree::Block& block,
                   const OctreeIndex& block_index);
};
}  // namespace wavemap

#include "wavemap/integrator/projective/coarse_to_fine/impl/hashed_chunked_wavelet_integrator_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_CHUNKED_WAVELET_INTEGRATOR_H_

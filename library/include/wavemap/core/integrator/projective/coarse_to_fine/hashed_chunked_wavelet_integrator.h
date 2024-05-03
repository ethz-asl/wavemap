#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_CHUNKED_WAVELET_INTEGRATOR_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_CHUNKED_WAVELET_INTEGRATOR_H_

#include <memory>
#include <utility>
#include <vector>

#include "wavemap/core/integrator/projective/coarse_to_fine/range_image_intersector.h"
#include "wavemap/core/integrator/projective/projective_integrator.h"
#include "wavemap/core/map/hashed_chunked_wavelet_octree.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap {
class HashedChunkedWaveletIntegrator : public ProjectiveIntegrator {
 public:
  HashedChunkedWaveletIntegrator(
      const ProjectiveIntegratorConfig& config,
      ProjectorBase::ConstPtr projection_model,
      PosedImage<>::Ptr posed_range_image,
      Image<Vector2D>::Ptr beam_offset_image,
      MeasurementModelBase::ConstPtr measurement_model,
      HashedChunkedWaveletOctree::Ptr occupancy_map,
      std::shared_ptr<ThreadPool> thread_pool = nullptr)
      : ProjectiveIntegrator(
            config, std::move(projection_model), std::move(posed_range_image),
            std::move(beam_offset_image), std::move(measurement_model)),
        occupancy_map_(std::move(CHECK_NOTNULL(occupancy_map))),
        thread_pool_(thread_pool ? std::move(thread_pool)
                                 : std::make_shared<ThreadPool>()) {}

 private:
  using BlockList = std::vector<HashedChunkedWaveletOctree::BlockIndex>;

  const HashedChunkedWaveletOctree::Ptr occupancy_map_;
  std::shared_ptr<ThreadPool> thread_pool_;
  std::shared_ptr<RangeImageIntersector> range_image_intersector_;

  // Cache/pre-computed commonly used values
  static constexpr FloatingPoint kNoiseThreshold = 1e-3f;
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
                   const HashedChunkedWaveletOctree::BlockIndex& block_index);

  void updateNodeRecursive(
      HashedChunkedWaveletOctreeBlock::ChunkedOctreeType::ChunkType&
          parent_chunk,
      const OctreeIndex& parent_node_index, LinearIndex parent_in_chunk_index,
      FloatingPoint& parent_value,
      HashedChunkedWaveletOctreeBlock::ChunkedOctreeType::ChunkType::BitRef
          parent_has_child,
      bool& block_needs_thresholding);
  void updateLeavesBatch(
      const OctreeIndex& parent_index, FloatingPoint& parent_value,
      HashedChunkedWaveletOctreeBlock::ChunkedOctreeType::ChunkType::DataType&
          parent_details);
};
}  // namespace wavemap

#include "wavemap/core/integrator/projective/coarse_to_fine/impl/hashed_chunked_wavelet_integrator_inl.h"

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_CHUNKED_WAVELET_INTEGRATOR_H_

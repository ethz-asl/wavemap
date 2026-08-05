#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_WAVELET_INTEGRATOR_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_WAVELET_INTEGRATOR_H_

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "wavemap/core/integrator/projective/coarse_to_fine/range_image_intersector.h"
#include "wavemap/core/integrator/projective/projective_integrator.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap {
template <typename CellDataT = FloatingPoint>
class HashedWaveletIntegratorT : public ProjectiveIntegrator {
 public:
  using MapType = HashedWaveletOctreeT<CellDataT>;
  using MapPtr = typename MapType::Ptr;

  HashedWaveletIntegratorT(
      const ProjectiveIntegratorConfig& config,
      ProjectorBase::ConstPtr projection_model,
      PosedImage<>::Ptr posed_range_image,
      Image<Vector2D>::Ptr beam_offset_image,
      MeasurementModelBase::ConstPtr measurement_model,
      MapPtr occupancy_map,
      std::shared_ptr<ThreadPool> thread_pool = nullptr)
      : ProjectiveIntegrator(
            config, std::move(projection_model), std::move(posed_range_image),
            std::move(beam_offset_image), std::move(measurement_model)),
        occupancy_map_(std::move(CHECK_NOTNULL(occupancy_map))),
        thread_pool_(thread_pool ? std::move(thread_pool)
                                 : std::make_shared<ThreadPool>()) {}

 private:
  using BlockList = std::vector<typename MapType::BlockIndex>;
  using Block = typename MapType::Block;
  using BlockIndex = typename MapType::BlockIndex;
  using OctreeType = typename Block::OctreeType;
  using Coefficients = typename Block::Coefficients;

  const MapPtr occupancy_map_;
  const std::shared_ptr<ThreadPool> thread_pool_;
  std::shared_ptr<RangeImageIntersector> range_image_intersector_;

  // Cache/pre-compute commonly used values
  const FloatingPoint min_cell_width_ = occupancy_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv_ = 1.f / min_cell_width_;
  static constexpr FloatingPoint kNoiseThreshold = 1e-4f;
  const FloatingPoint min_log_odds_ = occupancy_map_->getMinLogOdds();
  const FloatingPoint max_log_odds_ = occupancy_map_->getMaxLogOdds();
  const IndexElement tree_height_ = occupancy_map_->getTreeHeight();
  const IndexElement termination_height_ =
      min_cell_width_ < config_.max_update_resolution
          ? static_cast<IndexElement>(std::round(
                std::log2(config_.max_update_resolution / min_cell_width_)))
          : 0;
  static constexpr auto kUnitCubeHalfDiagonal =
      constants<FloatingPoint>::kSqrt3 / 2.f;

  static FloatingPoint occupancyOf(const CellDataT& value) {
    return static_cast<FloatingPoint>(value);
  }

  static void addClampedOccupancyUpdate(CellDataT& value,
                                        FloatingPoint update,
                                        FloatingPoint min_log_odds,
                                        FloatingPoint max_log_odds) {
    value += update;
    value = std::clamp(occupancyOf(value), min_log_odds, max_log_odds);
  }

  std::pair<OctreeIndex, OctreeIndex> getFovMinMaxIndices(
      const Point3D& sensor_origin) const;

  void recursiveTester(const OctreeIndex& node_index,
                       BlockList& update_job_list);

  void updateMap() override;
  void updateBlock(Block& block, const BlockIndex& block_index);
};

using HashedWaveletIntegrator = HashedWaveletIntegratorT<FloatingPoint>;
}  // namespace wavemap

#include "wavemap/core/integrator/projective/coarse_to_fine/impl/hashed_wavelet_integrator_inl.h"

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HASHED_WAVELET_INTEGRATOR_H_

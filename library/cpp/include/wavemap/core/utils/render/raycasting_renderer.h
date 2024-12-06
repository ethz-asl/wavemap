#ifndef WAVEMAP_CORE_UTILS_RENDER_RAYCASTING_RENDERER_H_
#define WAVEMAP_CORE_UTILS_RENDER_RAYCASTING_RENDERER_H_

#include <memory>

#include "wavemap/core/data_structure/image.h"
#include "wavemap/core/integrator/projection_model/projector_base.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap {
class RaycastingRenderer {
 public:
  RaycastingRenderer(MapBase::ConstPtr occupancy_map,
                     ProjectorBase::ConstPtr projection_model,
                     FloatingPoint log_odds_occupancy_threshold = 1e-3f,
                     FloatingPoint max_range = 10.f,
                     FloatingPoint default_depth_value = -1.f,
                     std::shared_ptr<ThreadPool> thread_pool = nullptr);

  const Image<>& render(const Transformation3D& T_W_C);

 private:
  static constexpr int kPatchWidth = 64;

  const MapBase::ConstPtr map_;
  const std::shared_ptr<ThreadPool> thread_pool_;

  const ProjectorBase::ConstPtr projection_model_;
  const FloatingPoint max_range_;
  const FloatingPoint log_odds_occupancy_threshold_;

  Image<FloatingPoint> depth_image_;

  template <typename MapT>
  void renderPatch(const MapT& map, const Transformation3D& T_W_C,
                   const Index2D& patch_index);
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_RENDER_RAYCASTING_RENDERER_H_

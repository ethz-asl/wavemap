#include "wavemap/core/utils/render/raycasting_renderer.h"

#include <memory>
#include <utility>

#include "wavemap/core/utils/iterate/grid_iterator.h"
#include "wavemap/core/utils/query/query_accelerator.h"
#include "wavemap/core/utils/render/raycast.h"

namespace wavemap {
RaycastingRenderer::RaycastingRenderer(
    MapBase::ConstPtr occupancy_map, ProjectorBase::ConstPtr projection_model,
    FloatingPoint log_odds_occupancy_threshold, FloatingPoint max_range,
    FloatingPoint default_depth_value, std::shared_ptr<ThreadPool> thread_pool)
    : map_(CHECK_NOTNULL(occupancy_map)),
      thread_pool_(thread_pool ? std::move(thread_pool)
                               : std::make_shared<ThreadPool>()),
      projection_model_(CHECK_NOTNULL(projection_model)),
      max_range_(max_range),
      log_odds_occupancy_threshold_(log_odds_occupancy_threshold),
      depth_image_(projection_model_->getDimensions(), default_depth_value) {}

template <typename MapT>
void RaycastingRenderer::renderPatch(const MapT& map,
                                     const Transformation3D& T_W_C,
                                     const Index2D& patch_index) {
  QueryAccelerator query_accelerator(map);

  const Index2D patch_min = patch_index * kPatchWidth;
  const Index2D patch_max = (patch_min.array() + kPatchWidth)
                                .min(depth_image_.getDimensions().array()) -
                            1;

  for (const Index2D& index : Grid(patch_min, patch_max)) {
    FloatingPoint& depth_pixel = depth_image_.at(index);
    const Vector2D image_xy = projection_model_->indexToImage(index);
    const Point3D& W_start_point = T_W_C.getPosition();
    const Point3D C_end_point =
        projection_model_->sensorToCartesian({image_xy, max_range_});
    const Point3D W_end_point = T_W_C * C_end_point;
    const auto d_start_collision = raycast::first_collision_distance(
        query_accelerator, W_start_point, W_end_point,
        log_odds_occupancy_threshold_);
    if (d_start_collision) {
      depth_pixel = d_start_collision.value();
    }
  }
}

const Image<>& RaycastingRenderer::render(const Transformation3D& T_W_C) {
  depth_image_.resetToInitialValue();

  const Index2D num_patches =
      int_math::div_round_up(depth_image_.getDimensions(), kPatchWidth);
  for (const Index2D& patch_index :
       Grid<2>(Index2D::Zero(), num_patches - Index2D::Ones())) {
    thread_pool_->add_task([this, &T_W_C, patch_index]() {
      if (const auto* map =
              dynamic_cast<const HashedWaveletOctree*>(map_.get());
          map) {
        renderPatch(*map, T_W_C, patch_index);
      }
      if (const auto* map =
              dynamic_cast<const HashedChunkedWaveletOctree*>(map_.get());
          map) {
        renderPatch(*map, T_W_C, patch_index);
      }
    });
  }
  thread_pool_->wait_all();

  return depth_image_;
}
}  // namespace wavemap

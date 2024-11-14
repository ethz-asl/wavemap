#include "wavemap_rviz_plugin/visuals/voxel_visual.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rviz/properties/parse_color.h>
#include <rviz/render_panel.h>
#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/profile/profiler_interface.h>

namespace wavemap::rviz_plugin {
VoxelVisual::VoxelVisual(Ogre::SceneManager* scene_manager,
                         rviz::ViewManager* view_manager,
                         Ogre::SceneNode* parent_node,
                         rviz::Property* submenu_root_property,
                         std::shared_ptr<MapAndMutex> map_and_mutex)
    : map_and_mutex_(std::move(map_and_mutex)),
      scene_manager_(CHECK_NOTNULL(scene_manager)),
      frame_node_(CHECK_NOTNULL(parent_node)->createChildSceneNode()),
      visibility_property_(
          "Enable", true,
          "Whether to show the octree as a multi-resolution voxel grid.",
          CHECK_NOTNULL(submenu_root_property),
          SLOT(visibilityUpdateCallback()), this),
      cell_selector_(submenu_root_property, [this]() { updateMap(true); }),
      termination_height_property_(
          "Termination height", 0,
          "Controls the resolution at which the map is drawn. Set to 0 to draw "
          "at the maximum available resolution; to 1 to stop at 1/2 of that, "
          "to 2 to stop at 1/4, etc.",
          submenu_root_property, SLOT(terminationHeightUpdateCallback()), this),
      opacity_property_("Alpha", 1.0, "Opacity of the displayed visuals.",
                        submenu_root_property, SLOT(opacityUpdateCallback()),
                        this),
      color_mode_property_(
          "Color mode", "", "Mode determining the voxel colors.",
          submenu_root_property, SLOT(colorModeUpdateCallback()), this),
      flat_color_property_(
          "Flat color", rviz::ogreToQt(voxel_flat_color_),
          R"(Solid color to use when "Color Mode" is set to "Flat")",
          submenu_root_property, SLOT(flatColorUpdateCallback()), this),
      frame_rate_properties_("Frame rate", QVariant(),
                             "Properties to control the frame rate.",
                             submenu_root_property),
      num_queued_blocks_indicator_("Queued updates", 0,
                                   "Number of blocks in the update queue.",
                                   &frame_rate_properties_),
      max_ms_per_frame_property_(
          "Max update time", 20,
          "Limit update time per frame in milliseconds, to maintain "
          "a reasonable frame rate when maps are large.",
          &frame_rate_properties_) {
  // Initialize the property menu
  // General
  termination_height_property_.setMin(0);
  num_queued_blocks_indicator_.setReadOnly(true);
  max_ms_per_frame_property_.setMin(0);
  // Color mode
  color_mode_property_.clearOptions();
  for (const auto& name : VoxelColorMode::names) {
    color_mode_property_.addOption(name);
  }
  color_mode_property_.setStringStd(voxel_color_mode_.toStr());
  flat_color_property_.setHidden(voxel_color_mode_ != VoxelColorMode::kFlat);

  // Initialize the camera tracker used to update the LOD levels for each block
  prerender_listener_ = std::make_unique<ViewportPrerenderListener>(
      view_manager->getRenderPanel()->getViewport(),
      [this](Ogre::Camera* active_camera) {
        prerenderCallback(active_camera);
      });

  // Initialize the voxel material
  // NOTE: Certain properties, such as alpha transparency, are set on a
  //       per-material basis. We therefore need to create one unique material
  //       for each voxel layer visual to keep them from overwriting each
  //       other's settings.
  static int instance_count = 0;
  ++instance_count;
  voxel_material_ =
      Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBox");
  voxel_material_ =
      Ogre::MaterialPtr(voxel_material_)
          ->clone("WavemapVoxelMaterial_" + std::to_string(instance_count));
  voxel_material_->load();
}

VoxelVisual::~VoxelVisual() {
  // Destroy the camera prerender listener
  // NOTE: This must be done before any of the objects that are used in the
  //       prerender callback (incl. members of VoxelVisual) are destroyed.
  prerender_listener_.reset();
  // Destroy the frame node
  scene_manager_->destroySceneNode(frame_node_);
}

void VoxelVisual::updateMap(bool redraw_all) {
  ProfilerZoneScoped;
  if (!visibility_property_.getBool()) {
    return;
  }

  // Lock the map mutex, to ensure it doesn't get written to while we read it
  {
    std::scoped_lock lock(map_and_mutex_->mutex);
    MapBase::ConstPtr map = map_and_mutex_->map;
    if (!map) {
      return;
    }
    cell_selector_.setMap(map);

    // Constants
    const IndexElement tree_height = map->getTreeHeight();
    const FloatingPoint min_cell_width = map->getMinCellWidth();
    const FloatingPoint alpha = opacity_property_.getFloat();

    // Limit the max selectable termination height to the height of the tree
    termination_height_property_.setMax(tree_height);

    // Start tracking time, s.t. we can later check how long we've been working
    // on the current cycle's updates
    const Timestamp start_time = Time::now();

    // If the map is of hash-map type, process it using the block update queue.
    if (const auto* hashed_map =
            dynamic_cast<const HashedWaveletOctree*>(map.get());
        hashed_map) {
      // Remove blocks that no longer exist in the map
      {
        // From the visuals (blocks that were already drawn)
        for (auto it = block_voxel_layers_map_.begin();
             it != block_voxel_layers_map_.end();) {
          const auto block_idx = it->first;
          if (!hashed_map->hasBlock(block_idx)) {
            it = block_voxel_layers_map_.erase(it);
          } else {
            ++it;
          }
        }
        // From the queue (blocks that were about to be drawn)
        for (auto it = block_update_queue_.begin();
             it != block_update_queue_.end();) {
          const auto block_idx = it->first;
          if (!hashed_map->hasBlock(block_idx)) {
            it = block_update_queue_.erase(it);
          } else {
            ++it;
          }
        }
      }

      // Add all blocks that changed since the last publication time
      // to the drawing queue
      hashed_map->forEachBlock(
          [this, redraw_all,
           min_termination_height = termination_height_property_.getInt()](
              const Index3D& block_index, const auto& block) {
            if (redraw_all || last_update_time_ < block.getLastUpdatedStamp()) {
              block_update_queue_[block_index] = min_termination_height;
              // Force the LODs to be updated, s.t. the new blocks directly get
              // drawn at the right max resolution
              force_lod_update_ = true;
            }
          });
    } else {  // Otherwise, draw the whole octree at once (legacy support)
      const IndexElement num_levels = tree_height + 1;
      VoxelsPerLevel voxels_per_level(num_levels);
      map->forEachLeaf([&voxels_per_level, this, tree_height, min_cell_width](
                           const auto& cell_index, auto cell_log_odds) {
        appendLeafCenterAndColor(tree_height, min_cell_width, cell_index,
                                 cell_log_odds, voxels_per_level);
      });
      const Index3D root_idx = Index3D::Zero();
      drawMultiResolutionVoxels(tree_height, min_cell_width, root_idx, alpha,
                                voxels_per_level,
                                block_voxel_layers_map_[root_idx]);
    }

    // Store the last update time,
    // used to check for blocks that changed since the last update
    last_update_time_ = start_time;
  }
}

void VoxelVisual::updateLOD(const Ogre::Camera& active_camera) {
  ProfilerZoneScoped;
  if (!visibility_property_.getBool()) {
    return;
  }

  // Lock to the map mutex, to ensure it doesn't get written to while we read it
  std::scoped_lock lock(map_and_mutex_->mutex);
  MapBase::ConstPtr map = map_and_mutex_->map;
  if (!map) {
    return;
  }

  // Cast the map to its derived hashed map type
  // NOTE: If the cast fails, we don't need to do anything as non-hashed maps
  //       are drawn without LODs or the block update queue.
  if (const auto* hashed_map =
          dynamic_cast<const HashedWaveletOctree*>(map.get());
      hashed_map) {
    hashed_map->forEachBlock(
        [this, tree_height = map->getTreeHeight(),
         min_termination_height = termination_height_property_.getInt(),
         min_cell_width = map->getMinCellWidth(),
         &block_update_queue = block_update_queue_,
         &active_camera](const Index3D& block_index, const auto& /*block*/) {
          // Compute the recommended LOD level height
          const OctreeIndex block_node_index{tree_height, block_index};
          const auto term_height_recommended = computeRecommendedBlockLodHeight(
              active_camera, block_node_index, min_cell_width,
              min_termination_height, tree_height - 1);

          // If the block is already queued to be updated, set the recommended
          // level
          if (block_update_queue.count(block_index)) {
            block_update_queue[block_index] = term_height_recommended;
          } else {
            // Otherwise, only add the block to the update queue if the
            // recommended level is higher than what's currently drawn or
            // significantly lower
            const auto term_height_current =
                getCurrentBlockLodHeight(tree_height, block_index);
            if (term_height_current) {
              if (term_height_current.value() < min_termination_height ||
                  term_height_current.value() < term_height_recommended - 1 ||
                  term_height_recommended < term_height_current.value()) {
                block_update_queue_[block_index] = term_height_recommended;
              }
            }
          }
        });
  }
}

IndexElement VoxelVisual::computeRecommendedBlockLodHeight(
    const Ogre::Camera& active_camera, const OctreeIndex& block_index,
    FloatingPoint min_cell_width, IndexElement min_height,
    IndexElement max_height) {
  ProfilerZoneScoped;
  // TODO(victorr): Compute the LoD level using the camera's projection matrix
  //                and the screen's pixel density, to better generalize across
  //                displays (high/low DPI) and alternative projection modes
  // If the projection type is orthographic, e.g. when
  // using Rviz's TopDownOrtho ViewController, always use the highest resolution
  if (active_camera.getProjectionType() == Ogre::PT_ORTHOGRAPHIC) {
    return min_height;
  }

  // Compute the recommended level based on the size of the cells projected into
  // the image plane
  const AABB block_aabb = convert::nodeIndexToAABB(block_index, min_cell_width);
  const Point3D camera_position{active_camera.getPosition().x,
                                active_camera.getPosition().y,
                                active_camera.getPosition().z};
  const FloatingPoint distance_to_cam =
      block_aabb.minDistanceTo(camera_position);
  constexpr FloatingPoint kFactor = 0.002f;
  return std::clamp(static_cast<IndexElement>(std::floor(std::log2(
                        1.f + kFactor * distance_to_cam / min_cell_width))),
                    min_height, max_height);
}

std::optional<IndexElement> VoxelVisual::getCurrentBlockLodHeight(
    IndexElement map_tree_height, const Index3D& block_idx) {
  ProfilerZoneScoped;
  if (block_voxel_layers_map_.count(block_idx)) {
    return map_tree_height -
           (static_cast<int>(block_voxel_layers_map_[block_idx].size()) - 1);
  } else {
    return std::nullopt;
  }
}

// Position and orientation are passed through to the SceneNode
void VoxelVisual::setFramePosition(const Ogre::Vector3& position) {
  ProfilerZoneScoped;
  frame_node_->setPosition(position);
}

void VoxelVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  ProfilerZoneScoped;
  frame_node_->setOrientation(orientation);
}

void VoxelVisual::visibilityUpdateCallback() {
  ProfilerZoneScoped;
  if (visibility_property_.getBool()) {
    updateMap(true);
  } else {
    clear();
  }
}

void VoxelVisual::opacityUpdateCallback() {
  ProfilerZoneScoped;
  FloatingPoint alpha = opacity_property_.getFloat();
  setAlpha(alpha);
}

void VoxelVisual::colorModeUpdateCallback() {
  ProfilerZoneScoped;
  // Update the cached color mode value
  const VoxelColorMode old_color_mode = voxel_color_mode_;
  voxel_color_mode_ = VoxelColorMode(color_mode_property_.getStdString());

  // Show/hide the flat color picker depending on the chosen mode
  flat_color_property_.setHidden(voxel_color_mode_ != VoxelColorMode::kFlat);

  // Update the map if the color mode changed
  if (voxel_color_mode_ != old_color_mode) {
    updateMap(true);
  }
}

void VoxelVisual::flatColorUpdateCallback() {
  ProfilerZoneScoped;
  // Update the cached color value
  const Ogre::ColourValue old_flat_color = voxel_flat_color_;
  voxel_flat_color_ = flat_color_property_.getOgreColor();

  // Update the map if the color changed
  if (voxel_flat_color_ != old_flat_color) {
    updateMap(true);
  }
}

void VoxelVisual::appendLeafCenterAndColor(int tree_height,
                                           FloatingPoint min_cell_width,
                                           const OctreeIndex& cell_index,
                                           FloatingPoint cell_log_odds,
                                           VoxelsPerLevel& voxels_per_level) {
  // Check if the cell should be drawn
  if (!cell_selector_.shouldBeDrawn(cell_index, cell_log_odds)) {
    return;
  }

  // Determine the cell's position
  const IndexElement depth = tree_height - cell_index.height;
  CHECK_GE(depth, 0);
  CHECK_LT(depth, voxels_per_level.size());
  const Point3D cell_center =
      convert::nodeIndexToCenterPoint(cell_index, min_cell_width);

  // Create the cube at the right scale
  auto& point = voxels_per_level[depth].emplace_back();
  point.center.x = cell_center[0];
  point.center.y = cell_center[1];
  point.center.z = cell_center[2];

  // Set the cube's color
  switch (voxel_color_mode_) {
    case VoxelColorMode::kFlat:
      point.color = voxel_flat_color_;
      break;
    case VoxelColorMode::kProbability:
      point.color = logOddsToColor(cell_log_odds);
      break;
    case VoxelColorMode::kHeight:
    default:
      point.color = positionToColor(cell_center);
      break;
  }
}

void VoxelVisual::drawMultiResolutionVoxels(IndexElement tree_height,
                                            FloatingPoint min_cell_width,
                                            const Index3D& block_index,
                                            FloatingPoint alpha,
                                            VoxelsPerLevel& voxels_per_level,
                                            VoxelLayers& voxel_layer_visuals) {
  ProfilerZoneScoped;
  // Add a voxel layer for each scale level
  const std::string prefix =
      "voxel_layer_" + std::to_string(Index3DHash()(block_index)) + "_";
  for (size_t depth = 0; depth < voxels_per_level.size(); ++depth) {
    // Allocate the pointcloud representing this voxel grid level if needed
    if (voxel_layer_visuals.size() <= depth) {
      const Ogre::String name = prefix + std::to_string(depth);
      const IndexElement height = tree_height - static_cast<int>(depth);
      const FloatingPoint cell_width =
          convert::heightToCellWidth(min_cell_width, height);
      auto& voxel_layer = voxel_layer_visuals.emplace_back(
          std::make_unique<CellLayer>(voxel_material_));
      voxel_layer->setName(name);
      voxel_layer->setCellDimensions(cell_width, cell_width, cell_width);
      voxel_layer->setAlpha(alpha);
      frame_node_->attachObject(voxel_layer.get());
    }
    // Update the cells
    auto& voxel_layer = voxel_layer_visuals[depth];
    const auto& cells_at_level = voxels_per_level[depth];
    voxel_layer->setCells(cells_at_level);
  }
  // Deallocate levels that are no longer needed
  for (size_t depth = voxel_layer_visuals.size() - 1;
       voxels_per_level.size() <= depth; --depth) {
    frame_node_->detachObject(voxel_layer_visuals[depth].get());
    voxel_layer_visuals.pop_back();
  }
}

void VoxelVisual::processBlockUpdateQueue(const Point3D& camera_position) {
  ProfilerZoneScoped;
  if (!visibility_property_.getBool()) {
    return;
  }

  // Get a shared-access lock to the map,
  // to ensure it doesn't get written to while we read it
  std::scoped_lock lock(map_and_mutex_->mutex);
  MapBase::ConstPtr map = map_and_mutex_->map;
  if (!map) {
    return;
  }

  if (const auto* hashed_map =
          dynamic_cast<const HashedWaveletOctree*>(map.get());
      hashed_map) {
    // Constants
    const FloatingPoint min_cell_width = map->getMinCellWidth();
    const IndexElement tree_height = map->getTreeHeight();
    const FloatingPoint alpha = opacity_property_.getFloat();

    // Sort the blocks in the queue by their drawing priority
    struct ChangedBlockToSort {
      Index3D block_index;
      IndexElement term_height_difference;
      FloatingPoint distance;
    };
    std::vector<ChangedBlockToSort> changed_blocks_sorted;
    for (const auto& [block_idx, requested_term_height] : block_update_queue_) {
      const auto block_aabb = convert::nodeIndexToAABB(
          OctreeIndex{tree_height, block_idx}, min_cell_width);
      const FloatingPoint distance = block_aabb.minDistanceTo(camera_position);
      const auto current_term_height =
          getCurrentBlockLodHeight(tree_height, block_idx);
      const IndexElement term_height_difference =
          current_term_height.has_value()
              ? std::abs(requested_term_height - current_term_height.value())
              : tree_height;
      changed_blocks_sorted.emplace_back(
          ChangedBlockToSort{block_idx, term_height_difference, distance});
    }
    std::sort(changed_blocks_sorted.begin(), changed_blocks_sorted.end(),
              [](const auto& lhs, const auto& rhs) {
                // If the LOD level difference is small,
                // prioritize the most visible (nearby) blocks
                if (std::abs(lhs.term_height_difference -
                             rhs.term_height_difference) < 2) {
                  return lhs.distance < rhs.distance;
                } else {
                  // Otherwise, prioritize the blocks with the largest requested
                  // vs actual LOD level discrepancy
                  // NOTE: We sort by decreasing absolute LOD level difference,
                  //       as we assign equal importance to drawing new details
                  //       (increasing the resolution) vs reducing memory usage
                  //       (reducing the resolution).
                  return lhs.term_height_difference >
                         rhs.term_height_difference;
                }
              });

    // Redraw blocks, starting with the oldest and
    // stopping after kMaxDrawsPerCycle
    const auto start_time = Time::now();
    const auto max_time_per_frame =
        std::chrono::milliseconds(max_ms_per_frame_property_.getInt());
    const auto max_end_time = start_time + max_time_per_frame;
    for (const auto& [block_idx, _1, _2] : changed_blocks_sorted) {
      if (const auto* block = hashed_map->getBlock(block_idx); block) {
        const IndexElement term_height = block_update_queue_[block_idx];
        const int num_levels = tree_height + 1 - term_height;
        VoxelsPerLevel voxels_per_level(num_levels);
        block->forEachLeaf(
            block_idx,
            [&voxels_per_level, this, tree_height, min_cell_width](
                const auto& cell_index, auto cell_log_odds) {
              appendLeafCenterAndColor(tree_height, min_cell_width, cell_index,
                                       cell_log_odds, voxels_per_level);
            },
            term_height);
        drawMultiResolutionVoxels(tree_height, min_cell_width, block_idx, alpha,
                                  voxels_per_level,
                                  block_voxel_layers_map_[block_idx]);
        block_update_queue_.erase(block_idx);

        const auto current_time = Time::now();
        if (max_end_time < current_time) {
          break;
        }
      }
    }
  }

  num_queued_blocks_indicator_.setInt(
      static_cast<int>(block_update_queue_.size()));
}

void VoxelVisual::setAlpha(FloatingPoint alpha) {
  // Update the material alpha
  if (alpha < 0.9998) {
    // Render in alpha blending mode
    if (voxel_material_->getBestTechnique()) {
      voxel_material_->getBestTechnique()->setSceneBlending(
          Ogre::SBT_TRANSPARENT_ALPHA);
      voxel_material_->getBestTechnique()->setDepthWriteEnabled(false);
    }
  } else {
    // Render in replace mode
    if (voxel_material_->getBestTechnique()) {
      voxel_material_->getBestTechnique()->setSceneBlending(Ogre::SBT_REPLACE);
      voxel_material_->getBestTechnique()->setDepthWriteEnabled(true);
    }
  }

  // Update the renderables
  for (auto& [block_idx, block_voxel_layers] : block_voxel_layers_map_) {
    for (auto& voxel_layer : block_voxel_layers) {
      voxel_layer->setAlpha(alpha);
    }
  }
}

void VoxelVisual::prerenderCallback(Ogre::Camera* active_camera) {
  ProfilerZoneScoped;
  CHECK_NOTNULL(active_camera);
  // Recompute the desired LOD level for each block in the map if
  // the camera moved significantly or an update was requested explicitly
  const bool camera_moved =
      lod_update_distance_threshold_ < active_camera->getPosition().distance(
                                           camera_position_at_last_lod_update_);
  const Point3D camera_position{active_camera->getDerivedPosition().x,
                                active_camera->getDerivedPosition().y,
                                active_camera->getDerivedPosition().z};
  if (force_lod_update_ || camera_moved) {
    updateLOD(*active_camera);
    camera_position_at_last_lod_update_ = active_camera->getPosition();
    force_lod_update_ = false;
  }

  // Process (parts of) the block update queue at each prerender frame
  processBlockUpdateQueue(camera_position);
}
}  // namespace wavemap::rviz_plugin

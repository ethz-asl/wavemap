#include "wavemap_rviz_plugin/visuals/voxel_visual.h"

#include <algorithm>
#include <array>
#include <memory>
#include <stack>
#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <wavemap_msgs/DiscreteLayer.h>

#include <rviz/properties/parse_color.h>
#include <rviz/render_panel.h>
#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/bits/bit_operations.h>
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
      layer_color_properties_("Layer color", QVariant(),
                              "Generic color settings for layered map fields.",
                              submenu_root_property),
      scalar_min_property_("Scalar min", 0.0,
                           "Lower value used when coloring scalar continuous layers.",
                           &layer_color_properties_,
                           SLOT(layerColorUpdateCallback()), this),
      scalar_max_property_("Scalar max", 1.0,
                           "Upper value used when coloring scalar continuous layers.",
                           &layer_color_properties_,
                           SLOT(layerColorUpdateCallback()), this),
      scalar_low_color_property_(
          "Scalar low color", rviz::ogreToQt(scalar_low_color_),
          "Color used for the scalar minimum.", &layer_color_properties_,
          SLOT(layerColorUpdateCallback()), this),
      scalar_high_color_property_(
          "Scalar high color", rviz::ogreToQt(scalar_high_color_),
          "Color used for the scalar maximum.", &layer_color_properties_,
          SLOT(layerColorUpdateCallback()), this),
      bool_true_color_property_(
          "Bool true color", rviz::ogreToQt(bool_true_color_),
          "Color used for true values in boolean discrete layers.",
          &layer_color_properties_, SLOT(layerColorUpdateCallback()), this),
      show_bool_false_property_(
          "Show bool false", false,
          "Whether to draw false values in boolean discrete layers.",
          &layer_color_properties_, SLOT(layerColorUpdateCallback()), this),
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
  clear();
  // Destroy the frame node
  scene_manager_->destroySceneNode(frame_node_);
}

void VoxelVisual::clear() {
  block_update_queue_.clear();
  for (auto& [block_index, voxel_layers] : block_voxel_layers_map_) {
    (void)block_index;
    for (auto& voxel_layer : voxel_layers) {
      if (!voxel_layer) {
        continue;
      }
      voxel_layer->clear();
      if (voxel_layer->getParentSceneNode()) {
        voxel_layer->getParentSceneNode()->detachObject(voxel_layer.get());
      }
    }
  }
  block_voxel_layers_map_.clear();
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
      if (map_and_mutex_->discrete_layer_msg) {
        drawDiscreteLayer(map_and_mutex_->discrete_layer_msg.value(),
                          map_and_mutex_->discrete_layer_min_cell_width);
      } else if (map_and_mutex_->layered_map) {
        cell_selector_.setOccupancyQuery(map_and_mutex_->layered_map);
        drawLayeredMapOccupancy(*map_and_mutex_->layered_map);
      } else if (map_and_mutex_->layered_map_msg) {
        drawLayeredMapOccupancy(map_and_mutex_->layered_map_msg.value());
      }
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

void VoxelVisual::layerColorUpdateCallback() {
  ProfilerZoneScoped;
  scalar_low_color_ = scalar_low_color_property_.getOgreColor();
  scalar_high_color_ = scalar_high_color_property_.getOgreColor();
  bool_true_color_ = bool_true_color_property_.getOgreColor();
  updateMap(true);
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

void VoxelVisual::appendLayeredLeafCenterAndColor(
    const LayeredMapInterface& layered_map,
    const std::string& selected_layer_name, int tree_height,
    FloatingPoint min_cell_width, const OctreeIndex& cell_index,
    FloatingPoint cell_log_odds, VoxelsPerLevel& voxels_per_level) {
  if (selected_layer_name == "occupancy") {
    appendLeafCenterAndColor(tree_height, min_cell_width, cell_index,
                             cell_log_odds, voxels_per_level);
    return;
  }

  if (!cell_selector_.shouldBeDrawn(cell_index, cell_log_odds)) {
    return;
  }

  Ogre::ColourValue layer_color;
  if (!layered_map.getLayerColor(selected_layer_name, cell_index, cell_log_odds,
                                 layer_color)) {
    appendLeafCenterAndColor(tree_height, min_cell_width, cell_index,
                             cell_log_odds, voxels_per_level);
    return;
  }

  const IndexElement depth = tree_height - cell_index.height;
  CHECK_GE(depth, 0);
  CHECK_LT(depth, voxels_per_level.size());
  const Point3D cell_center =
      convert::nodeIndexToCenterPoint(cell_index, min_cell_width);

  auto& point = voxels_per_level[depth].emplace_back();
  point.center.x = cell_center[0];
  point.center.y = cell_center[1];
  point.center.z = cell_center[2];
  point.color = layer_color;
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


void VoxelVisual::drawLayeredMapOccupancy(const LayeredMapInterface& layered_map) {
  ProfilerZoneScoped;
  const IndexElement tree_height = layered_map.getTreeHeight();
  const FloatingPoint min_cell_width = layered_map.getMinCellWidth();
  const FloatingPoint alpha = opacity_property_.getFloat();
  const std::string selected_layer_name = map_and_mutex_->selected_layer_name;
  const IndexElement termination_height =
      std::min<IndexElement>(termination_height_property_.getInt(), tree_height);

  termination_height_property_.setMax(tree_height);
  block_update_queue_.clear();

  // Remove visuals for blocks that no longer exist in the layered map.
  const std::vector<Index3D> block_indices = layered_map.getBlockIndices();
  std::unordered_set<Index3D, Index3DHash> allocated_blocks(
      block_indices.begin(), block_indices.end());
  for (auto it = block_voxel_layers_map_.begin();
       it != block_voxel_layers_map_.end();) {
    if (!allocated_blocks.count(it->first)) {
      it = block_voxel_layers_map_.erase(it);
    } else {
      ++it;
    }
  }

  // Draw occupancy through the runtime layered map interface. Layer-specific
  // color mapping will be added on top of this path.
  for (const auto& block_index : block_indices) {
    const int num_levels = tree_height + 1 - termination_height;
    VoxelsPerLevel voxels_per_level(num_levels);
    layered_map.forEachBlockLeaf(
        block_index,
        [this, &layered_map, &selected_layer_name, tree_height, min_cell_width,
         &voxels_per_level](const OctreeIndex& cell_index,
                            FloatingPoint cell_log_odds) {
          appendLayeredLeafCenterAndColor(layered_map, selected_layer_name,
                                           tree_height, min_cell_width,
                                           cell_index, cell_log_odds,
                                           voxels_per_level);
        },
        termination_height);
    drawMultiResolutionVoxels(tree_height, min_cell_width, block_index, alpha,
                              voxels_per_level,
                              block_voxel_layers_map_[block_index]);
  }

  num_queued_blocks_indicator_.setInt(0);
}


void VoxelVisual::drawLayeredMapOccupancy(const wavemap_msgs::LayeredHashedWaveletOctree& layered_map_msg) {
  ProfilerZoneScoped;
  const IndexElement tree_height = layered_map_msg.tree_height;
  const FloatingPoint min_cell_width = layered_map_msg.min_cell_width;
  const FloatingPoint alpha = opacity_property_.getFloat();
  const IndexElement termination_height = std::min<IndexElement>(
      termination_height_property_.getInt(), tree_height);

  termination_height_property_.setMax(tree_height);
  block_update_queue_.clear();

  // Remove visuals for blocks that no longer exist in the layered map message.
  std::unordered_set<Index3D, Index3DHash> allocated_blocks;
  for (const auto& block_index_msg : layered_map_msg.allocated_block_indices) {
    allocated_blocks.emplace(block_index_msg.x, block_index_msg.y, block_index_msg.z);
  }
  for (auto it = block_voxel_layers_map_.begin(); it != block_voxel_layers_map_.end();) {
    if (!allocated_blocks.count(it->first)) {
      it = block_voxel_layers_map_.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto& block_msg : layered_map_msg.blocks) {
    const Index3D block_index{block_msg.root_node_offset.x, block_msg.root_node_offset.y, block_msg.root_node_offset.z};
    const int num_levels = tree_height + 1 - termination_height;
    VoxelsPerLevel voxels_per_level(num_levels);
    appendLayeredBlockOccupancy(layered_map_msg, block_msg, termination_height, voxels_per_level);
    drawMultiResolutionVoxels(tree_height, min_cell_width, block_index, alpha, voxels_per_level, block_voxel_layers_map_[block_index]);
  }

  num_queued_blocks_indicator_.setInt(0);
}

namespace {
Ogre::ColourValue stableIntColor(int value) {
  uint32_t hash = static_cast<uint32_t>(value) * 2654435761u;
  const float r = 0.25f + 0.75f * static_cast<float>((hash >> 16u) & 0xffu) / 255.f;
  const float g = 0.25f + 0.75f * static_cast<float>((hash >> 8u) & 0xffu) / 255.f;
  const float b = 0.25f + 0.75f * static_cast<float>(hash & 0xffu) / 255.f;
  return Ogre::ColourValue(r, g, b, 1.f);
}

int continuousLayerComponentCount(const std::string& type) {
  if (type == "float32_rgb" || type == "float32_vec3") {
    return 3;
  }
  if (type == "float32") {
    return 1;
  }
  return 0;
}

FloatingPoint readFloatLayerValue(const wavemap_msgs::Layer& layer,
                                  size_t value_index,
                                  int component_count, int component_index) {
  const size_t array_index = value_index * static_cast<size_t>(component_count) +
                             static_cast<size_t>(component_index);
  if (array_index < layer.float32_values.size()) {
    return layer.float32_values[array_index];
  }
  return 0.f;
}

Ogre::ColourValue interpolateColor(const Ogre::ColourValue& low,
                                   const Ogre::ColourValue& high,
                                   FloatingPoint t) {
  t = std::clamp(t, 0.f, 1.f);
  return Ogre::ColourValue(low.r + t * (high.r - low.r),
                           low.g + t * (high.g - low.g),
                           low.b + t * (high.b - low.b), 1.f);
}
}  // namespace

bool VoxelVisual::getGenericContinuousLayerColor(
    const wavemap_msgs::LayeredHashedWaveletOctree& layered_map_msg,
    const std::vector<std::vector<FloatingPoint>>& layer_values,
    Ogre::ColourValue& color) const {
  const std::string selected_layer_name = map_and_mutex_->selected_layer_name;
  const auto layer_name_it = std::find(layered_map_msg.layer_names.begin(),
                                       layered_map_msg.layer_names.end(),
                                       selected_layer_name);
  if (layer_name_it == layered_map_msg.layer_names.end()) {
    return false;
  }

  const size_t layer_index = static_cast<size_t>(
      std::distance(layered_map_msg.layer_names.begin(), layer_name_it));
  if (layer_values.size() <= layer_index ||
      layered_map_msg.layer_types.size() <= layer_index) {
    return false;
  }

  const std::string& layer_type = layered_map_msg.layer_types[layer_index];
  const auto& values = layer_values[layer_index];
  if ((layer_type == "float32_rgb" || layer_type == "float32_vec3") &&
      values.size() >= 3u) {
    color = Ogre::ColourValue(std::clamp(values[0], 0.f, 1.f),
                              std::clamp(values[1], 0.f, 1.f),
                              std::clamp(values[2], 0.f, 1.f), 1.f);
    return true;
  }

  if (layer_type == "float32" && !values.empty()) {
    const FloatingPoint min_value = scalar_min_property_.getFloat();
    const FloatingPoint max_value = scalar_max_property_.getFloat();
    const FloatingPoint range = max_value - min_value;
    const FloatingPoint t = std::abs(range) < kEpsilon
                                ? 0.f
                                : (values.front() - min_value) / range;
    color = interpolateColor(scalar_low_color_, scalar_high_color_, t);
    return true;
  }

  return false;
}

void VoxelVisual::drawDiscreteLayer(
    const wavemap_msgs::DiscreteLayer& discrete_layer_msg,
    FloatingPoint min_cell_width) {
  ProfilerZoneScoped;
  const FloatingPoint alpha = opacity_property_.getFloat();
  const int side_length = 1 << discrete_layer_msg.block_height;
  const IndexElement termination_height = std::min<IndexElement>(
      termination_height_property_.getInt(), discrete_layer_msg.block_height);
  const bool draw_parent_cells =
      discrete_layer_msg.block_height <= termination_height;

  termination_height_property_.setMax(discrete_layer_msg.block_height);
  block_update_queue_.clear();

  std::unordered_map<Index3D, std::vector<Cell>, Index3DHash> cells_by_parent;
  std::unordered_set<Index3D, Index3DHash> allocated_parents;
  for (const auto& cell_msg : discrete_layer_msg.cells) {
    const Index3D parent_index{cell_msg.parent_index.x, cell_msg.parent_index.y,
                               cell_msg.parent_index.z};
    allocated_parents.insert(parent_index);
    auto& cells = cells_by_parent[parent_index];

    if (draw_parent_cells) {
      Ogre::ColourValue color;
      if (!getDiscreteCellColor(discrete_layer_msg, cell_msg,
                                /*exception_index=*/-1,
                                /*is_exception=*/false, color)) {
        continue;
      }

      auto& cell = cells.emplace_back();
      const FloatingPoint parent_width =
          static_cast<FloatingPoint>(side_length) * min_cell_width;
      cell.center = Ogre::Vector3(
          (static_cast<FloatingPoint>(parent_index.x()) + 0.5f) * parent_width,
          (static_cast<FloatingPoint>(parent_index.y()) + 0.5f) * parent_width,
          (static_cast<FloatingPoint>(parent_index.z()) + 0.5f) * parent_width);
      cell.color = color;
      continue;
    }

    std::unordered_map<int, int> exception_indices;
    for (size_t exception_i = 0u;
         exception_i < cell_msg.exception_offsets.size(); ++exception_i) {
      exception_indices[cell_msg.exception_offsets[exception_i]] =
          static_cast<int>(exception_i);
    }

    for (const int offset : cell_msg.observed_offsets) {
      const auto exception_it = exception_indices.find(offset);
      const bool is_exception = exception_it != exception_indices.end();
      const int exception_index = is_exception ? exception_it->second : -1;

      Ogre::ColourValue color;
      if (!getDiscreteCellColor(discrete_layer_msg, cell_msg, exception_index,
                                is_exception, color)) {
        continue;
      }

      const int local_x = offset % side_length;
      const int local_y = (offset / side_length) % side_length;
      const int local_z = offset / (side_length * side_length);
      const FloatingPoint x =
          (cell_msg.parent_index.x * side_length + local_x + 0.5f) *
          min_cell_width;
      const FloatingPoint y =
          (cell_msg.parent_index.y * side_length + local_y + 0.5f) *
          min_cell_width;
      const FloatingPoint z =
          (cell_msg.parent_index.z * side_length + local_z + 0.5f) *
          min_cell_width;

      auto& cell = cells.emplace_back();
      cell.center = Ogre::Vector3(x, y, z);
      cell.color = color;
    }
  }

  for (auto it = block_voxel_layers_map_.begin();
       it != block_voxel_layers_map_.end();) {
    if (!allocated_parents.count(it->first)) {
      it = block_voxel_layers_map_.erase(it);
    } else {
      ++it;
    }
  }

  const FloatingPoint cell_width =
      draw_parent_cells ? static_cast<FloatingPoint>(side_length) * min_cell_width
                        : min_cell_width;
  for (auto& [parent_index, cells] : cells_by_parent) {
    if (cells.empty()) {
      block_voxel_layers_map_.erase(parent_index);
      continue;
    }

    auto& voxel_layers = block_voxel_layers_map_[parent_index];
    if (voxel_layers.empty()) {
      auto& voxel_layer = voxel_layers.emplace_back(
          std::make_unique<CellLayer>(voxel_material_));
      voxel_layer->setName("discrete_layer_" + discrete_layer_msg.name + "_" +
                           std::to_string(Index3DHash()(parent_index)));
      frame_node_->attachObject(voxel_layer.get());
    }

    auto& voxel_layer = voxel_layers.front();
    voxel_layer->setCellDimensions(cell_width, cell_width, cell_width);
    voxel_layer->setAlpha(alpha);
    voxel_layer->setCells(cells);
  }

  num_queued_blocks_indicator_.setInt(0);
}

bool VoxelVisual::getDiscreteCellColor(
    const wavemap_msgs::DiscreteLayer& layer_msg,
    const wavemap_msgs::DiscreteLayerCell& cell_msg, int exception_index,
    bool is_exception, Ogre::ColourValue& color) const {
  if (layer_msg.value_type == "bool") {
    const bool value = is_exception
                           ? cell_msg.exception_uint8_values[exception_index] != 0u
                           : cell_msg.dominant_uint8 != 0u;
    if (!value && !show_bool_false_property_.getBool()) {
      return false;
    }
    color = value ? bool_true_color_ : Ogre::ColourValue(0.35f, 0.35f, 0.35f, 1.f);
    return true;
  }

  if (layer_msg.value_type == "int") {
    const int value = is_exception ? cell_msg.exception_int32_values[exception_index]
                                   : cell_msg.dominant_int32;
    color = stableIntColor(value);
    return true;
  }

  color = Ogre::ColourValue(0.7f, 0.7f, 0.7f, 1.f);
  return true;
}

void VoxelVisual::appendLayeredBlockOccupancy(const wavemap_msgs::LayeredHashedWaveletOctree& layered_map_msg, const wavemap_msgs::LayeredHashedWaveletOctreeBlock& block_msg, IndexElement termination_height, VoxelsPerLevel& voxels_per_level) {
  ProfilerZoneScoped;
  if (block_msg.nodes.empty()) {
    return;
  }

  using Coefficients = HashedWaveletOctreeBlock::Coefficients;
  using Transform = HashedWaveletOctreeBlock::Transform;
  using LayerValues = std::vector<std::vector<FloatingPoint>>;

  struct StackElement {
    OctreeIndex node_index;
    FloatingPoint occupancy_scale;
    LayerValues layer_scales;
    size_t node_msg_index;
  };

  LayerValues root_layer_scales;
  root_layer_scales.reserve(layered_map_msg.layer_types.size());
  for (size_t layer_i = 0u; layer_i < layered_map_msg.layer_types.size(); ++layer_i) {
    const int component_count = continuousLayerComponentCount(layered_map_msg.layer_types[layer_i]);
    std::vector<FloatingPoint> values;
    values.reserve(component_count);
    const wavemap_msgs::Layer* root_layer =
        layer_i < block_msg.root_node_layers.size() ? &block_msg.root_node_layers[layer_i] : nullptr;
    for (int component_i = 0; component_i < component_count; ++component_i) {
      values.push_back(root_layer ? readFloatLayerValue(*root_layer, 0u, component_count, component_i) : 0.f);
    }
    root_layer_scales.push_back(std::move(values));
  }

  auto append_leaf = [this, &layered_map_msg, &voxels_per_level](
                         const OctreeIndex& node_index,
                         FloatingPoint occupancy_scale,
                         const LayerValues& layer_scales) {
    if (map_and_mutex_->selected_layer_name == "occupancy") {
      appendLeafCenterAndColor(layered_map_msg.tree_height,
                               layered_map_msg.min_cell_width, node_index,
                               occupancy_scale, voxels_per_level);
      return;
    }

    if (!cell_selector_.shouldBeDrawn(node_index, occupancy_scale)) {
      return;
    }

    Ogre::ColourValue layer_color;
    if (!getGenericContinuousLayerColor(layered_map_msg, layer_scales,
                                        layer_color)) {
      appendLeafCenterAndColor(layered_map_msg.tree_height,
                               layered_map_msg.min_cell_width, node_index,
                               occupancy_scale, voxels_per_level);
      return;
    }

    const IndexElement depth = layered_map_msg.tree_height - node_index.height;
    CHECK_GE(depth, 0);
    CHECK_LT(depth, voxels_per_level.size());
    const Point3D cell_center = convert::nodeIndexToCenterPoint(
        node_index, layered_map_msg.min_cell_width);

    auto& point = voxels_per_level[depth].emplace_back();
    point.center.x = cell_center[0];
    point.center.y = cell_center[1];
    point.center.z = cell_center[2];
    point.color = layer_color;
  };

  size_t next_node_msg_index = 0u;
  std::stack<StackElement> stack;
  stack.emplace(StackElement{OctreeIndex{layered_map_msg.tree_height, Index3D{block_msg.root_node_offset.x, block_msg.root_node_offset.y, block_msg.root_node_offset.z}}, block_msg.root_node_occupancy_scale_coefficient, std::move(root_layer_scales), next_node_msg_index++});

  while (!stack.empty()) {
    StackElement stack_element = stack.top();
    stack.pop();

    if (block_msg.nodes.size() <= stack_element.node_msg_index) {
      ROS_WARN("Layered map block ended before all queued nodes were read.");
      return;
    }

    const auto& node_msg = block_msg.nodes[stack_element.node_msg_index];
    Coefficients::Details occupancy_details;
    std::copy_n(node_msg.occupancy_detail_coefficients.begin(),
                occupancy_details.size(), occupancy_details.begin());
    const auto child_occupancy_scales = Transform::backward(
        {stack_element.occupancy_scale, occupancy_details});

    std::array<LayerValues, OctreeIndex::kNumChildren> child_layer_scales;
    for (size_t layer_i = 0u; layer_i < layered_map_msg.layer_types.size(); ++layer_i) {
      const int component_count = continuousLayerComponentCount(layered_map_msg.layer_types[layer_i]);
      if (component_count == 0) {
        continue;
      }
      const wavemap_msgs::Layer* detail_layer =
          layer_i < node_msg.detail_layers.size() ? &node_msg.detail_layers[layer_i] : nullptr;
      for (int component_i = 0; component_i < component_count; ++component_i) {
        Coefficients::Details detail_values;
        for (size_t coefficient_i = 0u; coefficient_i < detail_values.size(); ++coefficient_i) {
          detail_values[coefficient_i] = detail_layer ? readFloatLayerValue(*detail_layer, coefficient_i, component_count, component_i) : 0.f;
        }
        const FloatingPoint parent_value =
            stack_element.layer_scales.size() > layer_i &&
                    stack_element.layer_scales[layer_i].size() > static_cast<size_t>(component_i)
                ? stack_element.layer_scales[layer_i][component_i]
                : 0.f;
        const auto child_values = Transform::backward({parent_value, detail_values});
        for (NdtreeIndexRelativeChild child_idx = 0; child_idx < OctreeIndex::kNumChildren; ++child_idx) {
          if (child_layer_scales[child_idx].size() <= layer_i) {
            child_layer_scales[child_idx].resize(layer_i + 1u);
          }
          child_layer_scales[child_idx][layer_i].push_back(child_values[child_idx]);
        }
      }
    }

    struct ChildToVisit {
      OctreeIndex node_index;
      FloatingPoint occupancy_scale;
      LayerValues layer_scales;
      size_t node_msg_index;
    };
    std::vector<ChildToVisit> children_to_visit;

    for (NdtreeIndexRelativeChild child_idx = 0; child_idx < OctreeIndex::kNumChildren; ++child_idx) {
      const OctreeIndex child_node_index = stack_element.node_index.computeChildIndex(child_idx);
      const FloatingPoint child_occupancy_scale = child_occupancy_scales[child_idx];
      const bool child_exists = bit_ops::is_bit_set(node_msg.allocated_children_bitset, child_idx);

      if (child_exists && termination_height < child_node_index.height) {
        children_to_visit.push_back({child_node_index, child_occupancy_scale,
                                     std::move(child_layer_scales[child_idx]),
                                     next_node_msg_index++});
      } else {
        append_leaf(child_node_index, child_occupancy_scale,
                    child_layer_scales[child_idx]);
      }
    }

    // The serialized node array follows the same DFS order as the stack traversal.
    // Push in reverse so the next serialized node is popped first.
    for (auto child_it = children_to_visit.rbegin(); child_it != children_to_visit.rend(); ++child_it) {
      stack.emplace(StackElement{child_it->node_index,
                                 child_it->occupancy_scale,
                                 std::move(child_it->layer_scales),
                                 child_it->node_msg_index});
    }
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

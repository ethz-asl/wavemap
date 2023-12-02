#include "wavemap_rviz_plugin/visuals/slice_visual.h"

#include <ros/console.h>
#include <wavemap/indexing/index_conversions.h>

#include "wavemap_rviz_plugin/utils/color_conversions.h"

namespace wavemap::rviz_plugin {
SliceVisual::SliceVisual(Ogre::SceneManager* scene_manager,
                         Ogre::SceneNode* parent_node,
                         rviz::Property* submenu_root_property,
                         std::shared_ptr<MapAndMutex> map_and_mutex)
    : map_and_mutex_(std::move(map_and_mutex)),
      scene_manager_(CHECK_NOTNULL(scene_manager)),
      frame_node_(CHECK_NOTNULL(parent_node)->createChildSceneNode()),
      visibility_property_(
          "Enable", false,
          "Whether to show the octree as a multi-resolution grid.",
          CHECK_NOTNULL(submenu_root_property), SLOT(generalUpdateCallback()),
          this),
      min_occupancy_threshold_property_(
          "Min log odds", -1e6, "Ranges from -Inf to Inf.",
          submenu_root_property, SLOT(generalUpdateCallback()), this),
      max_occupancy_threshold_property_(
          "Max log odds", 1e6, "Ranges from -Inf to Inf.",
          submenu_root_property, SLOT(generalUpdateCallback()), this),
      slice_height_property_(
          "Slice height", 0.0, "Z-coordinate of the map slice to display.",
          submenu_root_property, SLOT(generalUpdateCallback()), this),
      opacity_property_("Alpha", 1.0, "Opacity of the displayed visuals.",
                        submenu_root_property, SLOT(opacityUpdateCallback()),
                        this),
      color_mode_property_(
          "Color mode", "", "Mode determining the cell colors.",
          submenu_root_property, SLOT(colorModeUpdateCallback()), this) {
  // Initialize the property menu
  color_mode_property_.clearOptions();
  for (const auto& name : SliceColorMode::names) {
    color_mode_property_.addOption(name);
  }
  color_mode_property_.setStringStd(slice_color_mode_.toStr());

  // Initialize the slice cell material
  // NOTE: Certain properties, such as alpha transparency, are set on a
  //       per-material basis. We therefore need to create one unique material
  //       for each slice visual to keep them from overwriting each other's
  //       settings.
  static int instance_count = 0;
  ++instance_count;
  slice_cell_material_ =
      Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBox");
  slice_cell_material_ =
      Ogre::MaterialPtr(slice_cell_material_)
          ->clone("WavemapSliceMaterial_" + std::to_string(instance_count));
  slice_cell_material_->load();
}

SliceVisual::~SliceVisual() {
  // Destroy the frame node
  scene_manager_->destroySceneNode(frame_node_);
}

void SliceVisual::update() {
  if (!visibility_property_.getBool()) {
    clear();
    return;
  }

  // Lock the map mutex, to ensure it doesn't get written to while we read it
  std::scoped_lock lock(map_and_mutex_->mutex);
  const VolumetricDataStructureBase::ConstPtr map = map_and_mutex_->map;
  if (!map) {
    ROS_INFO("Map is empty. Nothing to draw.");
    return;
  }

  // Constants
  const FloatingPoint min_cell_width = map->getMinCellWidth();
  const FloatingPoint min_occupancy_log_odds =
      min_occupancy_threshold_property_.getFloat();
  const FloatingPoint max_occupancy_log_odds =
      max_occupancy_threshold_property_.getFloat();
  const FloatingPoint slice_height = slice_height_property_.getFloat();
  const FloatingPoint alpha = opacity_property_.getFloat();
  const int max_height = map->getTreeHeight();

  // Cache the intersecting node z-indices in function of node height
  const IndexElement num_levels = max_height + 1;
  std::vector<IndexElement> intersecting_indices(num_levels);
  std::generate(intersecting_indices.begin(), intersecting_indices.end(),
                [=, height = 0]() mutable {
                  return static_cast<IndexElement>(
                      std::floor(slice_height / convert::heightToCellWidth(
                                                    min_cell_width, height++)));
                });

  // Add a colored square for each leaf
  std::vector<std::vector<Cell>> cells_per_level(num_levels);
  map->forEachLeaf([=, &cells_per_level](const OctreeIndex& cell_index,
                                         FloatingPoint cell_log_odds) {
    // Skip cells that don't intersect the slice
    if (cell_index.position.z() != intersecting_indices[cell_index.height]) {
      return;
    }

    // Skip cells that don't meet the occupancy threshold
    if (cell_log_odds < min_occupancy_log_odds ||
        max_occupancy_log_odds < cell_log_odds) {
      return;
    }

    // Determine the cell's position
    CHECK_GE(cell_index.height, 0);
    CHECK_LE(cell_index.height, max_height);
    const Point3D cell_center =
        convert::nodeIndexToCenterPoint(cell_index, min_cell_width);

    // Create the cube at the right scale
    auto& cell = cells_per_level[cell_index.height].emplace_back();
    cell.center.x = cell_center[0];
    cell.center.y = cell_center[1];
    cell.center.z = slice_height;

    // Set the cube's color
    switch (slice_color_mode_) {
      case SliceColorMode::kRaw: {
        cell.color = scalarToColor(cell_log_odds, min_occupancy_log_odds,
                                   max_occupancy_log_odds);
        break;
      }
      case SliceColorMode::kProbability:
      default:
        cell.color = logOddsToColor(cell_log_odds);
        break;
    }
  });

  // Add a grid layer for each scale level
  for (int height = 0; height <= max_height; ++height) {
    // Allocate the pointcloud representing this grid level if needed
    if (static_cast<int>(grid_levels_.size()) <= height) {
      const Ogre::String name = "multi_res_slice_" + std::to_string(height);
      const FloatingPoint cell_width =
          convert::heightToCellWidth(min_cell_width, height);
      auto& grid_level = grid_levels_.emplace_back(
          std::make_unique<CellLayer>(slice_cell_material_));
      grid_level->setName(name);
      grid_level->setCellDimensions(cell_width, cell_width, 0.0);
      grid_level->setAlpha(alpha);
      frame_node_->attachObject(grid_level.get());
    }
    // Update the points
    auto& grid_level = grid_levels_[height];
    grid_level->clear();
    const auto& cells_at_level = cells_per_level[height];
    grid_level->setCells(cells_at_level);
  }
}

// Position and orientation are passed through to the SceneNode
void SliceVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void SliceVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void SliceVisual::opacityUpdateCallback() {
  FloatingPoint alpha = opacity_property_.getFloat();
  setAlpha(alpha);
}

void SliceVisual::setAlpha(FloatingPoint alpha) {
  // Update the material alpha
  if (alpha < 0.9998) {
    // Render in alpha blending mode
    if (slice_cell_material_->getBestTechnique()) {
      slice_cell_material_->getBestTechnique()->setSceneBlending(
          Ogre::SBT_TRANSPARENT_ALPHA);
      slice_cell_material_->getBestTechnique()->setDepthWriteEnabled(false);
    }
  } else {
    // Render in replace mode
    if (slice_cell_material_->getBestTechnique()) {
      slice_cell_material_->getBestTechnique()->setSceneBlending(
          Ogre::SBT_REPLACE);
      slice_cell_material_->getBestTechnique()->setDepthWriteEnabled(true);
    }
  }

  // Update the renderables
  for (auto& grid_level : grid_levels_) {
    grid_level->setAlpha(alpha);
  }
}

void SliceVisual::colorModeUpdateCallback() {
  // Update the cached color mode value
  const SliceColorMode old_color_mode = slice_color_mode_;
  slice_color_mode_ = SliceColorMode(color_mode_property_.getStdString());

  // Update the map if the color mode changed
  if (slice_color_mode_ != old_color_mode) {
    update();
  }
}
}  // namespace wavemap::rviz_plugin

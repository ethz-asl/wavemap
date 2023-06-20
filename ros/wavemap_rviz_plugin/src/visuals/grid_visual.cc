#include "wavemap_rviz_plugin/visuals/grid_visual.h"

#include <ros/console.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/indexing/index_conversions.h>

namespace wavemap::rviz_plugin {
GridVisual::GridVisual(
    Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
    rviz::Property* submenu_root_property,
    const std::shared_ptr<std::shared_mutex> map_mutex,
    const std::shared_ptr<VolumetricDataStructureBase::Ptr> map)
    : map_mutex_(map_mutex),
      map_ptr_(map),
      scene_manager_(CHECK_NOTNULL(scene_manager)),
      frame_node_(CHECK_NOTNULL(parent_node)->createChildSceneNode()),
      visibility_property_(
          "Show", true,
          "Whether to show the octree as a multi-resolution grid.",
          CHECK_NOTNULL(submenu_root_property), SLOT(generalUpdateCallback()),
          this),
      min_occupancy_threshold_property_(
          "Min log odds", 1e-3, "Ranges from -Inf to Inf.",
          submenu_root_property, SLOT(generalUpdateCallback()), this),
      max_occupancy_threshold_property_(
          "Max log odds", 1e6, "Ranges from -Inf to Inf.",
          submenu_root_property, SLOT(generalUpdateCallback()), this),
      opacity_property_("Alpha", 1.0, "Opacity of the displayed visuals.",
                        submenu_root_property, SLOT(opacityUpdateCallback()),
                        this) {}

GridVisual::~GridVisual() {
  // Destroy the frame node
  scene_manager_->destroySceneNode(frame_node_);
}

void GridVisual::update() {
  if (!visibility_property_.getBool()) {
    clear();
    return;
  }

  // Get a shared-access lock to the map,
  // to ensure it doesn't get written to while we read it
  std::shared_lock lock(*map_mutex_);
  const VolumetricDataStructureBase::ConstPtr map = *map_ptr_;
  if (!map) {
    ROS_INFO("Map is empty. Nothing to draw.");
    return;
  }

  // Constants
  const FloatingPoint min_cell_width = map->getMinCellWidth();
  const FloatingPoint min_log_odds =
      min_occupancy_threshold_property_.getFloat();
  const FloatingPoint max_log_odds =
      max_occupancy_threshold_property_.getFloat();
  const FloatingPoint alpha = opacity_property_.getFloat();
  const int max_height = 14;  // todo
  const NdtreeIndexElement num_levels = max_height + 1;

  const TimePoint start_time = std::chrono::steady_clock::now();

  if (const auto* hashed_map =
          dynamic_cast<const HashedWaveletOctree*>(map.get());
      hashed_map) {
    for (const auto& [block_idx, block] : hashed_map->getBlocks()) {
      // Skip blocks that haven't changed
      if (block.getLastUpdatedStamp() < last_update_time_) {
        continue;
      }

      PointcloudList cells_per_level(num_levels);
      block.forEachLeaf(
          block_idx, [&](const auto& cell_index, auto cell_log_odds) {
            getLeafCentersAndColors(min_log_odds, max_log_odds, min_cell_width,
                                    cell_index, cell_log_odds, cells_per_level);
          });

      drawMultiResGrid(block_idx, min_cell_width, alpha, cells_per_level,
                       block_grids_[block_idx]);
    }
  } else {
    PointcloudList cells_per_level(num_levels);
    map->forEachLeaf([&](const auto& cell_index, auto cell_log_odds) {
      getLeafCentersAndColors(min_log_odds, max_log_odds, min_cell_width,
                              cell_index, cell_log_odds, cells_per_level);
    });

    const Index3D root_idx = Index3D::Zero();
    drawMultiResGrid(root_idx, min_cell_width, alpha, cells_per_level,
                     block_grids_[root_idx]);
  }

  last_update_time_ = start_time;
}

// Position and orientation are passed through to the SceneNode
void GridVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void GridVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void GridVisual::opacityUpdateCallback() {
  for (auto& [block_idx, block_grid] : block_grids_) {
    for (auto& grid_level : block_grid) {
      grid_level->setAlpha(opacity_property_.getFloat());
    }
  }
}

Ogre::ColourValue GridVisual::logOddsToColor(FloatingPoint log_odds) {
  Ogre::ColourValue color;
  color.a = 1.f;

  const FloatingPoint cell_odds = std::exp(log_odds);
  const FloatingPoint cell_prob = cell_odds / (1.f + cell_odds);
  const FloatingPoint cell_free_prob = 1.f - cell_prob;
  color.r = cell_free_prob;
  color.g = cell_free_prob;
  color.b = cell_free_prob;
  return color;
}

// NOTE: This coloring code is based on octomap_mapping, see:
//       https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/
//       octomap_server/src/OctomapServer.cpp#L1234
Ogre::ColourValue GridVisual::positionToColor(const Point3D& center_point) {
  Ogre::ColourValue color;
  color.a = 1.0;

  // Blend over HSV-values (more colors)
  constexpr FloatingPoint kScaling = 0.2f;
  constexpr FloatingPoint kOffset = -2.f;
  FloatingPoint h = kScaling * center_point.z() + kOffset;
  h -= std::floor(h);
  h *= 6;
  const FloatingPoint s = 1.f;
  const FloatingPoint v = 1.f;

  const int band_idx = std::floor(h);
  FloatingPoint f = h - static_cast<FloatingPoint>(band_idx);
  // Flip f if the band index is even
  if (!(band_idx & 1)) {
    f = 1.f - f;
  }
  const FloatingPoint m = v * (1.f - s);
  const FloatingPoint n = v * (1.f - s * f);

  switch (band_idx) {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}

void GridVisual::getLeafCentersAndColors(FloatingPoint min_occupancy_log_odds,
                                         FloatingPoint max_occupancy_log_odds,
                                         FloatingPoint min_cell_width,
                                         const OctreeIndex& cell_index,
                                         FloatingPoint cell_log_odds,
                                         PointcloudList& cells_per_level) {
  // Skip cells that don't meet the occupancy threshold
  if (cell_log_odds < min_occupancy_log_odds ||
      max_occupancy_log_odds < cell_log_odds) {
    return;
  }

  // Determine the cell's position
  CHECK_GE(cell_index.height, 0);
  CHECK_LT(cell_index.height, cells_per_level.size());
  const Point3D cell_center =
      convert::nodeIndexToCenterPoint(cell_index, min_cell_width);

  // Create the cube at the right scale
  auto& point = cells_per_level[cell_index.height].emplace_back();
  point.position.x = cell_center[0];
  point.position.y = cell_center[1];
  point.position.z = cell_center[2];

  // Set the cube's color
  switch (kColorBy) {
    case ColorBy::kProbability:
      point.color = logOddsToColor(cell_log_odds);
      break;
    case ColorBy::kPosition:
    default:
      point.color = positionToColor(cell_center);
      break;
  }
}

void GridVisual::drawMultiResGrid(const Index3D& block_index,
                                  FloatingPoint min_cell_width,
                                  FloatingPoint alpha,
                                  PointcloudList& cells_per_level,
                                  MultiResGrid& multi_res_grid) {
  // Add a grid layer for each scale level
  const std::string prefix =
      "grid_" + std::to_string(Index3DHash()(block_index)) + "_";
  for (int height = 0; height < static_cast<int>(cells_per_level.size());
       ++height) {
    // Allocate the pointcloud representing this grid level if needed
    if (static_cast<int>(multi_res_grid.size()) <= height) {
      const Ogre::String name = prefix + std::to_string(height);
      const FloatingPoint cell_width =
          convert::heightToCellWidth(min_cell_width, height);
      auto& grid_level =
          multi_res_grid.emplace_back(std::make_unique<rviz::PointCloud>());
      grid_level->setName(name);
      grid_level->setRenderMode(rviz::PointCloud::RM_BOXES);
      grid_level->setDimensions(cell_width, cell_width, cell_width);
      grid_level->setAlpha(alpha, false);
      frame_node_->attachObject(grid_level.get());
    }
    // Update the points
    auto& grid_level = multi_res_grid[height];
    grid_level->clear();
    auto& cells_at_level = cells_per_level[height];
    grid_level->addPoints(&cells_at_level.front(), cells_at_level.size());
  }
}
}  // namespace wavemap::rviz_plugin

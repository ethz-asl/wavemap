#include "wavemap_rviz_plugin/visuals/multi_resolution_grid_visual.h"

#include <wavemap_common/indexing/index_conversions.h>

namespace wavemap::rviz_plugin {
MultiResolutionGridVisual::MultiResolutionGridVisual(
    Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;

  // Ogre::SceneNodes form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent. Ogre does the math of combining those transforms when it
  // is time to render.
  // Here we create a node to store the pose of the WavemapOctree's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();
}

MultiResolutionGridVisual::~MultiResolutionGridVisual() {
  // Destroy the frame node
  scene_manager_->destroySceneNode(frame_node_);
}

void MultiResolutionGridVisual::loadMap(const VolumetricDataStructure3D& map,
                                        FloatingPoint min_occupancy_log_odds,
                                        FloatingPoint max_occupancy_log_odds,
                                        FloatingPoint alpha) {
  // Constants
  const FloatingPoint min_cell_width = map.getMinCellWidth();
  const int max_height = 14;

  // Add a colored square for each leaf
  const NdtreeIndexElement num_levels = max_height + 1;
  std::vector<std::vector<rviz::PointCloud::Point>> cells_per_level(num_levels);
  map.forEachLeaf([=, &cells_per_level](const OctreeIndex& cell_index,
                                        FloatingPoint cell_log_odds) {
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
  });

  // Add a grid layer for each scale level
  for (int height = 0; height <= max_height; ++height) {
    // Allocate the pointcloud representing this grid level if needed
    if (static_cast<int>(grid_levels_.size()) <= height) {
      const Ogre::String name = "multi_res_grid_" + std::to_string(height);
      const FloatingPoint cell_width =
          convert::heightToCellWidth(min_cell_width, height);
      auto& grid_level =
          grid_levels_.emplace_back(std::make_unique<rviz::PointCloud>());
      grid_level->setName(name);
      grid_level->setRenderMode(rviz::PointCloud::RM_BOXES);
      grid_level->setDimensions(cell_width, cell_width, cell_width);
      grid_level->setAlpha(alpha, false);
      frame_node_->attachObject(grid_level.get());
    }
    // Update the points
    auto& grid_level = grid_levels_[height];
    grid_level->clear();
    auto& cells_at_level = cells_per_level[height];
    grid_level->addPoints(&cells_at_level.front(), cells_at_level.size());
  }
}

// Position and orientation are passed through to the SceneNode
void MultiResolutionGridVisual::setFramePosition(
    const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void MultiResolutionGridVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

Ogre::ColourValue MultiResolutionGridVisual::logOddsToColor(
    FloatingPoint log_odds) {
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
Ogre::ColourValue MultiResolutionGridVisual::positionToColor(
    const Point3D& center_point) {
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
}  // namespace wavemap::rviz_plugin

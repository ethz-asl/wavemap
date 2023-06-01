#include "wavemap_rviz_plugin/visuals/multi_resolution_slice_visual.h"

#include <wavemap_common/indexing/index_conversions.h>

namespace wavemap::rviz_plugin {
MultiResolutionSliceVisual::MultiResolutionSliceVisual(
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

MultiResolutionSliceVisual::~MultiResolutionSliceVisual() {
  // Destroy the frame node
  scene_manager_->destroySceneNode(frame_node_);
}

void MultiResolutionSliceVisual::loadMap(const VolumetricDataStructure3D& map,
                                         FloatingPoint min_occupancy_log_odds,
                                         FloatingPoint max_occupancy_log_odds,
                                         FloatingPoint slice_height,
                                         FloatingPoint alpha) {
  // Constants
  const FloatingPoint min_cell_width = map.getMinCellWidth();
  const int max_height = 14;

  // Cache the intersecting node z-indices in function of node height
  const NdtreeIndexElement num_levels = max_height + 1;
  std::vector<IndexElement> intersecting_indices(num_levels);
  std::generate(intersecting_indices.begin(), intersecting_indices.end(),
                [=, height = 0]() mutable {
                  return static_cast<IndexElement>(
                      std::floor(slice_height / convert::heightToCellWidth(
                                                    min_cell_width, height++)));
                });

  // Add a colored square for each leaf
  std::vector<std::vector<rviz::PointCloud::Point>> cells_per_level(num_levels);
  map.forEachLeaf([=, &cells_per_level](const OctreeIndex& cell_index,
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
    auto& point = cells_per_level[cell_index.height].emplace_back();
    point.position.x = cell_center[0];
    point.position.y = cell_center[1];
    point.position.z = slice_height;

    // Set the cube's color
    const FloatingPoint cell_odds = std::exp(cell_log_odds);
    const FloatingPoint cell_prob = cell_odds / (1.f + cell_odds);
    const FloatingPoint cell_free_prob = 1.f - cell_prob;
    point.color.a = 1.f;
    point.color.r = cell_free_prob;
    point.color.g = cell_free_prob;
    point.color.b = cell_free_prob;
  });

  // Add a grid layer for each scale level
  for (int height = 0; height <= max_height; ++height) {
    // Allocate the pointcloud representing this grid level if needed
    if (static_cast<int>(grid_levels_.size()) <= height) {
      const Ogre::String name = "multi_res_slice_" + std::to_string(height);
      const FloatingPoint cell_width =
          convert::heightToCellWidth(min_cell_width, height);
      auto& grid_level =
          grid_levels_.emplace_back(std::make_unique<rviz::PointCloud>());
      grid_level->setName(name);
      grid_level->setRenderMode(rviz::PointCloud::RM_BOXES);
      grid_level->setDimensions(cell_width, cell_width, 0.0);
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
void MultiResolutionSliceVisual::setFramePosition(
    const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void MultiResolutionSliceVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}
}  // namespace wavemap::rviz_plugin

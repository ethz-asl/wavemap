#include "wavemap_rviz_plugin/multi_resolution_grid_visual.h"

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
  // Remove the grids from the scene
  // NOTE: Since this deregisters the elements from the scene, it must be done
  //       before the scene itself is destroyed.
  grid_levels_.clear();
  // Destroy the frame node
  scene_manager_->destroySceneNode(frame_node_);
}

void MultiResolutionGridVisual::setOctree(
    const Octree& octree, FloatingPoint min_occupancy_log_odds,
    FloatingPoint max_occupancy_log_odds) {
  // Constants
  const FloatingPoint min_cell_width = octree.getMinCellWidth();
  const int max_height = Octree::kMaxHeight;

  // Delete the previous visuals
  grid_levels_.clear();

  // Add a colored square for each leaf
  const NdtreeIndexElement num_levels = max_height + 1;
  std::vector<std::vector<rviz::PointCloud::Point>> cells_per_level(num_levels);
  octree.forEachLeaf(
      [=, &cells_per_level](const NdtreeIndex<Octree::kDim>& cell_index,
                            FloatingPoint cell_log_odds) {
        // Skip cells that don't meet the occupancy threshold
        if (cell_log_odds < min_occupancy_log_odds ||
            max_occupancy_log_odds < cell_log_odds) {
          return;
        }

        // Determine the cell's position
        CHECK_GE(cell_index.height, 0);
        CHECK_LE(cell_index.height, max_height);
        const Point<Octree::kDim> cell_center =
            convert::nodeIndexToCenterPoint(cell_index, min_cell_width);

        // Create the cube at the right scale
        auto& point = cells_per_level[cell_index.height].emplace_back();
        point.position.x = cell_center[0];
        point.position.y = cell_center[1];
        point.position.z = cell_center[2];

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
  for (NdtreeIndexElement height = 0; height <= max_height; ++height) {
    const FloatingPoint cell_width =
        convert::heightToCellWidth(min_cell_width, height);
    auto& grid_level = grid_levels_.emplace_back();
    grid_level.setName(std::to_string(height));
    grid_level.setRenderMode(rviz::PointCloud::RM_BOXES);
    grid_level.setDimensions(cell_width, cell_width, cell_width);
    auto& cells_at_level = cells_per_level[height];
    grid_level.addPoints(&cells_at_level.front(), cells_at_level.size());
    frame_node_->attachObject(&grid_level);
  }

  frame_node_->setVisible(true);
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
}  // namespace wavemap::rviz_plugin

#include "wavemap_rviz_plugin/wavemap_octree_visual.h"

#include <OGRE/OgreVector3.h>

namespace wavemap_rviz_plugin {
WavemapOctreeVisual::WavemapOctreeVisual(Ogre::SceneManager* scene_manager,
                                         Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;

  // Ogre::SceneNodes form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent. Ogre does the math of combining those transforms when it
  // is time to render.
  // Here we create a node to store the pose of the WavemapOctree's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();
}

WavemapOctreeVisual::~WavemapOctreeVisual() {
  // Destroy the frame node since we don't need it anymore
  scene_manager_->destroySceneNode(frame_node_);
}

void WavemapOctreeVisual::setOctree(const Octree& octree,
                                    float occupancy_threshold_log_odds) {
  // TODO(victorr): Implement. The code should be quite similar to what was
  //                previously done in wavemap_common_ros' visualization utils.
  // std::vector<rviz::PointCloud::Point> points;
  // boxes_->setRenderMode(rviz::PointCloud::RM_BOXES);
  // boxes_->setDimensions(scale.x, scale.y, scale.z);
  // frame_node_->attachObject(boxes_.get());

  frame_node_->setVisible(true);
}

// Position and orientation are passed through to the SceneNode
void WavemapOctreeVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void WavemapOctreeVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}
}  // namespace wavemap_rviz_plugin

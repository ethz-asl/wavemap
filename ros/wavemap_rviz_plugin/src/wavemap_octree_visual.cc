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

  //  // We create the arrow object within the frame node so that we can
  //  // set its position and direction relative to its header frame.
  //  boxes_.reset(new rviz::PointCloud(scene_manager_, frame_node_));
}

WavemapOctreeVisual::~WavemapOctreeVisual() {
  // Destroy the frame node since we don't need it anymore
  scene_manager_->destroySceneNode(frame_node_);
}

void WavemapOctreeVisual::setMessage(
    const wavemap_msgs::Octree::ConstPtr& msg) {
  //  const geometry_msgs::Vector3& a = msg->linear_acceleration;
  //
  //  // Convert the geometry_msgs::Vector3 to an Ogre::Vector3
  //  Ogre::Vector3 acc(a.x, a.y, a.z);
  //
  //  // Find the magnitude of the acceleration vector
  //  float length = acc.length();
  //
  //  // Scale the arrow's thickness in each dimension along with its length
  //  Ogre::Vector3 scale(length, length, length);
  //  boxes_->setScale(scale);
  //
  //  // Set the orientation of the arrow to match the direction of the
  //  // acceleration vector.
  //  boxes_->setDirection(acc);
}

// Position and orientation are passed through to the SceneNode
void WavemapOctreeVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void WavemapOctreeVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void WavemapOctreeVisual::setOccupancyThreshold(float threshold_log_odds) {
  threshold_log_odds_ = threshold_log_odds;
  // TODO(victorr): Update the visuals
}
}  // namespace wavemap_rviz_plugin

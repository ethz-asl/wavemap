#ifndef ROS_WAVEMAP_RVIZ_PLUGIN_INCLUDE_WAVEMAP_RVIZ_PLUGIN_WAVEMAP_OCTREE_VISUAL_H_
#define ROS_WAVEMAP_RVIZ_PLUGIN_INCLUDE_WAVEMAP_RVIZ_PLUGIN_WAVEMAP_OCTREE_VISUAL_H_

#include <memory>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <wavemap_msgs/Octree.h>

namespace wavemap_rviz_plugin {

// Each instance of WavemapOctreeVisual represents the visualization of a single
// wavemap_msgs::Octree message.
class WavemapOctreeVisual {
 public:
  // Constructor. Creates the visual elements and puts them into the
  // scene, in an unconfigured state.
  WavemapOctreeVisual(Ogre::SceneManager* scene_manager,
                      Ogre::SceneNode* parent_node);

  // Destructor. Removes the visual elements from the scene
  virtual ~WavemapOctreeVisual();

  // Configure the visual to show the data in the message
  void setMessage(const wavemap_msgs::Octree::ConstPtr& msg);

  // Set the pose of the coordinate frame the message refers to
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the WavemapOctree message.
  void setOccupancyThreshold(float threshold_log_odds);

 private:
  // The object implementing the grid visuals
  std::unique_ptr<rviz::PointCloud> boxes_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the WavemapOctree message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the `frame_node_`.
  Ogre::SceneManager* scene_manager_;

  float threshold_log_odds_ = 0.f;
};
}  // namespace wavemap_rviz_plugin

#endif  // ROS_WAVEMAP_RVIZ_PLUGIN_INCLUDE_WAVEMAP_RVIZ_PLUGIN_WAVEMAP_OCTREE_VISUAL_H_

#ifndef WAVEMAP_RVIZ_PLUGIN_MULTI_RESOLUTION_SLICE_VISUAL_H_
#define WAVEMAP_RVIZ_PLUGIN_MULTI_RESOLUTION_SLICE_VISUAL_H_

#include <memory>
#include <vector>

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <wavemap_3d/data_structure/volumetric_data_structure_3d.h>

namespace wavemap::rviz_plugin {
// Each instance of MultiResolutionGridVisual represents the visualization of a
// map's leaves as squares whose sizes match their height in the tree.
class MultiResolutionSliceVisual {
 public:
  // Constructor. Creates the visual elements and puts them into the
  // scene, in an unconfigured state.
  MultiResolutionSliceVisual(Ogre::SceneManager* scene_manager,
                             Ogre::SceneNode* parent_node);

  // Destructor. Removes the visual elements from the scene
  virtual ~MultiResolutionSliceVisual();

  // Configure the visual to show the data in the message
  void loadMap(const VolumetricDataStructure3D& map,
               FloatingPoint min_occupancy_log_odds,
               FloatingPoint max_occupancy_log_odds, FloatingPoint slice_height,
               FloatingPoint alpha);

  // Set the pose of the coordinate frame the message refers to
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private:
  // The object implementing the grid visuals
  std::vector<rviz::PointCloud> grid_levels_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the WavemapOctree message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the `frame_node_`.
  Ogre::SceneManager* scene_manager_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_MULTI_RESOLUTION_SLICE_VISUAL_H_

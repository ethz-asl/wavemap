#ifndef WAVEMAP_RVIZ_PLUGIN_VISUALS_MESH_VISUAL_H_
#define WAVEMAP_RVIZ_PLUGIN_VISUALS_MESH_VISUAL_H_

#include <memory>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"

namespace wavemap::rviz_plugin {
// Each instance of MeshVisual represents the visualization of a map's
// iso-surface as a mesh.
class MeshVisual {
 public:
  // Constructor. Creates the visual elements and puts them into the
  // scene, in an unconfigured state.
  MeshVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  // Destructor. Removes the visual elements from the scene
  virtual ~MeshVisual();

  // Configure the visual to show the data in the message
  void loadMap(const VolumetricDataStructure3D& map,
               FloatingPoint min_occupancy_log_odds,
               FloatingPoint max_occupancy_log_odds, FloatingPoint alpha);

  // Set the pose of the coordinate frame the message refers to
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private:
  // A SceneNode whose pose is set to match the coordinate frame of
  // the WavemapOctree message header.
  Ogre::SceneNode* const frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the `frame_node_`.
  Ogre::SceneManager* const scene_manager_;

  Ogre::ManualObject* const mesh_object_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_VISUALS_MESH_VISUAL_H_
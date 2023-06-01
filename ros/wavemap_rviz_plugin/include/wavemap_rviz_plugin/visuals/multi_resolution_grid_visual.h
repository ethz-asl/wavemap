#ifndef WAVEMAP_RVIZ_PLUGIN_VISUALS_MULTI_RESOLUTION_GRID_VISUAL_H_
#define WAVEMAP_RVIZ_PLUGIN_VISUALS_MULTI_RESOLUTION_GRID_VISUAL_H_

#include <memory>
#include <vector>

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>

namespace wavemap::rviz_plugin {
// Each instance of MultiResolutionGridVisual represents the visualization of a
// map's leaves as cubes whose sizes match their height in the tree.
class MultiResolutionGridVisual {
 public:
  // Constructor. Creates the visual elements and puts them into the
  // scene, in an unconfigured state.
  MultiResolutionGridVisual(Ogre::SceneManager* scene_manager,
                            Ogre::SceneNode* parent_node);

  // Destructor. Removes the visual elements from the scene
  virtual ~MultiResolutionGridVisual();

  // Configure the visual to show the data in the message
  void loadMap(const VolumetricDataStructureBase& map,
               FloatingPoint min_occupancy_log_odds,
               FloatingPoint max_occupancy_log_odds, FloatingPoint alpha);

  // Set the pose of the coordinate frame the message refers to
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private:
  enum class ColorBy { kProbability, kPosition } kColorBy = ColorBy::kPosition;

  // The object implementing the grid visuals
  std::vector<std::unique_ptr<rviz::PointCloud>> grid_levels_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the WavemapOctree message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the `frame_node_`.
  Ogre::SceneManager* scene_manager_;

  // Map a voxel's log-odds value to a color (grey value)
  static Ogre::ColourValue logOddsToColor(FloatingPoint log_odds);

  // Map a voxel's position to a color (HSV color map)
  static Ogre::ColourValue positionToColor(const Point3D& center_point);
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_VISUALS_MULTI_RESOLUTION_GRID_VISUAL_H_

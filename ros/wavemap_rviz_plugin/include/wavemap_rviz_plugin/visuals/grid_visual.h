#ifndef WAVEMAP_RVIZ_PLUGIN_VISUALS_GRID_VISUAL_H_
#define WAVEMAP_RVIZ_PLUGIN_VISUALS_GRID_VISUAL_H_

#ifndef Q_MOC_RUN
#include <memory>
#include <vector>

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#endif

namespace wavemap::rviz_plugin {
// Each instance of MultiResolutionGridVisual represents the visualization of a
// map's leaves as cubes whose sizes match their height in the tree.
class GridVisual : public QObject {
  Q_OBJECT
 public:  // NOLINT
  // Constructor. Creates the visual elements and puts them into the
  // scene, in an unconfigured state.
  GridVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
             rviz::Property* submenu_root_property,
             const std::shared_ptr<std::shared_mutex> map_mutex,
             const VolumetricDataStructureBase::ConstPtr map);

  // Destructor. Removes the visual elements from the scene
  virtual ~GridVisual();

  void reset() { grid_levels_.clear(); }

  // Set the pose of the coordinate frame the message refers to
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private Q_SLOTS:  // NOLINT
  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties
  void update();
  void updateOpacity();
  void updateVisibility();

 private:
  enum class ColorBy { kProbability, kPosition } kColorBy = ColorBy::kPosition;

  // Read only shared pointer to the map, owned by WavemapMapDisplay
  const std::shared_ptr<std::shared_mutex> map_mutex_;
  const VolumetricDataStructureBase::ConstPtr map_;

  // The object implementing the grid visuals
  std::vector<std::unique_ptr<rviz::PointCloud>> grid_levels_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the `frame_node_`.
  Ogre::SceneManager* scene_manager_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the WavemapOctree message header.
  Ogre::SceneNode* frame_node_;

  // Map a voxel's log-odds value to a color (grey value)
  static Ogre::ColourValue logOddsToColor(FloatingPoint log_odds);

  // Map a voxel's position to a color (HSV color map)
  static Ogre::ColourValue positionToColor(const Point3D& center_point);

  // User-editable property variables, contained in the visual's submenu
  rviz::BoolProperty visibility_property_;
  rviz::FloatProperty min_occupancy_threshold_property_;
  rviz::FloatProperty max_occupancy_threshold_property_;
  rviz::FloatProperty opacity_property_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_VISUALS_GRID_VISUAL_H_

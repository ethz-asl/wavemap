#ifndef WAVEMAP_RVIZ_PLUGIN_VISUALS_SLICE_VISUAL_H_
#define WAVEMAP_RVIZ_PLUGIN_VISUALS_SLICE_VISUAL_H_

#ifndef Q_MOC_RUN
#include <memory>
#include <vector>

#include <OGRE/Ogre.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>

#include "wavemap_rviz_plugin/common.h"
#include "wavemap_rviz_plugin/visuals/cell_layer.h"
#endif

namespace wavemap::rviz_plugin {
// Each instance of MultiResolutionGridVisual represents the visualization of a
// map's leaves as squares whose sizes match their height in the tree.
class SliceVisual : public QObject {
  Q_OBJECT
 public:  // NOLINT
  // Constructor. Creates the visual elements and puts them into the
  // scene, in an unconfigured state.
  SliceVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
              rviz::Property* submenu_root_property,
              std::shared_ptr<MapAndMutex> map_and_mutex);

  // Destructor. Removes the visual elements from the scene.
  ~SliceVisual() override;

  void update();
  void clear() { grid_levels_.clear(); }

  // Set the pose of the coordinate frame the message refers to
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private Q_SLOTS:  // NOLINT
  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties
  void generalUpdateCallback() { update(); }
  void opacityUpdateCallback();

 private:
  // Shared pointer to the map, owned by WavemapMapDisplay
  const std::shared_ptr<MapAndMutex> map_and_mutex_;

  // The objects implementing the grid visuals
  std::vector<std::unique_ptr<CellLayer>> grid_levels_;

  // Material handling
  Ogre::MaterialPtr slice_cell_material_;
  void setAlpha(FloatingPoint alpha);

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the `frame_node_`.
  Ogre::SceneManager* scene_manager_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the WavemapOctree message header.
  Ogre::SceneNode* frame_node_;

  // User-editable property variables, contained in the visual's submenu
  rviz::BoolProperty visibility_property_;
  rviz::FloatProperty min_occupancy_threshold_property_;
  rviz::FloatProperty max_occupancy_threshold_property_;
  rviz::FloatProperty slice_height_property_;
  rviz::FloatProperty opacity_property_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_VISUALS_SLICE_VISUAL_H_

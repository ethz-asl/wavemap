#ifndef WAVEMAP_RVIZ_PLUGIN_VISUALS_GRID_VISUAL_H_
#define WAVEMAP_RVIZ_PLUGIN_VISUALS_GRID_VISUAL_H_

#ifndef Q_MOC_RUN
#include <memory>
#include <unordered_map>
#include <vector>

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/view_manager.h>
#include <wavemap/config/type_selector.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/indexing/index_hashes.h>
#include <wavemap/utils/time.h>

#include "wavemap_rviz_plugin/common.h"
#include "wavemap_rviz_plugin/visuals/grid_layer.h"
#endif

namespace wavemap::rviz_plugin {
struct ColorMode : public TypeSelector<ColorMode> {
  using TypeSelector<ColorMode>::TypeSelector;

  enum Id : TypeId { kHeight, kProbability, kConstant };

  static constexpr std::array names = {"Height", "Probability", "Constant"};
};

// Each instance of MultiResolutionGridVisual represents the visualization of a
// map's leaves as cubes whose sizes match their height in the tree.
class GridVisual : public QObject {
  Q_OBJECT
 public:  // NOLINT
  // Constructor. Creates the visual elements and puts them into the
  // scene, in an unconfigured state.
  GridVisual(Ogre::SceneManager* scene_manager, rviz::ViewManager* view_manager,
             Ogre::SceneNode* parent_node,
             rviz::Property* submenu_root_property,
             const std::shared_ptr<MapAndMutex> map_and_mutex);

  // Destructor. Removes the visual elements from the scene.
  ~GridVisual() override;

  void updateMap(bool redraw_all = false);

  void clear() { block_grids_.clear(); }

  // Set the pose of the coordinate frame the message refers to
  void setFramePosition(const Ogre::Vector3& position);

  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private Q_SLOTS:  // NOLINT
  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties
  void thresholdUpdateCallback() { updateMap(true); }
  void terminationHeightUpdateCallback() { force_lod_update_ = true; }
  void visibilityUpdateCallback();
  void opacityUpdateCallback();
  void colorModeUpdateCallback();

 private:
  ColorMode color_mode_ = ColorMode::kHeight;

  // Read only shared pointer to the map, owned by WavemapMapDisplay
  const std::shared_ptr<MapAndMutex> map_and_mutex_;

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
  rviz::IntProperty termination_height_property_;
  rviz::FloatProperty opacity_property_;
  rviz::EnumProperty color_mode_property_;
  rviz::Property frame_rate_properties_;
  rviz::IntProperty num_queued_blocks_indicator_;
  rviz::IntProperty max_ms_per_frame_property_;

  // The objects implementing the grid visuals
  Timestamp last_update_time_{};
  std::unordered_map<Index3D, IndexElement, Index3DHash> block_update_queue_;
  using MultiResGrid = std::vector<std::unique_ptr<GridLayer>>;
  std::unordered_map<Index3D, MultiResGrid, Index3DHash> block_grids_;
  Ogre::MaterialPtr grid_cell_material_;

  bool force_lod_update_ = true;
  float lod_update_distance_threshold_{0.1f};
  Ogre::Vector3 last_lod_update_position_{};
  void updateLOD(Ogre::Camera* cam);

  void processBlockUpdateQueue();

  using GridLayerList = std::vector<std::vector<GridCell>>;
  void getLeafCentersAndColors(int tree_height, FloatingPoint min_cell_width,
                               FloatingPoint min_occupancy_log_odds,
                               FloatingPoint max_occupancy_log_odds,
                               const OctreeIndex& cell_index,
                               FloatingPoint cell_log_odds,
                               GridLayerList& cells_per_level);
  void drawMultiResGrid(IndexElement tree_height, FloatingPoint min_cell_width,
                        const Index3D& block_index, FloatingPoint alpha,
                        GridLayerList& cells_per_level,
                        MultiResGrid& multi_res_grid);

  // Map a voxel's log-odds value to a color (grey value)
  static Ogre::ColourValue logOddsToColor(FloatingPoint log_odds);

  // Map a voxel's position to a color (HSV color map)
  static Ogre::ColourValue positionToColor(const Point3D& center_point);
};

template <typename CallbackT>
class ViewportCamChangedListener : public Ogre::Viewport::Listener {
 public:
  explicit ViewportCamChangedListener(CallbackT callback)
      : callback_(callback) {}

  void viewportCameraChanged(Ogre::Viewport* viewport) override {
    std::invoke(callback_, viewport);
  }

 private:
  CallbackT callback_;
};

template <typename CallbackT>
class CamPrerenderListener : public Ogre::Camera::Listener {
 public:
  explicit CamPrerenderListener(CallbackT callback) : callback_(callback) {}
  void cameraPreRenderScene(Ogre::Camera* cam) override {
    std::invoke(callback_, cam);
  }

 private:
  CallbackT callback_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_VISUALS_GRID_VISUAL_H_

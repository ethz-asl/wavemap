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
#include <wavemap/utils/time/time.h>

#include "wavemap_rviz_plugin/common.h"
#include "wavemap_rviz_plugin/utils/color_conversions.h"
#include "wavemap_rviz_plugin/utils/listeners.h"
#include "wavemap_rviz_plugin/visuals/cell_selector.h"
#include "wavemap_rviz_plugin/visuals/grid_layer.h"
#endif

namespace wavemap::rviz_plugin {
struct ColorMode : public TypeSelector<ColorMode> {
  using TypeSelector<ColorMode>::TypeSelector;

  enum Id : TypeId { kHeight, kProbability, kFlat };

  static constexpr std::array names = {"Height", "Probability", "Flat"};
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
             std::shared_ptr<MapAndMutex> map_and_mutex);

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
  void visibilityUpdateCallback();
  void terminationHeightUpdateCallback() { force_lod_update_ = true; }
  void opacityUpdateCallback();
  void colorModeUpdateCallback();
  void flatColorUpdateCallback();

 private:
  ColorMode grid_color_mode_ = ColorMode::kHeight;
  Ogre::ColourValue grid_flat_color_ = Ogre::ColourValue::Blue;

  // Shared pointer to the map, owned by WavemapMapDisplay
  const std::shared_ptr<MapAndMutex> map_and_mutex_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the `frame_node_`.
  Ogre::SceneManager* scene_manager_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the WavemapOctree message header.
  Ogre::SceneNode* frame_node_;

  // User-editable property variables, contained in the visual's submenu
  // Grid visibility
  rviz::BoolProperty visibility_property_;
  // Cell selection
  CellSelector cell_selector_;
  rviz::IntProperty termination_height_property_;
  // Colors
  rviz::FloatProperty opacity_property_;
  rviz::EnumProperty color_mode_property_;
  rviz::ColorProperty flat_color_property_;
  // Frame-rate stats
  rviz::Property frame_rate_properties_;
  rviz::IntProperty num_queued_blocks_indicator_;
  rviz::IntProperty max_ms_per_frame_property_;

  // The objects implementing the grid visuals
  using MultiResGrid = std::vector<std::unique_ptr<GridLayer>>;
  std::unordered_map<Index3D, MultiResGrid, Index3DHash> block_grids_;
  Ogre::MaterialPtr grid_cell_material_;

  // Level of Detail control
  std::unique_ptr<ViewportPrerenderListener> prerender_listener_;
  void prerenderCallback(Ogre::Camera* active_camera);
  float lod_update_distance_threshold_ = 0.1f;
  Ogre::Vector3 camera_position_at_last_lod_update_{};
  bool force_lod_update_ = true;
  void updateLOD(const Point3D& camera_position);
  static NdtreeIndexElement computeRecommendedBlockLodHeight(
      FloatingPoint distance_to_cam, FloatingPoint min_cell_width,
      NdtreeIndexElement min_height, NdtreeIndexElement max_height);
  std::optional<NdtreeIndexElement> getCurrentBlockLodHeight(
      IndexElement map_tree_height, const Index3D& block_idx);

  // Drawing related methods
  using GridLayerList = std::vector<std::vector<GridCell>>;
  void appendLeafCenterAndColor(int tree_height, FloatingPoint min_cell_width,
                                const OctreeIndex& cell_index,
                                FloatingPoint cell_log_odds,
                                GridLayerList& cells_per_level);
  void drawMultiResolutionGrid(IndexElement tree_height,
                               FloatingPoint min_cell_width,
                               const Index3D& block_index, FloatingPoint alpha,
                               GridLayerList& cells_per_level,
                               MultiResGrid& multi_res_grid);

  // Block update queue
  // NOTE: Instead of performing all the block updates at once whenever the map
  //       is updated or the LOD levels change (due to camera motion), we add
  //       the changed blocks to the block_update_queue_. Blocks are then popped
  //       from the queue and updated until max_ms_per_frame_property_ is
  //       reached. Any blocks that have not yet been processed will then be
  //       updated in the next prerender cycle. This avoids excessive frame rate
  //       drops when large changes occur.
  Timestamp last_update_time_{};
  std::unordered_map<Index3D, IndexElement, Index3DHash> block_update_queue_;
  void processBlockUpdateQueue(const Point3D& camera_position);
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_VISUALS_GRID_VISUAL_H_

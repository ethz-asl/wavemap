#ifndef WAVEMAP_RVIZ_PLUGIN_VISUALS_VOXEL_VISUAL_H_
#define WAVEMAP_RVIZ_PLUGIN_VISUALS_VOXEL_VISUAL_H_

#ifndef Q_MOC_RUN
#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <wavemap_msgs/DiscreteLayerCategory.h>
#include <wavemap_msgs/DiscreteLayerCell.h>
#include <wavemap_msgs/LayeredHashedWaveletOctree.h>
#include <wavemap_msgs/LayeredHashedWaveletOctreeBlock.h>

#include <OGRE/Ogre.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/properties/bool_property.h>
#include <rviz/config.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/view_manager.h>
#include <wavemap/core/config/type_selector.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/time/time.h>

#include "wavemap_rviz_plugin/common.h"
#include "wavemap_rviz_plugin/utils/color_conversions.h"
#include "wavemap_rviz_plugin/utils/listeners.h"
#include "wavemap_rviz_plugin/visuals/cell_layer.h"
#include "wavemap_rviz_plugin/visuals/cell_selector.h"
#endif

namespace wavemap::rviz_plugin {
struct VoxelColorMode : public TypeSelector<VoxelColorMode> {
  using TypeSelector<VoxelColorMode>::TypeSelector;

  enum Id : TypeId { kHeight, kProbability, kFlat };

  static constexpr std::array names = {"Height", "Probability", "Flat"};
};

// Each instance of VoxelVisual represents the map's leaves
// as voxels whose sizes match their height in the tree.
class VoxelVisual : public QObject {
  Q_OBJECT
 public:  // NOLINT
  // Constructor. Creates the visual elements and puts them into the
  // scene, in an unconfigured state.
  VoxelVisual(Ogre::SceneManager* scene_manager,
              rviz::ViewManager* view_manager, Ogre::SceneNode* parent_node,
              rviz::Property* submenu_root_property,
              std::shared_ptr<MapAndMutex> map_and_mutex,
              std::function<void()> layer_color_changed_callback = {});

  // Destructor. Removes the visual elements from the scene.
  ~VoxelVisual() override;

  void applyDiscreteLayerPatch(
      const wavemap_msgs::DiscreteLayer& patch,
      FloatingPoint min_cell_width);

  void updateMap(bool redraw_all = false);

  void clear();

  int terminationHeight() const {
    return termination_height_property_.getInt();
  }
  void setTerminationHeight(int max_height, int height) {
    termination_height_property_.setMax(max_height);
    termination_height_property_.setInt(std::clamp(height, 0, max_height));
  }
  FloatingPoint scalarDisplayMin() const {
    return scalar_min_property_.getFloat();
  }
  FloatingPoint scalarDisplayMax() const {
    return scalar_max_property_.getFloat();
  }
  Ogre::ColourValue scalarLowColor() const { return scalar_low_color_; }
  VoxelColorMode colorMode() const { return voxel_color_mode_; }
  Ogre::ColourValue flatColor() const { return voxel_flat_color_; }
  Ogre::ColourValue scalarHighColor() const { return scalar_high_color_; }
  void configureLayerAppearance(
      const std::string& layer_name, const std::string& layer_type,
      bool is_discrete, FloatingPoint scalar_min, FloatingPoint scalar_max,
      bool has_low_color, const Ogre::ColourValue& low_color,
      bool has_high_color, const Ogre::ColourValue& high_color,
      const std::vector<wavemap_msgs::DiscreteLayerCategory>& categories = {});
  Ogre::ColourValue categoryColor(int value) const;
  bool categoryVisible(int value) const;
  Ogre::ColourValue boolColor(bool value) const;
  bool boolVisible(bool value) const;

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
  void layerColorUpdateCallback();

 private:
  VoxelColorMode voxel_color_mode_ = VoxelColorMode::kHeight;
  Ogre::ColourValue voxel_flat_color_ = Ogre::ColourValue::Blue;
  Ogre::ColourValue scalar_low_color_ = Ogre::ColourValue(0.f, 0.f, 1.f);
  Ogre::ColourValue scalar_high_color_ = Ogre::ColourValue(1.f, 1.f, 0.f);
  Ogre::ColourValue bool_true_color_ = Ogre::ColourValue(1.f, 0.05f, 0.05f);
  Ogre::ColourValue bool_false_color_ =
      Ogre::ColourValue(0.35f, 0.35f, 0.35f);

  std::string selected_color_layer_;
  std::unordered_map<int, Ogre::ColourValue> category_colors_;
  static Ogre::ColourValue defaultCategoryColor(int value);

  // Shared pointer to the map, owned by WavemapMapDisplay
  const std::shared_ptr<MapAndMutex> map_and_mutex_;
  const std::function<void()> layer_color_changed_callback_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the `frame_node_`.
  Ogre::SceneManager* scene_manager_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the WavemapOctree message header.
  Ogre::SceneNode* frame_node_;

  // User-editable property variables, contained in the visual's submenu
  // Visibility
  rviz::BoolProperty visibility_property_;
  // Cell selection
  CellSelector cell_selector_;
  rviz::IntProperty termination_height_property_;
  // Colors
  rviz::FloatProperty opacity_property_;
  rviz::EnumProperty color_mode_property_;
  rviz::ColorProperty flat_color_property_;
  rviz::FloatProperty scalar_min_property_;
  rviz::FloatProperty scalar_max_property_;
  // Frame-rate stats
  rviz::Property frame_rate_properties_;
  rviz::IntProperty num_queued_blocks_indicator_;
  rviz::IntProperty max_ms_per_frame_property_;

  // The objects implementing the voxel visuals
  using VoxelLayers = std::vector<std::unique_ptr<CellLayer>>;
  std::unordered_map<Index3D, VoxelLayers, Index3DHash> block_voxel_layers_map_;
  void detachVoxelLayers(VoxelLayers& voxel_layers);

  // Material handling
  Ogre::MaterialPtr voxel_material_;
  void setAlpha(FloatingPoint alpha);

  // Level of Detail control
  std::unique_ptr<ViewportPrerenderListener> prerender_listener_;
  void prerenderCallback(Ogre::Camera* active_camera);
  float lod_update_distance_threshold_ = 0.1f;
  Ogre::Vector3 camera_position_at_last_lod_update_{};
  bool force_lod_update_ = true;
  void updateLOD(const Ogre::Camera& active_camera);
  static IndexElement computeRecommendedBlockLodHeight(
      const Ogre::Camera& active_camera, const OctreeIndex& block_index,
      FloatingPoint min_cell_width, IndexElement min_height,
      IndexElement max_height);
  std::optional<IndexElement> getCurrentBlockLodHeight(
      IndexElement map_tree_height, const Index3D& block_idx);

  // Drawing related methods
  using VoxelsPerLevel = std::vector<std::vector<Cell>>;
  void appendLeafCenterAndColor(int tree_height, FloatingPoint min_cell_width,
                                const OctreeIndex& cell_index,
                                FloatingPoint cell_log_odds,
                                VoxelsPerLevel& voxels_per_level);
  void appendLayeredLeafCenterAndColor(const LayeredMapInterface& layered_map,
                                       const std::string& selected_layer_name,
                                       int tree_height,
                                       FloatingPoint min_cell_width,
                                       const OctreeIndex& cell_index,
                                       FloatingPoint cell_log_odds,
                                       VoxelsPerLevel& voxels_per_level);
  void drawMultiResolutionVoxels(IndexElement tree_height,
                                 FloatingPoint min_cell_width,
                                 const Index3D& block_index,
                                 FloatingPoint alpha,
                                 VoxelsPerLevel& voxels_per_level,
                                 VoxelLayers& voxel_layer_visuals);
  void drawLayeredMapOccupancy(const LayeredMapInterface& layered_map);
  void drawLayeredMapOccupancy(
      const wavemap_msgs::LayeredHashedWaveletOctree& layered_map_msg);
  bool getGenericContinuousLayerColor(
      const wavemap_msgs::LayeredHashedWaveletOctree& layered_map_msg,
      const std::vector<std::vector<FloatingPoint>>& layer_values,
      Ogre::ColourValue& color) const;
  void drawDiscreteLayer(const wavemap_msgs::DiscreteLayer& discrete_layer_msg,
                         FloatingPoint min_cell_width);
  void rebuildDiscreteBatches(
      const std::unordered_set<Index3D, Index3DHash>& batch_indices,
      FloatingPoint min_cell_width);
  static Index3D discreteRenderBatchIndex(const Index3D& parent_index);

  std::optional<wavemap_msgs::DiscreteLayer> discrete_layer_metadata_;
  std::unordered_map<Index3D, wavemap_msgs::DiscreteLayerCell, Index3DHash>
      discrete_parent_cells_;
  std::unordered_map<Index3D,
                     std::unordered_set<Index3D, Index3DHash>, Index3DHash>
      discrete_batch_parents_;
  bool getDiscreteCellColor(const wavemap_msgs::DiscreteLayer& layer_msg,
                            const wavemap_msgs::DiscreteLayerCell& cell_msg,
                            int exception_index, bool is_exception,
                            Ogre::ColourValue& color) const;
  void appendLayeredBlockOccupancy(
      const wavemap_msgs::LayeredHashedWaveletOctree& layered_map_msg,
      const wavemap_msgs::LayeredHashedWaveletOctreeBlock& block_msg,
      IndexElement termination_height, VoxelsPerLevel& voxels_per_level);

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

#endif  // WAVEMAP_RVIZ_PLUGIN_VISUALS_VOXEL_VISUAL_H_

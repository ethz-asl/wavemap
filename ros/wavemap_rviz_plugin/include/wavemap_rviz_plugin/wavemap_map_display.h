#ifndef WAVEMAP_RVIZ_PLUGIN_WAVEMAP_MAP_DISPLAY_H_
#define WAVEMAP_RVIZ_PLUGIN_WAVEMAP_MAP_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <filesystem>
#include <memory>
#include <string>

#include <rviz/message_filter_display.h>
#include <rviz/properties/property.h>
#include <wavemap_msgs/Map.h>

#include "wavemap_rviz_plugin/common.h"
#include "wavemap_rviz_plugin/utils/button_property.h"
#include "wavemap_rviz_plugin/visuals/slice_visual.h"
#include "wavemap_rviz_plugin/visuals/voxel_visual.h"
#endif

namespace wavemap::rviz_plugin {
struct SourceMode : public TypeSelector<SourceMode> {
  using TypeSelector<SourceMode>::TypeSelector;

  enum Id : TypeId { kFromTopic, kFromFile };

  static constexpr std::array names = {"Topic", "File"};
};

// The WavemapMapDisplay class implements the editable parameters and Display
// subclass machinery. The visuals themselves are represented by a separate
// class, WavemapMapVisual. The idiom for the visuals is that when the
// objects exist, they appear in the scene, and when they are deleted, they
// disappear.
class WavemapMapDisplay : public rviz::MessageFilterDisplay<wavemap_msgs::Map> {
  Q_OBJECT
 public:  // NOLINT
  // Constructor. pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  WavemapMapDisplay();
  ~WavemapMapDisplay() override = default;

 protected:
  void onInitialize() override;

  // A helper to clear this display back to the initial state.
  void reset() override;

 private Q_SLOTS:  // NOLINT
  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties
  void updateSourceModeCallback();
  void requestWavemapServerResetCallback();
  void requestWholeMapCallback();
  void loadMapFromDiskCallback();

 private:
  SourceMode source_mode_ = SourceMode::kFromTopic;

  bool hasMap();
  void clearMap();
  bool loadMapFromDisk(const std::filesystem::path& filepath);
  void updateVisuals(bool redraw_all = false);

  // Function to handle an incoming ROS message
  void processMessage(const wavemap_msgs::Map::ConstPtr& map_msg) override;

  // Storage and message parsers for the map
  const std::shared_ptr<MapAndMutex> map_and_mutex_ =
      std::make_shared<MapAndMutex>();
  void updateMapFromRosMsg(const wavemap_msgs::Map& map_msg);

  // Submenus for each visual's properties
  rviz::EnumProperty source_mode_property_{"Source", "",
                                           "Where to load the map from.", this,
                                           SLOT(updateSourceModeCallback())};
  ButtonProperty request_wavemap_server_reset_property_{
      "Reset server",
      "Request",
      "Send a request to the wavemap_server to reset the map.",
      this,
      SLOT(requestWavemapServerResetCallback()),
      this};
  ButtonProperty request_whole_map_property_{
      "Map update",
      "Request full",
      "Send a request to the wavemap_server to republish the whole map, "
      "instead of only increments.",
      this,
      SLOT(requestWholeMapCallback()),
      this};
  ButtonProperty load_map_from_disk_property_{
      "Loaded map",
      "Choose file",
      "Open a file dialog to choose and load a map from disk.",
      this,
      SLOT(loadMapFromDiskCallback()),
      this};
  rviz::Property voxel_visual_properties_{
      "Render voxels", QVariant(),
      "Properties for the voxel-based visualization.", this};
  rviz::Property slice_visual_properties_{
      "Render slice", QVariant(), "Properties for the slice visualization.",
      this};

  // Service client to call wavemap's request_whole_map service
  inline static const std::string kResetWavemapServerService = "reset_map";
  inline static const std::string kRepublishWholeMapService =
      "republish_whole_map";
  static std::optional<std::string> resolveWavemapServerNamespaceFromMapTopic(
      const std::string& map_topic, const std::string& child_topic = "");
  ros::ServiceClient request_wavemap_server_reset_client_;
  ros::ServiceClient request_whole_map_client_;

  // Storage for the visuals
  // NOTE: Visuals are enabled when they are allocated, and automatically
  //       removed from the scene when destructed.
  std::unique_ptr<VoxelVisual> voxel_visual_;
  std::unique_ptr<SliceVisual> slice_visual_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_WAVEMAP_MAP_DISPLAY_H_

#ifndef WAVEMAP_RVIZ_PLUGIN_WAVEMAP_MAP_DISPLAY_H_
#define WAVEMAP_RVIZ_PLUGIN_WAVEMAP_MAP_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <memory>

#include <rviz/message_filter_display.h>
#include <rviz/properties/property.h>
#include <wavemap_msgs/Map.h>

#include "wavemap_rviz_plugin/common.h"
#include "wavemap_rviz_plugin/visuals/grid_visual.h"
#include "wavemap_rviz_plugin/visuals/slice_visual.h"
#endif

namespace wavemap::rviz_plugin {
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
  WavemapMapDisplay() = default;
  ~WavemapMapDisplay() override = default;

 protected:
  void onInitialize() override;

  // A helper to clear this display back to the initial state.
  void reset() override;

 private Q_SLOTS:  // NOLINT
  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties
  void requestWholeMap();

 private:
  // Function to handle an incoming ROS message
  void processMessage(const wavemap_msgs::Map::ConstPtr& map_msg) override;

  // Storage and message parsers for the map
  const std::shared_ptr<MapAndMutex> map_and_mutex_ =
      std::make_shared<MapAndMutex>();
  void updateMapFromRosMsg(const wavemap_msgs::Map& map_msg);

  // Submenus for each visual's properties
  rviz::Property grid_visual_properties_{
      "Show grid", QVariant(), "Properties for the grid visualization.", this};
  rviz::Property slice_visual_properties_{
      "Show slice", QVariant(), "Properties for the slice visualization.",
      this};
  rviz::BoolProperty request_whole_map_property_{
      "Request whole map", false,
      "Send a request to the wavemap_server to republish the whole map, "
      "instead of only increments.",
      this, SLOT(requestWholeMap())};

  // Service client to call wavemap's request_whole_map service
  ros::ServiceClient request_whole_map_client_;

  // Storage for the visuals
  // NOTE: Visuals are enabled when they are allocated, and automatically
  //       removed from the scene when destructed.
  std::unique_ptr<GridVisual> grid_visual_;
  std::unique_ptr<SliceVisual> slice_visual_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_WAVEMAP_MAP_DISPLAY_H_

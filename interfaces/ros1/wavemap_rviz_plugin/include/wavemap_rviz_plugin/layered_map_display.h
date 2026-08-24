#ifndef WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_DISPLAY_H_
#define WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <OGRE/Overlay/OgreOverlayPrerequisites.h>

#include <boost/shared_ptr.hpp>

#include <rviz/message_filter_display.h>
#include <rviz/config.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/property.h>
#include <wavemap_msgs/LayeredMap.h>
#include <pluginlib/class_loader.h>

#include "wavemap_rviz_plugin/common.h"
#include "wavemap_rviz_plugin/layered_map_factory.h"
#include "wavemap_rviz_plugin/utils/button_property.h"
#include "wavemap_rviz_plugin/visuals/voxel_visual.h"
#endif

namespace Ogre {
class OverlayElement;
class TextAreaOverlayElement;
}

namespace wavemap::rviz_plugin {
struct LayeredMapSourceMode : public TypeSelector<LayeredMapSourceMode> {
  using TypeSelector<LayeredMapSourceMode>::TypeSelector;

  enum Id : TypeId { kFromTopic, kFromFile };

  static constexpr std::array names = {"Topic", "File"};
};

class LayeredMapDisplay
    : public rviz::MessageFilterDisplay<wavemap_msgs::LayeredMap> {
  Q_OBJECT
 public:  // NOLINT
  LayeredMapDisplay();
  ~LayeredMapDisplay() override;

 protected:
  void onInitialize() override;
  void reset() override;

 private Q_SLOTS:  // NOLINT
  void updateSourceModeCallback();
  void loadMapFromDiskCallback();
  void updateLayerSelectionCallback();

 private:
  struct DisplayLayer {
    enum class Source { kContinuous, kDiscrete };

    std::string name;
    std::string type;
    Source source = Source::kContinuous;
    std::vector<FloatingPoint> min_values;
    std::vector<FloatingPoint> max_values;
  };

  std::vector<DisplayLayer> available_layers_;
  LayeredMapSourceMode source_mode_ = LayeredMapSourceMode::kFromTopic;
  std::string selected_layer_name_ = "occupancy";
  int continuous_termination_height_ = 0;
  int discrete_termination_height_ = 1;
  std::optional<wavemap_msgs::LayeredMap> latest_msg_;
  std::unique_ptr<pluginlib::ClassLoader<LayeredMapFactory>>
      layered_map_factory_loader_;
  std::vector<boost::shared_ptr<LayeredMapFactory>> layered_map_factories_;

  const std::shared_ptr<MapAndMutex> map_and_mutex_ =
      std::make_shared<MapAndMutex>();
  std::unique_ptr<VoxelVisual> voxel_visual_;

  rviz::EnumProperty source_mode_property_{"Source", "",
                                           "Where to load the layered map from.",
                                           this,
                                           SLOT(updateSourceModeCallback())};
  ButtonProperty load_map_from_disk_property_{
      "Loaded map",
      "Choose file",
      "Open a file dialog to choose and load a layered map from disk.",
      this,
      SLOT(loadMapFromDiskCallback()),
      this};
  rviz::Property voxel_visual_properties_{"Render voxels", QVariant(),
                                          "Properties for voxel rendering.",
                                          this};
  rviz::EnumProperty layer_property_{
      "Layer", "occupancy",
      "Layer to display. Populated from the received LayeredMap message.",
      &voxel_visual_properties_, SLOT(updateLayerSelectionCallback()), this};
  void processMessage(const wavemap_msgs::LayeredMap::ConstPtr& msg) override;
  bool loadMapFromDisk(const std::filesystem::path& filepath,
                       std::string* error_message = nullptr);
  void displayMessage(const wavemap_msgs::LayeredMap& msg);
  void clearStoredMap();
  void updateAvailableLayers(const wavemap_msgs::LayeredMap& msg);
  void updateStoredMapForSelectedLayer(const wavemap_msgs::LayeredMap& msg);
  void configureSelectedLayerAppearance(
      const wavemap_msgs::LayeredMap& msg);
  void loadLayeredMapFactories();
  void updateLegendOverlay(const wavemap_msgs::LayeredMap* msg = nullptr);
  void initializeLegendOverlay();
  void destroyLegendOverlay();
  void clearLegendOverlayElements();
  std::string makeLegendMaterial(const Ogre::ColourValue& color,
                                 const std::string& name_hint);
  void addLegendText(const std::string& text, float x, float y,
                     float char_height, const Ogre::ColourValue& color);
  void addLegendRow(const Ogre::ColourValue& color, const std::string& text,
                    float y);
  const DisplayLayer* selectedLayer() const;

  Ogre::Overlay* legend_overlay_ = nullptr;
  Ogre::OverlayContainer* legend_panel_ = nullptr;
  std::vector<Ogre::OverlayElement*> legend_elements_;
  std::vector<std::string> legend_material_names_;
  std::vector<std::string> legend_texture_names_;
  size_t legend_element_counter_ = 0u;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_DISPLAY_H_

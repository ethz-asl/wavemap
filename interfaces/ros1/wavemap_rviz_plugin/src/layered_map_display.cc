#include "wavemap_rviz_plugin/layered_map_display.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgrePass.h>
#include <OGRE/OgreResourceGroupManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/Overlay/OgreOverlay.h>
#include <OGRE/Overlay/OgreOverlayContainer.h>
#include <OGRE/Overlay/OgreOverlayManager.h>
#include <OGRE/Overlay/OgreTextAreaOverlayElement.h>
#include <pluginlib/class_list_macros.h>
#include <qfiledialog.h>
#include <QString>
#include <ros/console.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>
#include <wavemap/core/utils/profile/profiler_interface.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "example_layered_map_ros_config.h"
#include "layered_ros_converter.h"
#include "wavemap_rviz_plugin/layered_map_factory.h"
#include "wavemap_rviz_plugin/utils/alert_dialog.h"

namespace wavemap::rviz_plugin {
namespace {
struct ExampleContinuousLayerColorProvider {
  static bool getLayerColor(const std::string& layer_name,
                            const LayeredVoxel& voxel,
                            FloatingPoint /*occupancy*/,
                            Ogre::ColourValue& color) {
    if (layer_name == "color") {
      color = Ogre::ColourValue(std::clamp(voxel.data.rgb.r, 0.f, 1.f),
                                std::clamp(voxel.data.rgb.g, 0.f, 1.f),
                                std::clamp(voxel.data.rgb.b, 0.f, 1.f), 1.f);
      return true;
    }

    if (layer_name == "traversability") {
      const FloatingPoint t =
          std::clamp(voxel.data.traversability, 0.f, 1.f);
      color = Ogre::ColourValue(1.f - t, t, 0.15f, 1.f);
      return true;
    }

    return false;
  }
};

using ExampleContinuousMapFactory =
    TypedLayeredMapFactory<ContinuousWaveletMap, LayeredVoxelRosConverter,
                           ExampleContinuousLayerColorProvider>;

Ogre::ColourValue stableIntColor(int value) {
  const uint32_t hash = static_cast<uint32_t>(value) * 2654435761u;
  const float r = 0.25f +
                  0.75f * static_cast<float>((hash >> 16u) & 0xffu) / 255.f;
  const float g = 0.25f +
                  0.75f * static_cast<float>((hash >> 8u) & 0xffu) / 255.f;
  const float b = 0.25f + 0.75f * static_cast<float>(hash & 0xffu) / 255.f;
  return Ogre::ColourValue(r, g, b, 1.f);
}

std::set<int> collectIntValues(const wavemap_msgs::DiscreteLayer& layer) {
  std::set<int> values;
  for (const auto& cell : layer.cells) {
    values.insert(cell.dominant_int32);
    values.insert(cell.exception_int32_values.begin(),
                  cell.exception_int32_values.end());
  }
  return values;
}

std::set<bool> collectBoolValues(const wavemap_msgs::DiscreteLayer& layer) {
  std::set<bool> values;
  for (const auto& cell : layer.cells) {
    values.insert(cell.dominant_uint8 != 0u);
    for (const uint8_t exception_value : cell.exception_uint8_values) {
      values.insert(exception_value != 0u);
    }
  }
  return values;
}
}  // namespace

LayeredMapDisplay::LayeredMapDisplay() {
  source_mode_property_.clearOptions();
  for (const auto& name : LayeredMapSourceMode::names) {
    source_mode_property_.addOption(name);
  }
  source_mode_property_.setStringStd(source_mode_.toStr());
  load_map_from_disk_property_.setHidden(true);

  layer_property_.clearOptions();
  layer_property_.addOption(QString::fromStdString(selected_layer_name_));
  layer_property_.setStringStd(selected_layer_name_);
}

LayeredMapDisplay::~LayeredMapDisplay() {
  // Destroy Ogre visuals before Qt-owned RViz properties start tearing down.
  voxel_visual_.reset();
  destroyLegendOverlay();
}

void LayeredMapDisplay::onInitialize() {
  ProfilerZoneScoped;
  MFDClass::onInitialize();

  const int source_row = source_mode_property_.rowNumberInParent();
  const int topic_row = topic_property_->rowNumberInParent();
  if (0 <= topic_row && topic_row < source_row) {
    moveChild(source_row, topic_row);
  }

  if (topic_property_->getStdString().empty()) {
    topic_property_->setString("/wavemap/layered_map");
  }
  voxel_visual_ = std::make_unique<VoxelVisual>(
      scene_manager_, context_->getViewManager(), scene_node_,
      &voxel_visual_properties_, map_and_mutex_);
  initializeLegendOverlay();
}

void LayeredMapDisplay::reset() {
  ProfilerZoneScoped;
  MFDClass::reset();
  latest_msg_.reset();
  clearStoredMap();
  if (voxel_visual_) {
    voxel_visual_->clear();
  }
  updateLegendOverlay(nullptr);
}

void LayeredMapDisplay::processMessage(
    const wavemap_msgs::LayeredMap::ConstPtr& msg) {
  ProfilerZoneScoped;
  if (!msg) {
    ROS_WARN("Ignoring request to process non-existent layered map msg.");
    return;
  }

  displayMessage(*msg);
}

void LayeredMapDisplay::displayMessage(const wavemap_msgs::LayeredMap& msg) {
  latest_msg_ = msg;
  updateAvailableLayers(msg);
  updateLegendOverlay(&msg);
  updateStoredMapForSelectedLayer(msg);

  if (!voxel_visual_) {
    ROS_WARN("Voxel visual not initialized yet, skipping layered map message.");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg.header.frame_id,
                                                 msg.header.stamp,
                                                 position, orientation)) {
    ROS_WARN("Error transforming from frame '%s' to frame '%s'",
             msg.header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }
  voxel_visual_->setFramePosition(position);
  voxel_visual_->setFrameOrientation(orientation);
  voxel_visual_->updateMap(true);
}

bool LayeredMapDisplay::loadMapFromDisk(
    const std::filesystem::path& filepath, std::string* error_message) {
  ExampleLayeredMap map(ExampleLayeredMapConfig{});
  if (!ExampleLayeredMapIo::load(filepath, map, error_message)) {
    return false;
  }

  wavemap_msgs::LayeredMap msg;
  const std::string frame_id = fixed_frame_.isEmpty() ? "map" : fixed_frame_.toStdString();
  if (!convert::layeredMapToRosMsg<ExampleLayeredMap, LayeredVoxelRosConverter>(
          map, frame_id, ros::Time(0), msg)) {
    return false;
  }

  displayMessage(msg);
  return true;
}

void LayeredMapDisplay::clearStoredMap() {
  updateLegendOverlay(nullptr);
  std::scoped_lock lock(map_and_mutex_->mutex);
  map_and_mutex_->map.reset();
  map_and_mutex_->layered_map.reset();
  map_and_mutex_->layered_map_msg.reset();
  map_and_mutex_->discrete_layer_msg.reset();
}

void LayeredMapDisplay::updateAvailableLayers(
    const wavemap_msgs::LayeredMap& msg) {
  available_layers_.clear();
  available_layers_.push_back(
      {"occupancy", "float32", DisplayLayer::Source::kContinuous});

  if (msg.continuous_map.layered_hashed_wavelet_octree.size() == 1u) {
    const auto& continuous_msg =
        msg.continuous_map.layered_hashed_wavelet_octree.front();
    const size_t num_layers = std::min(continuous_msg.layer_names.size(),
                                       continuous_msg.layer_types.size());
    for (size_t layer_i = 0u; layer_i < num_layers; ++layer_i) {
      available_layers_.push_back({continuous_msg.layer_names[layer_i],
                                   continuous_msg.layer_types[layer_i],
                                   DisplayLayer::Source::kContinuous});
    }
  }

  for (const auto& discrete_layer : msg.discrete_layers) {
    available_layers_.push_back({discrete_layer.name, discrete_layer.value_type,
                                 DisplayLayer::Source::kDiscrete});
  }

  const bool selected_still_exists =
      std::any_of(available_layers_.begin(), available_layers_.end(),
                  [this](const DisplayLayer& layer) {
                    return layer.name == selected_layer_name_;
                  });
  if (!selected_still_exists) {
    selected_layer_name_ = available_layers_.empty() ? "occupancy"
                                                     : available_layers_.front().name;
  }

  layer_property_.clearOptions();
  for (const DisplayLayer& layer : available_layers_) {
    layer_property_.addOption(QString::fromStdString(layer.name));
  }
  layer_property_.setStringStd(selected_layer_name_);
}

const LayeredMapDisplay::DisplayLayer* LayeredMapDisplay::selectedLayer()
    const {
  const auto layer_it = std::find_if(
      available_layers_.begin(), available_layers_.end(),
      [this](const DisplayLayer& layer) { return layer.name == selected_layer_name_; });
  return layer_it == available_layers_.end() ? nullptr : &*layer_it;
}

void LayeredMapDisplay::updateStoredMapForSelectedLayer(
    const wavemap_msgs::LayeredMap& msg) {
  const DisplayLayer* layer = selectedLayer();
  const bool selected_is_discrete =
      layer && layer->source == DisplayLayer::Source::kDiscrete;

  std::scoped_lock lock(map_and_mutex_->mutex);
  map_and_mutex_->map.reset();
  map_and_mutex_->layered_map.reset();
  map_and_mutex_->layered_map_msg.reset();
  map_and_mutex_->discrete_layer_msg.reset();
  map_and_mutex_->selected_layer_name = selected_layer_name_;

  if (selected_is_discrete) {
    const auto discrete_it = std::find_if(
        msg.discrete_layers.begin(), msg.discrete_layers.end(),
        [this](const wavemap_msgs::DiscreteLayer& discrete_layer) {
          return discrete_layer.name == selected_layer_name_;
        });
    if (discrete_it != msg.discrete_layers.end()) {
      map_and_mutex_->discrete_layer_msg = *discrete_it;
      if (msg.continuous_map.layered_hashed_wavelet_octree.size() == 1u) {
        map_and_mutex_->discrete_layer_min_cell_width =
            msg.continuous_map.layered_hashed_wavelet_octree.front().min_cell_width;
      }
    }
    return;
  }

  if (msg.continuous_map.layered_hashed_wavelet_octree.size() == 1u) {
    const auto& continuous_msg =
        msg.continuous_map.layered_hashed_wavelet_octree.front();
    if (auto typed_map = ExampleContinuousMapFactory{}.tryCreate(continuous_msg)) {
      map_and_mutex_->layered_map = std::move(typed_map);
      return;
    }

    map_and_mutex_->layered_map_msg = continuous_msg;
    return;
  }

  MapBase::Ptr map;
  if (!convert::rosMsgToMap(msg.continuous_map, map)) {
    ROS_WARN("Failed to parse continuous map inside LayeredMap message.");
    return;
  }
  map_and_mutex_->map = std::move(map);
}

void LayeredMapDisplay::initializeLegendOverlay() {
  if (legend_overlay_) {
    return;
  }

  auto& overlay_manager = Ogre::OverlayManager::getSingleton();
  const std::string name_suffix = std::to_string(reinterpret_cast<uintptr_t>(this));
  legend_overlay_ = overlay_manager.create("LayeredMapLegendOverlay_" + name_suffix);
  legend_overlay_->setZOrder(650);

  legend_panel_ = static_cast<Ogre::OverlayContainer*>(
      overlay_manager.createOverlayElement(
          "Panel", "LayeredMapLegendPanel_" + name_suffix));
  legend_panel_->setMetricsMode(Ogre::GMM_PIXELS);
  legend_panel_->setHorizontalAlignment(Ogre::GHA_RIGHT);
  legend_panel_->setVerticalAlignment(Ogre::GVA_TOP);
  legend_panel_->setPosition(-430.f, 18.f);
  legend_panel_->setDimensions(410.f, 150.f);
  legend_panel_->setMaterialName(makeLegendMaterial(
      Ogre::ColourValue(0.92f, 0.94f, 0.96f, 0.88f), "background"));

  legend_overlay_->add2D(legend_panel_);
  legend_overlay_->show();
  updateLegendOverlay(nullptr);
}

void LayeredMapDisplay::clearLegendOverlayElements() {
  if (!legend_panel_) {
    return;
  }

  auto& overlay_manager = Ogre::OverlayManager::getSingleton();
  for (Ogre::OverlayElement* element : legend_elements_) {
    if (!element) {
      continue;
    }
    legend_panel_->removeChild(element->getName());
    overlay_manager.destroyOverlayElement(element);
  }
  legend_elements_.clear();
}

void LayeredMapDisplay::destroyLegendOverlay() {
  auto& overlay_manager = Ogre::OverlayManager::getSingleton();
  clearLegendOverlayElements();

  if (legend_overlay_ && legend_panel_) {
    legend_overlay_->remove2D(legend_panel_);
    overlay_manager.destroyOverlayElement(legend_panel_);
    legend_panel_ = nullptr;
  }

  if (legend_overlay_) {
    overlay_manager.destroy(legend_overlay_);
    legend_overlay_ = nullptr;
  }

  auto& material_manager = Ogre::MaterialManager::getSingleton();
  for (const std::string& material_name : legend_material_names_) {
    if (material_manager.resourceExists(material_name)) {
      material_manager.remove(material_name);
    }
  }
  legend_material_names_.clear();

  auto& texture_manager = Ogre::TextureManager::getSingleton();
  for (const std::string& texture_name : legend_texture_names_) {
    if (texture_manager.resourceExists(texture_name)) {
      texture_manager.remove(texture_name);
    }
  }
  legend_texture_names_.clear();
}

std::string LayeredMapDisplay::makeLegendMaterial(
    const Ogre::ColourValue& color, const std::string& name_hint) {
  std::ostringstream material_name;
  material_name << "LayeredMapLegend_" << name_hint << "_"
                << reinterpret_cast<uintptr_t>(this) << "_"
                << legend_element_counter_++;

  Ogre::TexturePtr texture;
  std::string texture_name;
  if (name_hint != "background") {
    std::ostringstream texture_name_stream;
    texture_name_stream << "LayeredMapLegendTexture_" << name_hint << "_"
                        << reinterpret_cast<uintptr_t>(this) << "_"
                        << legend_element_counter_++;
    texture_name = texture_name_stream.str();
    texture = Ogre::TextureManager::getSingleton().createManual(
        texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, 1u, 1u, 0u, Ogre::PF_A8R8G8B8,
        Ogre::TU_DYNAMIC_WRITE_ONLY);

    const auto to_byte = [](float value) {
      return static_cast<uint8_t>(
          std::clamp(static_cast<int>(std::round(value * 255.f)), 0, 255));
    };
    std::array<uint8_t, 4> pixel = {to_byte(color.b), to_byte(color.g),
                                    to_byte(color.r), to_byte(color.a)};
    Ogre::PixelBox pixel_box(1u, 1u, 1u, Ogre::PF_A8R8G8B8, pixel.data());
    texture->getBuffer()->blitFromMemory(pixel_box);
    legend_texture_names_.push_back(texture_name);
  }

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
      material_name.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setLightingEnabled(false);
  material->setDepthCheckEnabled(false);
  material->setDepthWriteEnabled(false);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
  pass->setAmbient(color);
  pass->setDiffuse(color);
  pass->setSelfIllumination(color);
  if (!texture.isNull()) {
    pass->createTextureUnitState(texture_name);
  }

  legend_material_names_.push_back(material_name.str());
  return material_name.str();
}

void LayeredMapDisplay::addLegendText(const std::string& text, float x, float y,
                                      float char_height,
                                      const Ogre::ColourValue& color) {
  if (!legend_panel_) {
    return;
  }

  auto& overlay_manager = Ogre::OverlayManager::getSingleton();
  std::ostringstream element_name;
  element_name << "LayeredMapLegendText_" << reinterpret_cast<uintptr_t>(this)
               << "_" << legend_element_counter_++;

  auto* text_element = static_cast<Ogre::TextAreaOverlayElement*>(
      overlay_manager.createOverlayElement("TextArea", element_name.str()));
  text_element->setMetricsMode(Ogre::GMM_PIXELS);
  text_element->setPosition(x, y);
  text_element->setDimensions(380.f - x, 38.f);
  text_element->setCharHeight(char_height);
  text_element->setFontName("Liberation Sans");
  text_element->setColour(color);
  text_element->setCaption(text);

  legend_panel_->addChild(text_element);
  legend_elements_.push_back(text_element);
}

void LayeredMapDisplay::addLegendRow(const Ogre::ColourValue& color,
                                     const std::string& text, float y) {
  if (!legend_panel_) {
    return;
  }

  auto& overlay_manager = Ogre::OverlayManager::getSingleton();
  std::ostringstream swatch_name;
  swatch_name << "LayeredMapLegendSwatch_" << reinterpret_cast<uintptr_t>(this)
              << "_" << legend_element_counter_++;

  auto* swatch = static_cast<Ogre::OverlayContainer*>(
      overlay_manager.createOverlayElement("Panel", swatch_name.str()));
  swatch->setMetricsMode(Ogre::GMM_PIXELS);
  constexpr float kRowHeight = 24.f;
  constexpr float kSwatchSize = 24.f;
  swatch->setPosition(24.f, y + 0.5f * (kRowHeight - kSwatchSize));
  swatch->setDimensions(kSwatchSize, kSwatchSize);
  swatch->setMaterialName(makeLegendMaterial(color, "swatch"));

  legend_panel_->addChild(swatch);
  legend_elements_.push_back(swatch);

  addLegendText(text, 62.f, y + 2.f, 24.f,
                Ogre::ColourValue(0.05f, 0.06f, 0.07f, 1.f));
}

void LayeredMapDisplay::updateLegendOverlay(const wavemap_msgs::LayeredMap* msg) {
  if (!legend_panel_ || !legend_overlay_) {
    return;
  }

  clearLegendOverlayElements();

  struct LegendRow {
    Ogre::ColourValue color;
    std::string text;
  };

  const DisplayLayer* layer = selectedLayer();
  if (!layer) {
    legend_panel_->setDimensions(410.f, 128.f);
    addLegendText("Layered map", 22.f, 18.f, 28.f,
                  Ogre::ColourValue(0.02f, 0.025f, 0.03f, 1.f));
    addLegendText("No layer selected", 22.f, 68.f, 22.f,
                  Ogre::ColourValue(0.18f, 0.20f, 0.24f, 1.f));
    legend_overlay_->show();
    return;
  }

  std::vector<LegendRow> rows;
  std::string subtitle;

  if (layer->source == DisplayLayer::Source::kContinuous) {
    if (layer->name == "occupancy") {
      subtitle = "occupancy / log-odds";
      rows.push_back({Ogre::ColourValue(0.12f, 0.32f, 0.95f, 1.f),
                      "lower probability"});
      rows.push_back({Ogre::ColourValue(0.95f, 0.18f, 0.10f, 1.f),
                      "higher probability"});
    } else if (layer->type == "float32_rgb" || layer->type == "float32_vec3") {
      subtitle = "direct RGB color";
      rows.push_back({Ogre::ColourValue(1.f, 0.f, 0.f, 1.f), "red channel"});
      rows.push_back({Ogre::ColourValue(0.f, 1.f, 0.f, 1.f), "green channel"});
      rows.push_back({Ogre::ColourValue(0.f, 0.f, 1.f, 1.f), "blue channel"});
    } else if (layer->type == "float32") {
      subtitle = "scalar value";
      rows.push_back({Ogre::ColourValue(1.f, 0.f, 0.15f, 1.f), "0.0"});
      rows.push_back({Ogre::ColourValue(0.f, 1.f, 0.15f, 1.f), "1.0"});
    } else {
      subtitle = "No legend rule for type " + layer->type;
    }
  } else if (!msg) {
    subtitle = "No discrete data loaded";
  } else {
    const auto layer_it = std::find_if(
        msg->discrete_layers.begin(), msg->discrete_layers.end(),
        [this](const wavemap_msgs::DiscreteLayer& discrete_layer) {
          return discrete_layer.name == selected_layer_name_;
        });
    if (layer_it == msg->discrete_layers.end()) {
      subtitle = "Layer missing in map";
    } else if (layer_it->value_type == "bool") {
      subtitle = "boolean values";
      const std::set<bool> values = collectBoolValues(*layer_it);
      if (values.count(true)) {
        rows.push_back({Ogre::ColourValue(1.f, 0.05f, 0.05f, 1.f), "true"});
      }
      if (values.count(false)) {
        rows.push_back({Ogre::ColourValue(0.45f, 0.45f, 0.45f, 1.f),
                        "false hidden by default"});
      }
      if (values.empty()) {
        subtitle = "No observed values";
      }
    } else if (layer_it->value_type == "int") {
      subtitle = "integer values";
      const std::set<int> values = collectIntValues(*layer_it);
      constexpr size_t kMaxLegendEntries = 10u;
      size_t entry_count = 0u;
      for (const int value : values) {
        if (kMaxLegendEntries <= entry_count) {
          rows.push_back({Ogre::ColourValue(0.70f, 0.70f, 0.70f, 1.f),
                          "+" + std::to_string(values.size() - entry_count) +
                              " more"});
          break;
        }
        rows.push_back({stableIntColor(value), std::to_string(value)});
        ++entry_count;
      }
      if (values.empty()) {
        subtitle = "No observed values";
      }
    } else {
      subtitle = "No legend rule for type " + layer_it->value_type;
    }
  }

  constexpr size_t kMaxVisibleRows = 10u;
  const size_t num_visible_rows = std::min(rows.size(), kMaxVisibleRows);
  constexpr float kLegendRowHeight = 40.f;
  const float panel_height = std::max(136.f + kLegendRowHeight *
                                                 static_cast<float>(num_visible_rows),
                                       170.f);
  legend_panel_->setDimensions(410.f, panel_height);

  addLegendText("Layer: " + layer->name, 22.f, 20.f, 28.f,
                Ogre::ColourValue(0.02f, 0.025f, 0.03f, 1.f));
  addLegendText(subtitle, 22.f, 70.f, 21.f,
                Ogre::ColourValue(0.22f, 0.24f, 0.28f, 1.f));

  float row_y = 108.f;
  size_t visible_row_count = 0u;
  for (const LegendRow& row : rows) {
    if (kMaxVisibleRows <= visible_row_count) {
      break;
    }
    addLegendRow(row.color, row.text, row_y);
    row_y += kLegendRowHeight;
    ++visible_row_count;
  }

  legend_overlay_->show();
}

void LayeredMapDisplay::updateSourceModeCallback() {
  ProfilerZoneScoped;
  const LayeredMapSourceMode old_source_mode = source_mode_;
  source_mode_ = LayeredMapSourceMode(source_mode_property_.getStdString());

  unreliable_property_->setHidden(source_mode_ != LayeredMapSourceMode::kFromTopic);
  queue_size_property_->setHidden(source_mode_ != LayeredMapSourceMode::kFromTopic);
  topic_property_->setHidden(source_mode_ != LayeredMapSourceMode::kFromTopic);

  load_map_from_disk_property_.setHidden(source_mode_ != LayeredMapSourceMode::kFromFile);
  load_map_from_disk_property_.resetAllValues();

  if (source_mode_ != old_source_mode) {
    if (source_mode_ == LayeredMapSourceMode::kFromTopic) {
      updateTopic();
    } else {
      unsubscribe();
    }
    latest_msg_.reset();
    clearStoredMap();
    if (voxel_visual_) {
      voxel_visual_->clear();
    }
  }
}

void LayeredMapDisplay::loadMapFromDiskCallback() {
  ProfilerZoneScoped;
  const auto filepath_qt = QFileDialog::getOpenFileName(
      nullptr, "Open layered wavemap", QString(),
      "Layered wavemap files (*.lwvmp *.lwmp);;All files (*)");

  if (filepath_qt.isEmpty()) {
    load_map_from_disk_property_.resetAllValues();
    return;
  }

  const std::filesystem::path filepath{filepath_qt.toStdString()};
  std::string load_error;
  if (!loadMapFromDisk(filepath, &load_error)) {
    if (load_error.empty()) {
      load_error = "This display currently supports layered map files that "
                   "match the registered ExampleLayeredMap schema.";
    }
    AlertDialog alert{
        "Could not load layered map",
        std::string("Failed to load:\n\"") + filepath.string() +
            "\"\n\n" + load_error};
    alert.exec();
    load_map_from_disk_property_.resetAllValues();
    return;
  }

  load_map_from_disk_property_.setAtRestValue(filepath.filename().string());
}

void LayeredMapDisplay::updateLayerSelectionCallback() {
  ProfilerZoneScoped;
  selected_layer_name_ = layer_property_.getStdString();
  if (latest_msg_) {
    updateLegendOverlay(&latest_msg_.value());
    updateStoredMapForSelectedLayer(latest_msg_.value());
  } else {
    updateLegendOverlay(nullptr);
  }
  if (voxel_visual_) {
    voxel_visual_->clear();
    voxel_visual_->updateMap(true);
  }
}
}  // namespace wavemap::rviz_plugin

PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::LayeredMapDisplay, rviz::Display)

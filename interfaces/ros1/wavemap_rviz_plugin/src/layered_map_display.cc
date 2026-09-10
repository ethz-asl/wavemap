#include "wavemap_rviz_plugin/layered_map_display.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <memory>
#include <map>
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
#include <QSignalBlocker>
#include <QString>
#include <ros/console.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>
#include <wavemap/core/utils/profile/profiler_interface.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "wavemap_rviz_plugin/utils/alert_dialog.h"

namespace wavemap::rviz_plugin {
namespace {
using IndexTuple = std::tuple<int, int, int>;

template <typename IndexMsgT>
IndexTuple indexTuple(const IndexMsgT& index) {
  return {index.x, index.y, index.z};
}

void mergeContinuousPatch(const wavemap_msgs::Map& patch,
                          wavemap_msgs::Map& stored) {
  if (patch.layered_hashed_wavelet_octree.size() != 1u) {
    return;
  }
  if (stored.layered_hashed_wavelet_octree.size() != 1u) {
    stored = patch;
    return;
  }
  const auto& patch_map = patch.layered_hashed_wavelet_octree.front();
  auto& stored_map = stored.layered_hashed_wavelet_octree.front();
  std::set<IndexTuple> allocated;
  for (const auto& block_index : patch_map.allocated_block_indices) {
    allocated.emplace(indexTuple(block_index));
  }
  stored_map.allocated_block_indices = patch_map.allocated_block_indices;
  stored_map.min_cell_width = patch_map.min_cell_width;
  stored_map.min_log_odds = patch_map.min_log_odds;
  stored_map.max_log_odds = patch_map.max_log_odds;
  stored_map.tree_height = patch_map.tree_height;
  stored_map.layer_names = patch_map.layer_names;
  stored_map.layer_types = patch_map.layer_types;
  stored_map.layer_min_values = patch_map.layer_min_values;
  stored_map.layer_max_values = patch_map.layer_max_values;
  stored_map.layer_visualizations = patch_map.layer_visualizations;

  stored_map.blocks.erase(
      std::remove_if(stored_map.blocks.begin(), stored_map.blocks.end(),
                     [&](const auto& block) {
                       return allocated.find(indexTuple(block.root_node_offset)) ==
                              allocated.end();
                     }),
      stored_map.blocks.end());
  for (const auto& patch_block : patch_map.blocks) {
    const auto patch_index = indexTuple(patch_block.root_node_offset);
    const auto stored_it = std::find_if(
        stored_map.blocks.begin(), stored_map.blocks.end(),
        [&](const auto& block) {
          return indexTuple(block.root_node_offset) == patch_index;
        });
    if (stored_it == stored_map.blocks.end()) {
      stored_map.blocks.emplace_back(patch_block);
    } else {
      *stored_it = patch_block;
    }
  }
  stored.header = patch.header;
}

void mergeDiscretePatch(const wavemap_msgs::DiscreteLayer& patch,
                        wavemap_msgs::DiscreteLayer& stored) {
  stored.name = patch.name;
  stored.value_type = patch.value_type;
  stored.block_height = patch.block_height;
  stored.categories = patch.categories;

  std::set<IndexTuple> deleted_keys;
  for (const auto& deleted_index : patch.deleted_parent_indices) {
    deleted_keys.insert(indexTuple(deleted_index));
  }
  if (!deleted_keys.empty()) {
    stored.cells.erase(
        std::remove_if(stored.cells.begin(), stored.cells.end(),
                       [&](const auto& cell) {
                         return deleted_keys.count(
                                    indexTuple(cell.parent_index)) != 0u;
                       }),
        stored.cells.end());
  }

  std::map<IndexTuple, size_t> stored_positions;
  for (size_t index = 0u; index < stored.cells.size(); ++index) {
    stored_positions[indexTuple(stored.cells[index].parent_index)] = index;
  }
  for (const auto& patch_cell : patch.cells) {
    const auto key = indexTuple(patch_cell.parent_index);
    const auto position_it = stored_positions.find(key);
    if (position_it == stored_positions.end()) {
      stored_positions[key] = stored.cells.size();
      stored.cells.emplace_back(patch_cell);
    } else {
      stored.cells[position_it->second] = patch_cell;
    }
  }
  stored.deleted_parent_indices.clear();
}

void mergeLayeredPatch(const wavemap_msgs::LayeredMap& patch,
                       wavemap_msgs::LayeredMap& stored) {
  stored.header = patch.header;
  mergeContinuousPatch(patch.continuous_map, stored.continuous_map);
  for (const auto& layer_patch : patch.discrete_layers) {
    const auto stored_it = std::find_if(
        stored.discrete_layers.begin(), stored.discrete_layers.end(),
        [&](const auto& layer) { return layer.name == layer_patch.name; });
    if (stored_it == stored.discrete_layers.end()) {
      stored.discrete_layers.emplace_back(layer_patch);
      stored.discrete_layers.back().deleted_parent_indices.clear();
    } else {
      mergeDiscretePatch(layer_patch, *stored_it);
    }
  }
  stored.is_full_update = true;
}

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
      &voxel_visual_properties_, map_and_mutex_, [this]() {
        updateLegendOverlay(latest_msg_ ? &latest_msg_.value() : nullptr);
      });
  loadLayeredMapFactories();
  initializeLegendOverlay();
}

void LayeredMapDisplay::loadLayeredMapFactories() {
  layered_map_factories_.clear();
  try {
    layered_map_factory_loader_ = std::make_unique<
        pluginlib::ClassLoader<LayeredMapFactory>>(
        "wavemap_rviz_plugin", "wavemap::rviz_plugin::LayeredMapFactory");
    for (const std::string& class_name :
         layered_map_factory_loader_->getDeclaredClasses()) {
      try {
        layered_map_factories_.emplace_back(
            layered_map_factory_loader_->createInstance(class_name));
      } catch (const pluginlib::PluginlibException& exception) {
        ROS_WARN_STREAM("Could not load layered map schema handler '"
                        << class_name << "': " << exception.what());
      }
    }
  } catch (const pluginlib::PluginlibException& exception) {
    ROS_ERROR_STREAM("Could not initialize layered map schema handlers: "
                     << exception.what());
  }
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

void LayeredMapDisplay::displayMessage(
    const wavemap_msgs::LayeredMap& incoming_msg) {
  const bool replace_all = incoming_msg.is_full_update || !latest_msg_;
  if (replace_all) {
    latest_msg_ = incoming_msg;
  } else {
    mergeLayeredPatch(incoming_msg, *latest_msg_);
  }
  const wavemap_msgs::LayeredMap& msg = *latest_msg_;
  updateAvailableLayers(msg);
  configureSelectedLayerAppearance(msg);
  updateLegendOverlay(&msg);
  const DisplayLayer* selected_layer = selectedLayer();
  const bool selected_is_discrete =
      selected_layer &&
      selected_layer->source == DisplayLayer::Source::kDiscrete;
  if (!selected_is_discrete || replace_all) {
    updateStoredMapForSelectedLayer(msg);
  }

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

  if (selected_is_discrete && !replace_all) {
    const auto patch_it = std::find_if(
        incoming_msg.discrete_layers.begin(),
        incoming_msg.discrete_layers.end(),
        [this](const wavemap_msgs::DiscreteLayer& layer) {
          return layer.name == selected_layer_name_;
        });
    if (patch_it != incoming_msg.discrete_layers.end()) {
      FloatingPoint min_cell_width = 0.1f;
      if (msg.continuous_map.layered_hashed_wavelet_octree.size() == 1u) {
        min_cell_width = msg.continuous_map.layered_hashed_wavelet_octree
                             .front()
                             .min_cell_width;
      }
      voxel_visual_->applyDiscreteLayerPatch(*patch_it, min_cell_width);
    }
    return;
  }
  voxel_visual_->updateMap(true);
}

void LayeredMapDisplay::configureSelectedLayerAppearance(
    const wavemap_msgs::LayeredMap& msg) {
  if (!voxel_visual_) {
    return;
  }
  const DisplayLayer* layer = selectedLayer();
  if (!layer) {
    return;
  }

  bool has_low_color = false;
  bool has_high_color = false;
  Ogre::ColourValue low_color;
  FloatingPoint scalar_min = voxel_visual_->scalarDisplayMin();
  FloatingPoint scalar_max = voxel_visual_->scalarDisplayMax();
  Ogre::ColourValue high_color;
  std::vector<wavemap_msgs::DiscreteLayerCategory> categories;

  if (layer->source == DisplayLayer::Source::kContinuous &&
      !msg.continuous_map.layered_hashed_wavelet_octree.empty()) {
    const auto& continuous =
        msg.continuous_map.layered_hashed_wavelet_octree.front();
    const auto layer_name_it =
        std::find(continuous.layer_names.begin(), continuous.layer_names.end(),
                  selected_layer_name_);
    if (layer_name_it != continuous.layer_names.end()) {
      const size_t layer_index = static_cast<size_t>(
          std::distance(continuous.layer_names.begin(), layer_name_it));
      if (layer_index < continuous.layer_min_values.size() &&
          layer_index < continuous.layer_max_values.size() &&
          !continuous.layer_min_values[layer_index].float32_values.empty() &&
          !continuous.layer_max_values[layer_index].float32_values.empty()) {
        scalar_min = continuous.layer_min_values[layer_index].float32_values[0];
        scalar_max = continuous.layer_max_values[layer_index].float32_values[0];
      }
    }
    const auto visualization_it = std::find_if(
        continuous.layer_visualizations.begin(),
        continuous.layer_visualizations.end(), [&](const auto& visualization) {
          return visualization.name == selected_layer_name_;
        });
    if (visualization_it != continuous.layer_visualizations.end()) {
      has_low_color = visualization_it->has_low_color;
      has_high_color = visualization_it->has_high_color;
      low_color = Ogre::ColourValue(visualization_it->low_r,
                                    visualization_it->low_g,
                                    visualization_it->low_b, 1.f);
      high_color = Ogre::ColourValue(visualization_it->high_r,
                                     visualization_it->high_g,
                                     visualization_it->high_b, 1.f);
    }
  } else if (layer->source == DisplayLayer::Source::kDiscrete) {
    const auto layer_it = std::find_if(
        msg.discrete_layers.begin(), msg.discrete_layers.end(),
        [this](const auto& discrete_layer) {
          return discrete_layer.name == selected_layer_name_;
        });
    if (layer_it != msg.discrete_layers.end()) {
      categories = layer_it->categories;
    }
  }

  voxel_visual_->configureLayerAppearance(
      layer->name, layer->type,
      layer->source == DisplayLayer::Source::kDiscrete,
      scalar_min, scalar_max, has_low_color, low_color,
      has_high_color, high_color, categories);
}


bool LayeredMapDisplay::loadMapFromDisk(
    const std::filesystem::path& filepath, std::string* error_message) {
  const std::string frame_id =
      fixed_frame_.isEmpty() ? "map" : fixed_frame_.toStdString();
  for (const auto& factory : layered_map_factories_) {
    wavemap_msgs::LayeredMap msg;
    std::string factory_error;
    if (factory->tryLoad(filepath, frame_id, msg, &factory_error)) {
      displayMessage(msg);
      return true;
    }
    if (error_message && !factory_error.empty()) {
      *error_message = std::move(factory_error);
    }
  }
  return false;
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
  std::vector<DisplayLayer> new_layers;
  new_layers.push_back(
      {"occupancy", "float32", DisplayLayer::Source::kContinuous, {}, {}});

  if (msg.continuous_map.layered_hashed_wavelet_octree.size() == 1u) {
    const auto& continuous_msg =
        msg.continuous_map.layered_hashed_wavelet_octree.front();
    const size_t num_layers = std::min(continuous_msg.layer_names.size(),
                                       continuous_msg.layer_types.size());
    for (size_t layer_i = 0u; layer_i < num_layers; ++layer_i) {
      DisplayLayer layer{continuous_msg.layer_names[layer_i],
                         continuous_msg.layer_types[layer_i],
                         DisplayLayer::Source::kContinuous, {}, {}};
      if (layer_i < continuous_msg.layer_min_values.size()) {
        layer.min_values =
            continuous_msg.layer_min_values[layer_i].float32_values;
      }
      if (layer_i < continuous_msg.layer_max_values.size()) {
        layer.max_values =
            continuous_msg.layer_max_values[layer_i].float32_values;
      }
      new_layers.emplace_back(std::move(layer));
    }
  }

  for (const auto& discrete_layer : msg.discrete_layers) {
    new_layers.push_back({discrete_layer.name, discrete_layer.value_type,
                          DisplayLayer::Source::kDiscrete, {}, {}});
  }

  const bool selected_still_exists =
      std::any_of(new_layers.begin(), new_layers.end(),
                  [this](const DisplayLayer& layer) {
                    return layer.name == selected_layer_name_;
                  });
  if (!selected_still_exists) {
    selected_layer_name_ =
        new_layers.empty() ? "occupancy" : new_layers.front().name;
  }

  const auto sameLayer = [](const DisplayLayer& lhs, const DisplayLayer& rhs) {
    return lhs.name == rhs.name && lhs.type == rhs.type &&
           lhs.source == rhs.source && lhs.min_values == rhs.min_values &&
           lhs.max_values == rhs.max_values;
  };
  const bool schema_unchanged =
      available_layers_.size() == new_layers.size() &&
      std::equal(available_layers_.begin(), available_layers_.end(),
                 new_layers.begin(), sameLayer);
  available_layers_ = std::move(new_layers);
  if (schema_unchanged) {
    return;
  }

  // Rebuilding an EnumProperty emits selection-change signals. Block them
  // here because displayMessage() performs exactly one map update below.
  const QSignalBlocker signal_blocker(&layer_property_);
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
    for (const auto& factory : layered_map_factories_) {
      if (auto typed_map = factory->tryCreate(continuous_msg)) {
        map_and_mutex_->layered_map = std::move(typed_map);
        return;
      }
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

  if (!show_legend_property_.getBool()) {
    legend_overlay_->hide();
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
      const VoxelColorMode color_mode =
          voxel_visual_ ? voxel_visual_->colorMode()
                        : VoxelColorMode{VoxelColorMode::kHeight};
      switch (color_mode.toTypeId()) {
        case VoxelColorMode::kProbability:
          subtitle = "occupancy / probability";
          rows.push_back({logOddsToColor(-20.f), "0% probability"});
          rows.push_back({logOddsToColor(0.f), "50% probability"});
          rows.push_back({logOddsToColor(20.f), "100% probability"});
          break;
        case VoxelColorMode::kFlat:
          subtitle = "occupancy / flat color";
          rows.push_back(
              {voxel_visual_ ? voxel_visual_->flatColor()
                             : Ogre::ColourValue::Blue,
               "all displayed voxels"});
          break;
        case VoxelColorMode::kHeight:
        default:
          subtitle = "Color based on height";
          break;
      }
    } else if (layer->type == "float32_rgb" || layer->type == "float32_vec3") {
      subtitle = "direct RGB color";
      rows.push_back({Ogre::ColourValue(1.f, 0.f, 0.f, 1.f), "red channel"});
      rows.push_back({Ogre::ColourValue(0.f, 1.f, 0.f, 1.f), "green channel"});
      rows.push_back({Ogre::ColourValue(0.f, 0.f, 1.f, 1.f), "blue channel"});
    } else if (layer->type == "float32") {
      subtitle = "scalar value / RViz display range";
      const FloatingPoint min_value =
          voxel_visual_ ? voxel_visual_->scalarDisplayMin() : 0.f;
      const FloatingPoint max_value =
          voxel_visual_ ? voxel_visual_->scalarDisplayMax() : 1.f;
      std::ostringstream min_label;
      std::ostringstream max_label;
      min_label << min_value;
      max_label << max_value;
      rows.push_back(
          {voxel_visual_ ? voxel_visual_->scalarLowColor()
                         : Ogre::ColourValue(0.f, 0.f, 1.f, 1.f),
           min_label.str()});
      rows.push_back(
          {voxel_visual_ ? voxel_visual_->scalarHighColor()
                         : Ogre::ColourValue(1.f, 1.f, 0.f, 1.f),
           max_label.str()});
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
        rows.push_back({voxel_visual_ ? voxel_visual_->boolColor(true)
                                     : Ogre::ColourValue(1.f, 0.05f, 0.05f),
                        "true"});
      }
      if (values.count(false)) {
        rows.push_back({voxel_visual_ ? voxel_visual_->boolColor(false)
                                     : Ogre::ColourValue(0.35f, 0.35f, 0.35f),
                        "false"});
      }
      if (values.empty()) {
        subtitle = "No observed values";
      }
    } else if (layer_it->value_type == "int") {
      subtitle = "integer values";
      const std::set<int> values = collectIntValues(*layer_it);
      std::map<int, std::string> labels;
      for (const auto& category : layer_it->categories) {
        labels[category.value] = category.label;
      }
      constexpr size_t kMaxLegendEntries = 10u;
      const size_t category_limit = values.size() > kMaxLegendEntries
                                        ? kMaxLegendEntries - 1u
                                        : kMaxLegendEntries;
      size_t entry_count = 0u;
      for (const int value : values) {
        if (category_limit <= entry_count) {
          rows.push_back({Ogre::ColourValue(0.70f, 0.70f, 0.70f, 1.f),
                          "+" + std::to_string(values.size() - entry_count) +
                              " more"});
          break;
        }
        const auto label_it = labels.find(value);
        const std::string label =
            label_it == labels.end() || label_it->second.empty()
                ? "Value " + std::to_string(value)
                : label_it->second + " (" + std::to_string(value) + ")";
        rows.push_back(
            {voxel_visual_ ? voxel_visual_->categoryColor(value)
                           : stableIntColor(value),
             label});
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

void LayeredMapDisplay::updateLegendVisibilityCallback() {
  updateLegendOverlay(latest_msg_ ? &latest_msg_.value() : nullptr);
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
                   "match a registered layered-map schema.";
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
  const DisplayLayer* previous_layer = selectedLayer();
  if (voxel_visual_ && previous_layer) {
    if (previous_layer->source == DisplayLayer::Source::kDiscrete) {
      discrete_termination_height_ = voxel_visual_->terminationHeight();
    } else {
      continuous_termination_height_ = voxel_visual_->terminationHeight();
    }
  }
  selected_layer_name_ = layer_property_.getStdString();
  if (latest_msg_) {
    configureSelectedLayerAppearance(latest_msg_.value());
    updateLegendOverlay(&latest_msg_.value());
    updateStoredMapForSelectedLayer(latest_msg_.value());
  } else {
    updateLegendOverlay(nullptr);
  }
  if (voxel_visual_) {
    const DisplayLayer* current_layer = selectedLayer();
    if (current_layer && latest_msg_) {
      if (current_layer->source == DisplayLayer::Source::kDiscrete) {
        const auto layer_it = std::find_if(
            latest_msg_->discrete_layers.begin(),
            latest_msg_->discrete_layers.end(),
            [this](const auto& layer) {
              return layer.name == selected_layer_name_;
            });
        const int max_height =
            layer_it == latest_msg_->discrete_layers.end()
                ? 1
                : layer_it->block_height;
        voxel_visual_->setTerminationHeight(max_height,
                                             discrete_termination_height_);
      } else if (latest_msg_->continuous_map.layered_hashed_wavelet_octree.size() ==
                 1u) {
        voxel_visual_->setTerminationHeight(
            latest_msg_->continuous_map.layered_hashed_wavelet_octree.front()
                .tree_height,
            continuous_termination_height_);
      }
    }
    voxel_visual_->clear();
    voxel_visual_->updateMap(true);
  }
}
}  // namespace wavemap::rviz_plugin

PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::LayeredMapDisplay, rviz::Display)

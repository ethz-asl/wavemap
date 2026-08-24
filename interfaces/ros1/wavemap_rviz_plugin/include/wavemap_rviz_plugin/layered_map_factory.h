#ifndef WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_FACTORY_H_
#define WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_FACTORY_H_

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <ros/console.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include <wavemap_msgs/LayeredHashedWaveletOctree.h>
#include <wavemap_msgs/LayeredMap.h>
#include <wavemap_ros_conversions/layered_map_msg_conversions.h>

#include "wavemap_rviz_plugin/layered_map_interface.h"

namespace wavemap::rviz_plugin {
inline std::vector<LayerMetadata> extractLayerMetadata(
    const wavemap_msgs::LayeredHashedWaveletOctree& msg) {
  std::vector<LayerMetadata> layers;
  const size_t num_layers =
      std::min(msg.layer_names.size(), msg.layer_types.size());
  layers.reserve(num_layers);
  for (size_t layer_idx = 0u; layer_idx < num_layers; ++layer_idx) {
    LayerMetadata metadata{msg.layer_names[layer_idx],
                           msg.layer_types[layer_idx], {}, {}};
    if (layer_idx < msg.layer_min_values.size()) {
      metadata.min_values = msg.layer_min_values[layer_idx].float32_values;
    }
    if (layer_idx < msg.layer_max_values.size()) {
      metadata.max_values = msg.layer_max_values[layer_idx].float32_values;
    }
    layers.emplace_back(std::move(metadata));
  }
  return layers;
}

// Runtime factory interface. The display owns factories through this base class,
// while each concrete factory knows how to rebuild one specific user voxel type.
class LayeredMapFactory {
 public:
  virtual ~LayeredMapFactory() = default;

  virtual std::shared_ptr<LayeredMapInterface> tryCreate(
      const wavemap_msgs::LayeredHashedWaveletOctree& msg) const = 0;

  virtual bool tryLoad(const std::filesystem::path& /*filepath*/,
                       const std::string& /*frame_id*/,
                       wavemap_msgs::LayeredMap& /*msg*/,
                       std::string* /*error_message*/) const {
    return false;
  }
};

template <typename LayeredMapT, typename CellDataRosConverterT,
          typename LayerColorProviderT = NoLayerColorProvider>
class TypedLayeredMapFactory : public LayeredMapFactory {
 public:
  std::shared_ptr<LayeredMapInterface> tryCreate(
      const wavemap_msgs::LayeredHashedWaveletOctree& msg) const override {
    // Never invoke a typed converter on a different schema. Besides producing
    // invalid values, converters commonly index their expected layer arrays
    // directly, so a mismatch can otherwise become an out-of-bounds access.
    if (msg.layer_names != CellDataRosConverterT::layerNames() ||
        msg.layer_types != CellDataRosConverterT::layerTypes()) {
      return nullptr;
    }
    typename LayeredMapT::Ptr map;
    convert::rosMsgToMap<typename LayeredMapT::CellDataType,
                         CellDataRosConverterT>(msg, map);
    if (!map) {
      ROS_WARN("Failed to create typed layered map for RViz display.");
      return nullptr;
    }

    return std::make_shared<
        HashedWaveletOctreeLayeredMapAdapter<LayeredMapT, LayerColorProviderT>>(
        map, extractLayerMetadata(msg));
  }
};

// Adds file loading to a typed topic factory. A user registering a new schema
// only supplies their map, existing IO codec, and config factory; ROS message
// construction remains shared infrastructure.
template <typename ContinuousMapT, typename CellDataRosConverterT,
          typename LayerColorProviderT, typename FullLayeredMapT,
          typename LayeredMapIoT, auto MakeConfigT>
class TypedLayeredMapFileFactory
    : public TypedLayeredMapFactory<ContinuousMapT, CellDataRosConverterT,
                                    LayerColorProviderT> {
 public:
  bool tryLoad(const std::filesystem::path& filepath,
               const std::string& frame_id, wavemap_msgs::LayeredMap& msg,
               std::string* error_message) const override {
    FullLayeredMapT map(MakeConfigT());
    if (!LayeredMapIoT::load(filepath, map, error_message)) {
      return false;
    }
    return convert::layeredMapToRosMsg<FullLayeredMapT,
                                       CellDataRosConverterT>(
        map, frame_id, ros::Time(0), msg);
  }
};

}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_FACTORY_H_

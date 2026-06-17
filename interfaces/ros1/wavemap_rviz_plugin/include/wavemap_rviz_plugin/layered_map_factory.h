#ifndef WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_FACTORY_H_
#define WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_FACTORY_H_

#include <algorithm>
#include <memory>
#include <vector>

#include <ros/console.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include <wavemap_msgs/LayeredHashedWaveletOctree.h>

#include "wavemap_rviz_plugin/layered_map_interface.h"

namespace wavemap::rviz_plugin {
inline std::vector<LayerMetadata> extractLayerMetadata(
    const wavemap_msgs::LayeredHashedWaveletOctree& msg) {
  std::vector<LayerMetadata> layers;
  const size_t num_layers =
      std::min(msg.layer_names.size(), msg.layer_types.size());
  layers.reserve(num_layers);
  for (size_t layer_idx = 0u; layer_idx < num_layers; ++layer_idx) {
    layers.push_back({msg.layer_names[layer_idx], msg.layer_types[layer_idx]});
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
};

template <typename LayeredMapT, typename CellDataRosConverterT,
          typename LayerColorProviderT = NoLayerColorProvider>
class TypedLayeredMapFactory : public LayeredMapFactory {
 public:
  std::shared_ptr<LayeredMapInterface> tryCreate(
      const wavemap_msgs::LayeredHashedWaveletOctree& msg) const override {
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

}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_FACTORY_H_

#ifndef WAVEMAP_RVIZ_PLUGIN_COMMON_H_
#define WAVEMAP_RVIZ_PLUGIN_COMMON_H_

#include <mutex>
#include <memory>
#include <optional>
#include <string>

#include <wavemap/core/map/map_base.h>
#include <wavemap_msgs/LayeredHashedWaveletOctree.h>

#include "wavemap_rviz_plugin/layered_map_interface.h"

namespace wavemap::rviz_plugin {
struct MapAndMutex {
  MapBase::Ptr map;
  std::shared_ptr<LayeredMapInterface> layered_map;
  std::optional<wavemap_msgs::LayeredHashedWaveletOctree> layered_map_msg;
  std::string selected_layer_name = "occupancy";
  std::mutex mutex;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_COMMON_H_

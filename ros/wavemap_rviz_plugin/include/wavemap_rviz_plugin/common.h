#ifndef WAVEMAP_RVIZ_PLUGIN_COMMON_H_
#define WAVEMAP_RVIZ_PLUGIN_COMMON_H_

#include <mutex>

#include <wavemap/map/volumetric_data_structure_base.h>

namespace wavemap::rviz_plugin {
struct MapAndMutex {
  VolumetricDataStructureBase::Ptr map;
  std::mutex mutex;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_COMMON_H_

#ifndef WAVEMAP_ROS_MAP_OPERATIONS_MAP_ROS_OPERATION_TYPES_H_
#define WAVEMAP_ROS_MAP_OPERATIONS_MAP_ROS_OPERATION_TYPES_H_

#include <wavemap/core/config/type_selector.h>

namespace wavemap {
struct MapRosOperationType : public TypeSelector<MapRosOperationType> {
  using TypeSelector<MapRosOperationType>::TypeSelector;

  enum Id : TypeId { kPublishMap, kPublishPointcloud, kCropMap };

  static constexpr std::array names = {"publish_map", "publish_pointcloud",
                                       "crop_map"};
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_MAP_OPERATIONS_MAP_ROS_OPERATION_TYPES_H_

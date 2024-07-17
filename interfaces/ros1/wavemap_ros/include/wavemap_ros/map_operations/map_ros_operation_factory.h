#ifndef WAVEMAP_ROS_MAP_OPERATIONS_MAP_ROS_OPERATION_FACTORY_H_
#define WAVEMAP_ROS_MAP_OPERATIONS_MAP_ROS_OPERATION_FACTORY_H_

#include <memory>
#include <string>

#include <ros/ros.h>
#include <wavemap/core/config/param.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>

#include "wavemap_ros/map_operations/map_ros_operation_types.h"
#include "wavemap_ros/utils/tf_transformer.h"

namespace wavemap {
class MapRosOperationFactory {
 public:
  static std::unique_ptr<MapOperationBase> create(
      const param::Value& params, MapBase::Ptr occupancy_map,
      std::shared_ptr<ThreadPool> thread_pool,
      std::shared_ptr<TfTransformer> transformer, std::string world_frame,
      ros::NodeHandle nh_private);

  static std::unique_ptr<MapOperationBase> create(
      MapRosOperationType ros_operation_type, const param::Value& params,
      MapBase::Ptr occupancy_map, std::shared_ptr<ThreadPool> thread_pool,
      std::shared_ptr<TfTransformer> transformer, std::string world_frame,
      ros::NodeHandle nh_private);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_MAP_OPERATIONS_MAP_ROS_OPERATION_FACTORY_H_

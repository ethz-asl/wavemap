#ifndef WAVEMAP_PIPELINE_MAP_OPERATIONS_MAP_OPERATION_FACTORY_H_
#define WAVEMAP_PIPELINE_MAP_OPERATIONS_MAP_OPERATION_FACTORY_H_

#include <memory>
#include <string>

#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/thread_pool.h"
#include "wavemap/pipeline/map_operations/map_operation_base.h"

namespace wavemap {
class MapOperationFactory {
 public:
  static std::unique_ptr<MapOperationBase> create(const param::Value& params,
                                                  MapBase::Ptr occupancy_map);

  static std::unique_ptr<MapOperationBase> create(
      MapOperationType operation_type, const param::Value& params,
      MapBase::Ptr occupancy_map);
};
}  // namespace wavemap

#endif  // WAVEMAP_PIPELINE_MAP_OPERATIONS_MAP_OPERATION_FACTORY_H_

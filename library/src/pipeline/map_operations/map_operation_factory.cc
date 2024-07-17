#include "wavemap/pipeline/map_operations/map_operation_factory.h"

#include "wavemap/pipeline/map_operations/prune_map_operation.h"
#include "wavemap/pipeline/map_operations/threshold_map_operation.h"

namespace wavemap {
std::unique_ptr<MapOperationBase> MapOperationFactory::create(
    const param::Value& params, MapBase::Ptr occupancy_map) {
  if (const auto type = MapOperationType::from(params); type) {
    return create(type.value(), params, std::move(occupancy_map));
  }

  LOG(ERROR) << "Could not create map operation. Returning nullptr.";
  return nullptr;
}

std::unique_ptr<MapOperationBase> MapOperationFactory::create(
    MapOperationType operation_type, const param::Value& params,
    MapBase::Ptr occupancy_map) {
  if (!operation_type.isValid()) {
    LOG(ERROR) << "Received request to create map operation with invalid type.";
    return nullptr;
  }

  // Create the operation handler
  switch (operation_type) {
    case MapOperationType::kThresholdMap:
      if (const auto config = ThresholdMapOperationConfig::from(params);
          config) {
        return std::make_unique<ThresholdMapOperation>(
            config.value(), std::move(occupancy_map));
      } else {
        LOG(ERROR) << "Threshold map operation config could not be loaded.";
        return nullptr;
      }
    case MapOperationType::kPruneMap:
      if (const auto config = PruneMapOperationConfig::from(params); config) {
        return std::make_unique<PruneMapOperation>(config.value(),
                                                   std::move(occupancy_map));
      } else {
        LOG(ERROR) << "Prune map operation config could not be loaded.";
        return nullptr;
      }
  }

  LOG(ERROR) << "Factory does not (yet) support creation of map operation type "
             << operation_type.toStr() << ".";
  return nullptr;
}
}  // namespace wavemap

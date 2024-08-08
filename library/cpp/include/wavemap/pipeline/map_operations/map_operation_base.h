#ifndef WAVEMAP_PIPELINE_MAP_OPERATIONS_MAP_OPERATION_BASE_H_
#define WAVEMAP_PIPELINE_MAP_OPERATIONS_MAP_OPERATION_BASE_H_

#include <utility>

#include "wavemap/core/config/type_selector.h"

namespace wavemap {
struct MapOperationType : public TypeSelector<MapOperationType> {
  using TypeSelector<MapOperationType>::TypeSelector;

  enum Id : TypeId { kThresholdMap, kPruneMap };

  static constexpr std::array names = {"threshold_map", "prune_map"};
};

class MapOperationBase {
 public:
  explicit MapOperationBase(MapBase::Ptr occupancy_map)
      : occupancy_map_(std::move(occupancy_map)) {}

  virtual ~MapOperationBase() = default;

  virtual void run(bool force_run) = 0;

 protected:
  const MapBase::Ptr occupancy_map_;
};
}  // namespace wavemap

#endif  // WAVEMAP_PIPELINE_MAP_OPERATIONS_MAP_OPERATION_BASE_H_

#ifndef WAVEMAP_PIPELINE_MAP_OPERATIONS_THRESHOLD_MAP_OPERATION_H_
#define WAVEMAP_PIPELINE_MAP_OPERATIONS_THRESHOLD_MAP_OPERATION_H_

#include <utility>

#include "wavemap/core/config/config_base.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/time/time.h"
#include "wavemap/pipeline/map_operations/map_operation_base.h"

namespace wavemap {
/**
 * Config struct for map thresholding operations.
 */
struct ThresholdMapOperationConfig
    : public ConfigBase<ThresholdMapOperationConfig, 1> {
  //! Time period controlling how often the map is thresholded.
  Seconds<FloatingPoint> once_every = 2.f;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class ThresholdMapOperation : public MapOperationBase {
 public:
  ThresholdMapOperation(const ThresholdMapOperationConfig& config,
                        MapBase::Ptr occupancy_map)
      : MapOperationBase(std::move(occupancy_map)),
        config_(config.checkValid()) {}

  bool shouldRun(const Timestamp& current_time = Time::now());

  void run(bool force_run) override;

 private:
  const ThresholdMapOperationConfig config_;
  Timestamp last_run_timestamp_;
};
}  // namespace wavemap

#endif  // WAVEMAP_PIPELINE_MAP_OPERATIONS_THRESHOLD_MAP_OPERATION_H_

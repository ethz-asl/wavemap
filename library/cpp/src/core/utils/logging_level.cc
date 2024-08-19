#include "wavemap/core/utils/logging_level.h"

#include <glog/logging.h>

namespace wavemap {
void LoggingLevel::applyToGlog() const { FLAGS_minloglevel = toTypeId(); }
}  // namespace wavemap

#include "wavemap/core/utils/logging_level.h"

#include <glog/logging.h>

namespace wavemap {
void LoggingLevel::applyToGlog() const {
  google::SetCommandLineOption("minloglevel",
                               std::to_string(toTypeId()).c_str());
}
}  // namespace wavemap

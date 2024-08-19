#include "wavemap_ros/utils/ros_logging_level.h"

namespace wavemap {
RosLoggingLevel::operator LoggingLevel() const {
  if (id_ == Id::kDebug) {
    return LoggingLevel::kInfo;
  } else if (Id::kInfo < id_ && id_ <= Id::kFatal) {
    return id_ - 1;
  } else {
    return LoggingLevel::kInvalidTypeId;
  }
}

void RosLoggingLevel::applyToGlog() const {
  operator LoggingLevel().applyToGlog();
}

bool RosLoggingLevel::applyToRosConsole(const std::string& name) const {
  if (ros::console::set_logger_level(name, ros_levels[toTypeId()])) {
    ros::console::notifyLoggerLevelsChanged();
    return true;
  }
  return false;
}
}  // namespace wavemap

#ifndef WAVEMAP_ROS_UTILS_ROS_LOGGING_LEVEL_H_
#define WAVEMAP_ROS_UTILS_ROS_LOGGING_LEVEL_H_

#include <string>

#include <ros/console.h>
#include <wavemap/core/config/type_selector.h>
#include <wavemap/core/utils/logging_level.h>

namespace wavemap {
struct RosLoggingLevel : public TypeSelector<RosLoggingLevel> {
  using TypeSelector<RosLoggingLevel>::TypeSelector;

  enum Id : TypeId { kDebug, kInfo, kWarning, kError, kFatal };

  static constexpr std::array names = {"debug", "info", "warning", "error",
                                       "fatal"};
  static constexpr std::array ros_levels = {
      ros::console::levels::Debug, ros::console::levels::Info,
      ros::console::levels::Warn, ros::console::levels::Error,
      ros::console::levels::Fatal};

  // Conversion to general LoggingLevel (from the C++ Library)
  operator LoggingLevel() const;  // NOLINT

  // Apply the logger level to a given output
  void applyToGlog() const;
  bool applyToRosConsole(
      const std::string& name = ROSCONSOLE_DEFAULT_NAME) const;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_UTILS_ROS_LOGGING_LEVEL_H_

#ifndef WAVEMAP_ROS_LOGGING_LEVEL_H_
#define WAVEMAP_ROS_LOGGING_LEVEL_H_

#include <ros/console.h>
#include <wavemap/config/type_selector.h>

namespace wavemap {
struct LoggingLevel : public TypeSelector<LoggingLevel> {
  using TypeSelector<LoggingLevel>::TypeSelector;

  enum Id : TypeId { kDebug, kInfo, kWarning, kError, kFatal };

  static constexpr std::array names = {"debug", "info", "warning", "error",
                                       "fatal"};
  static constexpr std::array ros_levels = {
      ros::console::levels::Debug, ros::console::levels::Info,
      ros::console::levels::Warn, ros::console::levels::Error,
      ros::console::levels::Fatal};
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_LOGGING_LEVEL_H_

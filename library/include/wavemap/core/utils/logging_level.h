#ifndef WAVEMAP_CORE_UTILS_LOGGING_LEVEL_H_
#define WAVEMAP_CORE_UTILS_LOGGING_LEVEL_H_

#include <wavemap/core/config/type_selector.h>

namespace wavemap {
struct LoggingLevel : public TypeSelector<LoggingLevel> {
  using TypeSelector<LoggingLevel>::TypeSelector;

  enum Id : TypeId { kInfo, kWarning, kError, kFatal };

  static constexpr std::array names = {"info", "warning", "error", "fatal"};

  // Apply the logger level to GLOG
  void applyToGlog() const;
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_LOGGING_LEVEL_H_

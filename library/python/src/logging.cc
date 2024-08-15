#include "pywavemap/logging.h"

#include <glog/logging.h>
#include <nanobind/stl/string.h>
#include <wavemap/core/utils/logging_level.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_logging_module(nb::module_& m_logging) {
  // Initialize GLOG
  google::InitGoogleLogging("pywavemap");
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_log_prefix = false;

  // Methods to configure GLOG
  m_logging.def(
      "set_level",
      [](const std::string& level) {
        if (const auto glog_level = LoggingLevel::from(level); glog_level) {
          glog_level->applyToGlog();
        }
      },
      "level"_a = "info", "Set pywavemap's logging level.");
  m_logging.def(
      "enable_prefix", [](bool enable) { FLAGS_log_prefix = enable; },
      "enable"_a = false,
      "Whether to prefix log messages with timestamps and line numbers.");
}
}  // namespace wavemap

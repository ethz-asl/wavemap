#include "wavemap/io/config/stream_conversions.h"

#ifdef YAML_CPP_AVAILABLE
#include <yaml-cpp/yaml.h>

#include "wavemap/io/config/yaml_cpp_conversions.h"
#endif

namespace wavemap::io {
std::optional<param::Value> yamlStreamToParams(
    [[maybe_unused]] std::istream& istream) {
#ifdef YAML_CPP_AVAILABLE
  try {
    YAML::Node yaml = YAML::Load(istream);
    return convert::yamlToParams(yaml);
  } catch (YAML::ParserException&) {
    LOG(WARNING) << "Failed to parse bytestream using yaml-cpp.";
    return std::nullopt;
  }
#endif
  LOG(ERROR) << "No YAML parser is available. Install yaml-cpp or add an "
                "interface to your preferred parser in wavemap/io/config.";
  return std::nullopt;
}
}  // namespace wavemap::io

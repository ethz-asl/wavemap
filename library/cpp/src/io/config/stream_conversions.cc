#include "wavemap/io/config/stream_conversions.h"

#ifdef YAML_CPP_AVAILABLE
#include <yaml-cpp/yaml.h>

#include "wavemap/io/config/yaml_cpp_conversions.h"
#endif

namespace wavemap::io {
bool yamlStreamToParams(std::istream& istream, param::Value& params) {
#ifdef YAML_CPP_AVAILABLE
  try {
    YAML::Node yaml = YAML::Load(istream);
    params = convert::yamlToParams(yaml);
    return true;
  } catch (YAML::ParserException&) {
    LOG(WARNING) << "Failed to parse bytestream using yaml-cpp.";
    return false;
  }
#endif
  LOG(ERROR) << "No YAML parser is available. Install yaml-cpp or add an "
                "interface to your preferred parser in wavemap/io/config.";
  return false;
}
}  // namespace wavemap::io

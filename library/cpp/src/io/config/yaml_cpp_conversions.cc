#include "wavemap/io/config/yaml_cpp_conversions.h"

#include <string>

namespace wavemap::convert {
param::Map yamlToParamMap(const YAML::Node& yaml_cpp_value) {  // NOLINT
  if (!yaml_cpp_value.IsMap()) {
    LOG(WARNING) << "Expected YAML param map.";
    return {};
  }

  param::Map param_map;
  for (const auto& kv : yaml_cpp_value) {
    if (std::string key; YAML::convert<std::string>::decode(kv.first, key)) {
      param_map.emplace(key, yamlToParams(kv.second));
    } else {
      LOG(WARNING) << "Ignoring YAML map entry. Key not convertible to string.";
    }
  }
  return param_map;
}

param::Array yamlToParamArray(const YAML::Node& yaml_cpp_value) {  // NOLINT
  if (!yaml_cpp_value.IsSequence()) {
    LOG(WARNING) << "Expected YAML param sequence.";
    return {};
  }

  param::Array array;
  for (const auto& kv : yaml_cpp_value) {
    array.emplace_back(yamlToParams(kv));
  }
  return array;
}

param::Value yamlToParams(const YAML::Node& yaml_cpp_value) {  // NOLINT
  if (yaml_cpp_value.IsDefined()) {
    switch (yaml_cpp_value.Type()) {
      case YAML::NodeType::Map:
        return param::Value{yamlToParamMap(yaml_cpp_value)};
      case YAML::NodeType::Sequence:
        return param::Value{yamlToParamArray(yaml_cpp_value)};
      case YAML::NodeType::Scalar:
        if (bool value; YAML::convert<bool>::decode(yaml_cpp_value, value)) {
          return param::Value{value};
        }
        if (int value; YAML::convert<int>::decode(yaml_cpp_value, value)) {
          return param::Value{value};
        }
        if (double value;
            YAML::convert<double>::decode(yaml_cpp_value, value)) {
          return param::Value{value};
        }
        if (std::string value;
            YAML::convert<std::string>::decode(yaml_cpp_value, value)) {
          return param::Value{value};
        }
        LOG(ERROR) << "Encountered unknown type while parsing YAML params.";
        break;
      case YAML::NodeType::Undefined:
        LOG(ERROR) << "Encountered undefined type while parsing YAML params.";
        break;
      case YAML::NodeType::Null:
        LOG(ERROR) << "Encountered null type while parsing YAML params.";
        break;
      default:
        break;
    }
  } else {
    LOG(ERROR) << "Encountered undefined node while parsing YAML params.";
  }

  // On error, return an empty array
  return param::Value{param::Array{}};
}
}  // namespace wavemap::convert

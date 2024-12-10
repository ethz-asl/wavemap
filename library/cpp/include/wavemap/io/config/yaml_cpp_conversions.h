#ifndef WAVEMAP_IO_CONFIG_YAML_CPP_CONVERSIONS_H_
#define WAVEMAP_IO_CONFIG_YAML_CPP_CONVERSIONS_H_

#include <wavemap/core/config/param.h>
#include <yaml-cpp/yaml.h>

namespace wavemap::convert {
param::Map yamlToParamMap(const YAML::Node& yaml_cpp_value);
param::Array yamlToParamArray(const YAML::Node& yaml_cpp_value);
param::Value yamlToParams(const YAML::Node& yaml_cpp_value);
}  // namespace wavemap::convert

#endif  // WAVEMAP_IO_CONFIG_YAML_CPP_CONVERSIONS_H_

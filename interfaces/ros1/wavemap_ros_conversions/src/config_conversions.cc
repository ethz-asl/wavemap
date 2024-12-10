#include "wavemap_ros_conversions/config_conversions.h"

#include <string>

namespace wavemap::convert {
param::Map xmlRpcToParamMap(  // NOLINT
    const XmlRpc::XmlRpcValue& xml_rpc_value) {
  if (xml_rpc_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_WARN("Expected ROS param map.");
    return {};
  }

  param::Map param_map;
  for (const auto& kv : xml_rpc_value) {
    param_map.emplace(kv.first, xmlRpcToParams(kv.second));
  }
  return param_map;
}

param::Array xmlRpcToParamArray(  // NOLINT
    const XmlRpc::XmlRpcValue& xml_rpc_value) {
  if (xml_rpc_value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("Expected ROS param array.");
    return {};
  }

  param::Array array;
  array.reserve(xml_rpc_value.size());
  for (int idx = 0; idx < xml_rpc_value.size(); ++idx) {  // NOLINT
    array.emplace_back(xmlRpcToParams(xml_rpc_value[idx]));
  }
  return array;
}

param::Value xmlRpcToParams(  // NOLINT
    const XmlRpc::XmlRpcValue& xml_rpc_value) {
  switch (xml_rpc_value.getType()) {
    case XmlRpc::XmlRpcValue::TypeStruct:
      return param::Value{xmlRpcToParamMap(xml_rpc_value)};
    case XmlRpc::XmlRpcValue::TypeArray:
      return param::Value{xmlRpcToParamArray(xml_rpc_value)};
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return param::Value{static_cast<bool>(xml_rpc_value)};
    case XmlRpc::XmlRpcValue::TypeInt:
      return param::Value{static_cast<int>(xml_rpc_value)};
    case XmlRpc::XmlRpcValue::TypeDouble:
      return param::Value{static_cast<double>(xml_rpc_value)};
    case XmlRpc::XmlRpcValue::TypeString:
      return param::Value{static_cast<std::string>(xml_rpc_value)};
    case XmlRpc::XmlRpcValue::TypeInvalid:
      ROS_ERROR("Encountered invalid type while parsing ROS params.");
      break;
    case XmlRpc::XmlRpcValue::TypeDateTime:
      ROS_ERROR(
          "Encountered a date-time field when parsing ROS params. These "
          "are not yet supported and will be ignored.");
      break;
    case XmlRpc::XmlRpcValue::TypeBase64:
      ROS_ERROR(
          "Encountered a binary data field when parsing ROS paramsThese "
          "are not yet supported and will be ignored.");
      break;
    default:
      ROS_ERROR("Encountered unknown type while parsing ROS params.");
      break;
  }

  // On error, return an empty array
  return param::Value{param::Array{}};
}

param::Map rosToParamMap(const ros::NodeHandle& nh, const std::string& ns) {
  XmlRpc::XmlRpcValue xml_rpc_value;
  if (nh.getParam(ns, xml_rpc_value)) {
    return xmlRpcToParamMap(xml_rpc_value);
  }

  ROS_WARN_STREAM("Could not load ROS params under namespace "
                  << nh.resolveName(ns));
  return {};
}

param::Array rosToParamArray(const ros::NodeHandle& nh, const std::string& ns) {
  XmlRpc::XmlRpcValue xml_rpc_value;
  if (nh.getParam(ns, xml_rpc_value)) {
    return xmlRpcToParamArray(xml_rpc_value);
  }

  ROS_WARN_STREAM("Could not load ROS params under namespace "
                  << nh.resolveName(ns));
  return {};
}

param::Value rosToParams(const ros::NodeHandle& nh, const std::string& ns) {
  XmlRpc::XmlRpcValue xml_rpc_value;
  if (nh.getParam(ns, xml_rpc_value)) {
    return xmlRpcToParams(xml_rpc_value);
  }

  ROS_WARN_STREAM("Could not load ROS params under namespace "
                  << nh.resolveName(ns));
  return param::Value{param::Map{}};  // Return an empty map
}
}  // namespace wavemap::convert

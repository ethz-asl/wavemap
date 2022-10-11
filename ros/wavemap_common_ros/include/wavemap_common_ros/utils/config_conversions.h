#ifndef WAVEMAP_COMMON_ROS_UTILS_CONFIG_CONVERSIONS_H_
#define WAVEMAP_COMMON_ROS_UTILS_CONFIG_CONVERSIONS_H_

#include <string>

#include <ros/node_handle.h>
#include <wavemap_common/utils/config_utils.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace wavemap::param {
param::Map rosParamsToParamMap(const ros::NodeHandle& nh,
                               const std::string& ns);

param::Array xmlRpcToParamArray(const XmlRpc::XmlRpcValue& xml_rpc_value);
param::Map xmlRpcToParamMap(const XmlRpc::XmlRpcValue& xml_rpc_value);

param::Value xmlRpcToParamValue(const XmlRpc::XmlRpcValue& xml_rpc_value);
}  // namespace wavemap::param

#endif  // WAVEMAP_COMMON_ROS_UTILS_CONFIG_CONVERSIONS_H_

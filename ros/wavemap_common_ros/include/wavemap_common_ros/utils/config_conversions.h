#ifndef WAVEMAP_COMMON_ROS_UTILS_CONFIG_CONVERSIONS_H_
#define WAVEMAP_COMMON_ROS_UTILS_CONFIG_CONVERSIONS_H_

#include <string>

#include <ros/node_handle.h>
#include <wavemap_common/utils/config_utils.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace wavemap::param::convert {
param::Map toParamMap(const ros::NodeHandle& nh, const std::string& ns);
param::Array toParamArray(const ros::NodeHandle& nh, const std::string& ns);

param::Map toParamMap(const XmlRpc::XmlRpcValue& xml_rpc_value);
param::Array toParamArray(const XmlRpc::XmlRpcValue& xml_rpc_value);

param::Value toParamValue(const XmlRpc::XmlRpcValue& xml_rpc_value);
}  // namespace wavemap::param::convert

#endif  // WAVEMAP_COMMON_ROS_UTILS_CONFIG_CONVERSIONS_H_

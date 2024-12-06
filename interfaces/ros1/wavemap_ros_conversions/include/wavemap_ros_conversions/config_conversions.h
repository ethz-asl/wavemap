#ifndef WAVEMAP_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_

#include <string>

#include <ros/node_handle.h>
#include <wavemap/core/config/config_base.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace wavemap::convert {
param::Map xmlRpcToParamMap(const XmlRpc::XmlRpcValue& xml_rpc_value);
param::Array xmlRpcToParamArray(const XmlRpc::XmlRpcValue& xml_rpc_value);
param::Value xmlRpcToParams(const XmlRpc::XmlRpcValue& xml_rpc_value);

param::Map rosToParamMap(const ros::NodeHandle& nh, const std::string& ns);
param::Array rosToParamArray(const ros::NodeHandle& nh, const std::string& ns);
param::Value rosToParams(const ros::NodeHandle& nh, const std::string& ns);
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_

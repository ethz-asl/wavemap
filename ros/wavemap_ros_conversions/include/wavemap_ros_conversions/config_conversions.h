#ifndef WAVEMAP_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <wavemap/config/config_base.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace wavemap::param::convert {
param::Map toParamMap(const rclcpp::Node& nh, const std::string& ns);
param::Array toParamArray(const rclcpp::Node& nh, const std::string& ns);
param::Value toParamValue(const rclcpp::Node& nh, const std::string& ns);

param::Map toParamMap(const XmlRpc::XmlRpcValue& xml_rpc_value);
param::Array toParamArray(const XmlRpc::XmlRpcValue& xml_rpc_value);
param::Value toParamValue(const XmlRpc::XmlRpcValue& xml_rpc_value);
}  // namespace wavemap::param::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_

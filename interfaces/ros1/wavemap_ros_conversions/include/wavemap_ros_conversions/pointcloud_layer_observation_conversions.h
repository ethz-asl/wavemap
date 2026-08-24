#ifndef WAVEMAP_ROS_CONVERSIONS_POINTCLOUD_LAYER_OBSERVATION_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_POINTCLOUD_LAYER_OBSERVATION_CONVERSIONS_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <wavemap/layered/endpoint_attribute_pointcloud.h>
#include <wavemap_ros_conversions/layer_observation_msg_conversions.h>

namespace wavemap::convert {

struct ReflectivityObservationConfig {
  std::string field_name = "reflectivity";
  FloatingPoint scale = 1.f / 255.f;
  FloatingPoint offset = 0.f;
  FloatingPoint min_value = 0.f;
  FloatingPoint max_value = 1.f;
  size_t point_stride = 1u;
};
namespace detail {

inline const sensor_msgs::PointField* findField(
    const sensor_msgs::PointCloud2& msg, const std::string& name) {
  const auto it = std::find_if(
      msg.fields.cbegin(), msg.fields.cend(),
      [&](const auto& field) { return field.name == name; });
  return it == msg.fields.cend() ? nullptr : &*it;
}

template <typename T>
bool readPointField(const sensor_msgs::PointCloud2& msg,
                    const sensor_msgs::PointField& field,
                    size_t point_offset, FloatingPoint& value) {
  if (point_offset + field.offset + sizeof(T) > msg.data.size()) {
    return false;
  }
  T stored{};
  std::memcpy(&stored, msg.data.data() + point_offset + field.offset,
              sizeof(T));
  value = static_cast<FloatingPoint>(stored);
  return true;
}

inline bool readNumericPointField(
    const sensor_msgs::PointCloud2& msg,
    const sensor_msgs::PointField& field, size_t point_offset,
    FloatingPoint& value) {
  if (field.count != 1u) {
    return false;
  }
  switch (field.datatype) {
    case sensor_msgs::PointField::INT8:
      return readPointField<int8_t>(msg, field, point_offset, value);
    case sensor_msgs::PointField::UINT8:
      return readPointField<uint8_t>(msg, field, point_offset, value);
    case sensor_msgs::PointField::INT16:
      return readPointField<int16_t>(msg, field, point_offset, value);
    case sensor_msgs::PointField::UINT16:
      return readPointField<uint16_t>(msg, field, point_offset, value);
    case sensor_msgs::PointField::INT32:
      return readPointField<int32_t>(msg, field, point_offset, value);
    case sensor_msgs::PointField::UINT32:
      return readPointField<uint32_t>(msg, field, point_offset, value);
    case sensor_msgs::PointField::FLOAT32:
      return readPointField<float>(msg, field, point_offset, value);
    case sensor_msgs::PointField::FLOAT64:
      return readPointField<double>(msg, field, point_offset, value);
    default:
      return false;
  }
}

}  // namespace detail

template <typename ReflectivityLayerTagT>
struct RosReflectivityEndpointPointcloud {
  std_msgs::Header header;
  layered::EndpointAttributePointcloud<ReflectivityLayerTagT> pointcloud;
  size_t rejected = 0u;
  std::string error;

  explicit operator bool() const { return error.empty(); }
};

template <typename ReflectivityLayerTagT>
RosReflectivityEndpointPointcloud<ReflectivityLayerTagT>
pointcloudToReflectivityEndpointPointcloud(
    const sensor_msgs::PointCloud2& msg,
    const ReflectivityObservationConfig& config = {}) {
  static_assert(
      std::is_same_v<layered::LayerValueT<ReflectivityLayerTagT>,
                     FloatingPoint>,
      "The reflectivity endpoint layer must store FloatingPoint values.");
  RosReflectivityEndpointPointcloud<ReflectivityLayerTagT> result;
  result.header = msg.header;
  if (msg.is_bigendian) {
    result.error = "Big-endian PointCloud2 messages are not supported.";
    return result;
  }
  if (config.point_stride == 0u || msg.point_step == 0u || msg.width == 0u) {
    result.error =
        "Point stride, PointCloud2 point_step, and width must be nonzero.";
    return result;
  }

  const auto* x = detail::findField(msg, "x");
  const auto* y = detail::findField(msg, "y");
  const auto* z = detail::findField(msg, "z");
  const auto* reflectivity = detail::findField(msg, config.field_name);
  if (!x || !y || !z || !reflectivity) {
    result.error = "PointCloud2 is missing x, y, z, or reflectivity.";
    return result;
  }

  const size_t point_count =
      static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  const size_t capacity =
      (point_count + config.point_stride - 1u) / config.point_stride;
  result.pointcloud.resize(capacity);
  size_t accepted = 0u;
  for (size_t index = 0u; index < point_count;
       index += config.point_stride) {
    const size_t row = index / msg.width;
    const size_t column = index % msg.width;
    const size_t point_offset =
        row * msg.row_step + column * msg.point_step;
    FloatingPoint px = 0.f;
    FloatingPoint py = 0.f;
    FloatingPoint pz = 0.f;
    FloatingPoint raw_reflectivity = 0.f;
    if (!detail::readNumericPointField(msg, *x, point_offset, px) ||
        !detail::readNumericPointField(msg, *y, point_offset, py) ||
        !detail::readNumericPointField(msg, *z, point_offset, pz) ||
        !detail::readNumericPointField(
            msg, *reflectivity, point_offset, raw_reflectivity) ||
        !std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) ||
        !std::isfinite(raw_reflectivity)) {
      ++result.rejected;
      continue;
    }

    result.pointcloud.point(accepted) = Point3D(px, py, pz);
    result.pointcloud.template attribute<ReflectivityLayerTagT>(accepted) =
        std::clamp(config.offset + config.scale * raw_reflectivity,
                   config.min_value, config.max_value);
    ++accepted;
  }
  result.pointcloud.truncate(accepted);
  return result;
}

inline RosLayerObservationBatch<FloatingPoint>
pointcloudToReflectivityObservations(
    const sensor_msgs::PointCloud2& msg,
    const ReflectivityObservationConfig& config = {}) {
  RosLayerObservationBatch<FloatingPoint> result;
  result.header = msg.header;
  if (msg.is_bigendian) {
    result.error = "Big-endian PointCloud2 messages are not supported.";
    return result;
  }
  if (config.point_stride == 0u || msg.point_step == 0u) {
    result.error = "Point stride and PointCloud2 point_step must be nonzero.";
    return result;
  }

  const auto* x = detail::findField(msg, "x");
  const auto* y = detail::findField(msg, "y");
  const auto* z = detail::findField(msg, "z");
  const auto* reflectivity = detail::findField(msg, config.field_name);
  if (!x || !y || !z || !reflectivity) {
    result.error = "PointCloud2 is missing x, y, z, or reflectivity.";
    return result;
  }

  const size_t point_count =
      static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  result.observations.reserve(
      (point_count + config.point_stride - 1u) / config.point_stride);
  for (size_t index = 0u; index < point_count;
       index += config.point_stride) {
    const size_t row = index / msg.width;
    const size_t column = index % msg.width;
    const size_t point_offset =
        row * msg.row_step + column * msg.point_step;
    FloatingPoint px = 0.f;
    FloatingPoint py = 0.f;
    FloatingPoint pz = 0.f;
    FloatingPoint raw_reflectivity = 0.f;
    if (!detail::readNumericPointField(msg, *x, point_offset, px) ||
        !detail::readNumericPointField(msg, *y, point_offset, py) ||
        !detail::readNumericPointField(msg, *z, point_offset, pz) ||
        !detail::readNumericPointField(
            msg, *reflectivity, point_offset, raw_reflectivity) ||
        !std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) ||
        !std::isfinite(raw_reflectivity)) {
      ++result.rejected;
      continue;
    }

    const FloatingPoint value = std::clamp(
        config.offset + config.scale * raw_reflectivity,
        config.min_value, config.max_value);
    result.observations.emplace_back(Point3D(px, py, pz), value);
  }
  return result;
}

}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_POINTCLOUD_LAYER_OBSERVATION_CONVERSIONS_H_

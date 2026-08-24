#ifndef WAVEMAP_ROS_INPUTS_POINTCLOUD_ENDPOINT_ADAPTER_H_
#define WAVEMAP_ROS_INPUTS_POINTCLOUD_ENDPOINT_ADAPTER_H_

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <wavemap/core/data_structure/pointcloud.h>
#include <wavemap/core/utils/undistortion/stamped_pointcloud.h>
#include <wavemap_ros_conversions/pointcloud_layer_observation_conversions.h>

namespace wavemap {

template <typename ValueT>
struct PointFieldBinding {
  std::string field_name;
  FloatingPoint scale = 1.f;
  FloatingPoint offset = 0.f;
  ValueT min_value = std::numeric_limits<ValueT>::lowest();
  ValueT max_value = std::numeric_limits<ValueT>::max();

  explicit PointFieldBinding(
      std::string field_name = {}, FloatingPoint scale = 1.f,
      FloatingPoint offset = 0.f,
      ValueT min_value = std::numeric_limits<ValueT>::lowest(),
      ValueT max_value = std::numeric_limits<ValueT>::max())
      : field_name(std::move(field_name)),
        scale(scale),
        offset(offset),
        min_value(min_value),
        max_value(max_value) {}

  static PointFieldBinding normalized(std::string field_name,
                                      FloatingPoint raw_min,
                                      FloatingPoint raw_max) {
    CHECK_LT(raw_min, raw_max);
    const FloatingPoint scale = 1.f / (raw_max - raw_min);
    return PointFieldBinding(std::move(field_name), scale,
                             -raw_min * scale, ValueT{0}, ValueT{1});
  }
};

class DecodedPointcloudEndpointChannel {
 public:
  virtual ~DecodedPointcloudEndpointChannel() = default;
  virtual bool integrate(
      const PosedPointcloud<>& pointcloud,
      const std::vector<undistortion::StampedPoint>& sorted_points) const = 0;
};

// A message-local decoder. PointcloudTopicInput owns the outer point loop and
// calls every prepared decoder once per point, so adding endpoint layers does
// not add another traversal of the PointCloud2 message.
class PointcloudEndpointDecoder {
 public:
  virtual ~PointcloudEndpointDecoder() = default;
  virtual bool decodePoint(const sensor_msgs::PointCloud2& msg,
                           size_t point_offset) = 0;
  virtual std::unique_ptr<DecodedPointcloudEndpointChannel> finish() = 0;
};

class PointcloudEndpointAdapter {
 public:
  virtual ~PointcloudEndpointAdapter() = default;
  virtual std::unique_ptr<PointcloudEndpointDecoder> prepare(
      const sensor_msgs::PointCloud2& msg, size_t num_points) const = 0;
};

template <typename ValueT>
class DecodedScalarEndpointChannel final
    : public DecodedPointcloudEndpointChannel {
 public:
  using Callback =
      std::function<void(const PosedPointcloud<>&, const std::vector<ValueT>&)>;

  DecodedScalarEndpointChannel(std::vector<ValueT> values, Callback callback)
      : values_(std::move(values)), callback_(std::move(callback)) {}

  bool integrate(
      const PosedPointcloud<>& pointcloud,
      const std::vector<undistortion::StampedPoint>& sorted_points)
      const override {
    if (values_.size() != pointcloud.size() ||
        sorted_points.size() != pointcloud.size()) {
      return false;
    }
    std::vector<ValueT> ordered_values(values_.size());
    for (size_t index = 0u; index < sorted_points.size(); ++index) {
      if (sorted_points[index].source_index >= values_.size()) {
        return false;
      }
      ordered_values[index] = values_[sorted_points[index].source_index];
    }
    callback_(pointcloud, ordered_values);
    return true;
  }

 private:
  std::vector<ValueT> values_;
  Callback callback_;
};

template <typename ValueT>
class NumericPointcloudEndpointDecoder final
    : public PointcloudEndpointDecoder {
 public:
  using Callback = typename DecodedScalarEndpointChannel<ValueT>::Callback;

  NumericPointcloudEndpointDecoder(const sensor_msgs::PointField& field,
                                   size_t num_points, FloatingPoint scale,
                                   FloatingPoint offset, ValueT min_value,
                                   ValueT max_value, Callback callback)
      : field_(field),
        scale_(scale),
        offset_(offset),
        min_value_(min_value),
        max_value_(max_value),
        callback_(std::move(callback)) {
    values_.reserve(num_points);
  }

  bool decodePoint(const sensor_msgs::PointCloud2& msg,
                   size_t point_offset) override {
    FloatingPoint raw = 0.f;
    if (!convert::detail::readNumericPointField(msg, field_, point_offset,
                                                raw) ||
        !std::isfinite(raw)) {
      return false;
    }
    const FloatingPoint converted = offset_ + scale_ * raw;
    if (!std::isfinite(converted)) {
      return false;
    }
    if constexpr (std::is_integral_v<ValueT>) {
      if (converted != std::trunc(converted) ||
          converted < static_cast<FloatingPoint>(
                          std::numeric_limits<ValueT>::lowest()) ||
          static_cast<FloatingPoint>(std::numeric_limits<ValueT>::max()) <
              converted) {
        return false;
      }
    }
    values_.emplace_back(
        std::clamp(static_cast<ValueT>(converted), min_value_, max_value_));
    return true;
  }

  std::unique_ptr<DecodedPointcloudEndpointChannel> finish() override {
    return std::make_unique<DecodedScalarEndpointChannel<ValueT>>(
        std::move(values_), std::move(callback_));
  }

 private:
  sensor_msgs::PointField field_;
  FloatingPoint scale_;
  FloatingPoint offset_;
  ValueT min_value_;
  ValueT max_value_;
  Callback callback_;
  std::vector<ValueT> values_;
};

template <typename ValueT>
class NumericPointcloudEndpointAdapter final
    : public PointcloudEndpointAdapter {
 public:
  using Callback = typename DecodedScalarEndpointChannel<ValueT>::Callback;

  NumericPointcloudEndpointAdapter(std::string field_name, FloatingPoint scale,
                                   FloatingPoint offset, ValueT min_value,
                                   ValueT max_value, Callback callback)
      : field_name_(std::move(field_name)),
        scale_(scale),
        offset_(offset),
        min_value_(min_value),
        max_value_(max_value),
        callback_(std::move(callback)) {}

  std::unique_ptr<PointcloudEndpointDecoder> prepare(
      const sensor_msgs::PointCloud2& msg, size_t num_points) const override {
    const auto* field = convert::detail::findField(msg, field_name_);
    if (!field || msg.point_step == 0u || msg.width == 0u ||
        msg.is_bigendian) {
      return nullptr;
    }
    return std::make_unique<NumericPointcloudEndpointDecoder<ValueT>>(
        *field, num_points, scale_, offset_, min_value_, max_value_, callback_);
  }

 private:
  std::string field_name_;
  FloatingPoint scale_;
  FloatingPoint offset_;
  ValueT min_value_;
  ValueT max_value_;
  Callback callback_;
};

// Lightweight view passed to a user conversion function. Field names and ROS
// datatypes are resolved once in prepare(); the per-point callback uses only
// numeric indices and performs no allocation or string lookup.
class NumericPointFieldValues {
 public:
  explicit NumericPointFieldValues(const std::vector<FloatingPoint>& values)
      : values_(values) {}

  size_t size() const { return values_.size(); }
  FloatingPoint operator[](size_t index) const { return values_[index]; }

 private:
  const std::vector<FloatingPoint>& values_;
};

template <typename ValueT>
class DecodedOptionalScalarEndpointChannel final
    : public DecodedPointcloudEndpointChannel {
 public:
  using OptionalValue = std::optional<ValueT>;
  using Callback = std::function<void(const PosedPointcloud<>&,
                                      const std::vector<OptionalValue>&)>;

  DecodedOptionalScalarEndpointChannel(std::vector<OptionalValue> values,
                                       Callback callback)
      : values_(std::move(values)), callback_(std::move(callback)) {}

  bool integrate(
      const PosedPointcloud<>& pointcloud,
      const std::vector<undistortion::StampedPoint>& sorted_points)
      const override {
    if (values_.size() != pointcloud.size() ||
        sorted_points.size() != pointcloud.size()) {
      return false;
    }
    std::vector<OptionalValue> ordered_values(values_.size());
    for (size_t index = 0u; index < sorted_points.size(); ++index) {
      if (sorted_points[index].source_index >= values_.size()) {
        return false;
      }
      ordered_values[index] = values_[sorted_points[index].source_index];
    }
    callback_(pointcloud, ordered_values);
    return true;
  }

 private:
  std::vector<OptionalValue> values_;
  Callback callback_;
};

template <typename ValueT>
class ComputedNumericPointcloudEndpointDecoder final
    : public PointcloudEndpointDecoder {
 public:
  using OptionalValue = std::optional<ValueT>;
  using Converter =
      std::function<OptionalValue(const NumericPointFieldValues&)>;
  using Callback =
      typename DecodedOptionalScalarEndpointChannel<ValueT>::Callback;

  ComputedNumericPointcloudEndpointDecoder(
      std::vector<sensor_msgs::PointField> fields, size_t num_points,
      Converter converter, Callback callback)
      : fields_(std::move(fields)),
        field_values_(fields_.size()),
        converter_(std::move(converter)),
        callback_(std::move(callback)) {
    values_.reserve(num_points);
  }

  bool decodePoint(const sensor_msgs::PointCloud2& msg,
                   size_t point_offset) override {
    for (size_t field_index = 0u; field_index < fields_.size(); ++field_index) {
      auto& value = field_values_[field_index];
      if (!convert::detail::readNumericPointField(
              msg, fields_[field_index], point_offset, value) ||
          !std::isfinite(value)) {
        values_.emplace_back(std::nullopt);
        return true;
      }
    }
    values_.emplace_back(converter_(NumericPointFieldValues(field_values_)));
    return true;
  }

  std::unique_ptr<DecodedPointcloudEndpointChannel> finish() override {
    return std::make_unique<DecodedOptionalScalarEndpointChannel<ValueT>>(
        std::move(values_), std::move(callback_));
  }

 private:
  std::vector<sensor_msgs::PointField> fields_;
  std::vector<FloatingPoint> field_values_;
  Converter converter_;
  Callback callback_;
  std::vector<OptionalValue> values_;
};

template <typename ValueT>
class ComputedNumericPointcloudEndpointAdapter final
    : public PointcloudEndpointAdapter {
 public:
  using Decoder = ComputedNumericPointcloudEndpointDecoder<ValueT>;
  using Converter = typename Decoder::Converter;
  using Callback = typename Decoder::Callback;

  ComputedNumericPointcloudEndpointAdapter(
      std::vector<std::string> field_names, Converter converter,
      Callback callback)
      : field_names_(std::move(field_names)),
        converter_(std::move(converter)),
        callback_(std::move(callback)) {}

  std::unique_ptr<PointcloudEndpointDecoder> prepare(
      const sensor_msgs::PointCloud2& msg, size_t num_points) const override {
    if (field_names_.empty() || msg.point_step == 0u || msg.width == 0u ||
        msg.is_bigendian) {
      return nullptr;
    }
    std::vector<sensor_msgs::PointField> fields;
    fields.reserve(field_names_.size());
    for (const auto& field_name : field_names_) {
      const auto* field = convert::detail::findField(msg, field_name);
      if (!field) {
        return nullptr;
      }
      fields.emplace_back(*field);
    }
    return std::make_unique<Decoder>(
        std::move(fields), num_points, converter_, callback_);
  }

 private:
  std::vector<std::string> field_names_;
  Converter converter_;
  Callback callback_;
};

}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUTS_POINTCLOUD_ENDPOINT_ADAPTER_H_

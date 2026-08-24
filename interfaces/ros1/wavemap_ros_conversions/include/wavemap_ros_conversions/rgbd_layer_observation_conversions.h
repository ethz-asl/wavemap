#ifndef WAVEMAP_ROS_CONVERSIONS_RGBD_LAYER_OBSERVATION_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_RGBD_LAYER_OBSERVATION_CONVERSIONS_H_

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <wavemap/layered/types/rgb.h>

#include <wavemap_ros_conversions/layer_observation_msg_conversions.h>

namespace wavemap::convert {

struct RgbdColorObservationConfig {
  FloatingPoint depth_scale_16uc1 = 1e-3f;
  FloatingPoint min_depth = 0.1f;
  FloatingPoint max_depth = 20.f;
  size_t pixel_stride = 1u;
};

inline RosLayerObservationBatch<layered::Rgb>
rgbdToColorObservations(
    const sensor_msgs::Image& color, const sensor_msgs::Image& aligned_depth,
    const sensor_msgs::CameraInfo& camera_info,
    const RgbdColorObservationConfig& config = {}) {
  RosLayerObservationBatch<layered::Rgb> result;
  result.header = color.header;

  const bool rgb = color.encoding == sensor_msgs::image_encodings::RGB8;
  const bool bgr = color.encoding == sensor_msgs::image_encodings::BGR8;
  const bool depth_u16 =
      aligned_depth.encoding == sensor_msgs::image_encodings::TYPE_16UC1;
  const bool depth_f32 =
      aligned_depth.encoding == sensor_msgs::image_encodings::TYPE_32FC1;
  if ((!rgb && !bgr) || (!depth_u16 && !depth_f32)) {
    result.error =
        "Expected rgb8/bgr8 color and 16UC1/32FC1 aligned depth.";
    return result;
  }
  if (color.is_bigendian || aligned_depth.is_bigendian) {
    result.error = "Big-endian RGB-D images are not supported.";
    return result;
  }
  if (config.pixel_stride == 0u || color.width != aligned_depth.width ||
      color.height != aligned_depth.height || color.width == 0u ||
      color.height == 0u) {
    result.error = "RGB and aligned depth dimensions must match and be nonzero.";
    return result;
  }
  if ((!aligned_depth.header.frame_id.empty() &&
       aligned_depth.header.frame_id != color.header.frame_id) ||
      (!camera_info.header.frame_id.empty() &&
       camera_info.header.frame_id != color.header.frame_id)) {
    result.error = "RGB, aligned depth, and CameraInfo frames must match.";
    return result;
  }
  if (color.step < 3u * color.width ||
      aligned_depth.step <
          (depth_u16 ? 2u : 4u) * aligned_depth.width ||
      color.data.size() <
          static_cast<size_t>(color.step) * color.height ||
      aligned_depth.data.size() <
          static_cast<size_t>(aligned_depth.step) * aligned_depth.height) {
    result.error = "RGB-D image payload is smaller than its declared layout.";
    return result;
  }

  const FloatingPoint fx = camera_info.K[0];
  const FloatingPoint fy = camera_info.K[4];
  const FloatingPoint cx = camera_info.K[2];
  const FloatingPoint cy = camera_info.K[5];
  if (!(0.f < fx) || !(0.f < fy)) {
    result.error = "CameraInfo contains invalid focal lengths.";
    return result;
  }

  const size_t sampled_width =
      (color.width + config.pixel_stride - 1u) / config.pixel_stride;
  const size_t sampled_height =
      (color.height + config.pixel_stride - 1u) / config.pixel_stride;
  result.observations.reserve(sampled_width * sampled_height);

  for (size_t v = 0u; v < color.height; v += config.pixel_stride) {
    for (size_t u = 0u; u < color.width; u += config.pixel_stride) {
      const size_t depth_offset = v * aligned_depth.step +
                                  u * (depth_u16 ? 2u : 4u);
      FloatingPoint depth = 0.f;
      if (depth_u16) {
        uint16_t raw_depth = 0u;
        std::memcpy(&raw_depth, aligned_depth.data.data() + depth_offset,
                    sizeof(raw_depth));
        depth = config.depth_scale_16uc1 * raw_depth;
      } else {
        std::memcpy(&depth, aligned_depth.data.data() + depth_offset,
                    sizeof(depth));
      }
      if (!std::isfinite(depth) || depth < config.min_depth ||
          config.max_depth < depth) {
        ++result.rejected;
        continue;
      }

      const size_t color_offset = v * color.step + 3u * u;
      const auto* channels = color.data.data() + color_offset;
      const FloatingPoint red = (rgb ? channels[0] : channels[2]) / 255.f;
      const FloatingPoint green = channels[1] / 255.f;
      const FloatingPoint blue = (rgb ? channels[2] : channels[0]) / 255.f;
      const Point3D position((static_cast<FloatingPoint>(u) - cx) * depth / fx,
                             (static_cast<FloatingPoint>(v) - cy) * depth / fy,
                             depth);
      result.observations.emplace_back(
          position, layered::Rgb{red, green, blue});
    }
  }
  return result;
}

}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_RGBD_LAYER_OBSERVATION_CONVERSIONS_H_

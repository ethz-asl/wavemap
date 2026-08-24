#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/image_encodings.h>
#include <wavemap/core/utils/undistortion/stamped_pointcloud.h>
#include <wavemap_ros_conversions/pointcloud_layer_observation_conversions.h>
#include <wavemap_ros_conversions/rgbd_layer_observation_conversions.h>

#include "../common/field_layer_policies.h"

namespace {
bool approximatelyEqual(float lhs, float rhs, float tolerance = 1e-5f) {
  return std::abs(lhs - rhs) <= tolerance;
}
bool expect(const std::string& label, bool condition) {
  std::cout << "  " << label << ": " << (condition ? "pass" : "FAIL") << "\n";
  return condition;
}
sensor_msgs::PointField field(const std::string& name, uint32_t offset,
                              uint8_t datatype) {
  sensor_msgs::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1u;
  return result;
}
template <typename T>
void write(std::vector<uint8_t>& data, size_t offset, T value) {
  std::memcpy(data.data() + offset, &value, sizeof(value));
}
}  // namespace

int main() {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "os_sensor";
  cloud.height = 1u;
  cloud.width = 3u;
  cloud.point_step = 16u;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.fields = {
      field("x", 0u, sensor_msgs::PointField::FLOAT32),
      field("y", 4u, sensor_msgs::PointField::FLOAT32),
      field("z", 8u, sensor_msgs::PointField::FLOAT32),
      field("reflectivity", 12u, sensor_msgs::PointField::UINT16)};
  cloud.data.resize(cloud.row_step);
  write(cloud.data, 0u, 1.f);
  write(cloud.data, 4u, 2.f);
  write(cloud.data, 8u, 3.f);
  write<uint16_t>(cloud.data, 12u, 128u);
  write(cloud.data, 16u, -1.f);
  write(cloud.data, 20u, -2.f);
  write(cloud.data, 24u, -3.f);
  write<uint16_t>(cloud.data, 28u, 255u);
  write(cloud.data, 32u, std::numeric_limits<float>::quiet_NaN());
  write(cloud.data, 36u, 0.f);
  write(cloud.data, 40u, 0.f);
  write<uint16_t>(cloud.data, 44u, 64u);

  const auto reflectivity =
      wavemap::convert::pointcloudToReflectivityObservations(cloud);
  const auto endpoint_cloud = wavemap::convert::
      pointcloudToReflectivityEndpointPointcloud<
          wavemap::examples::field_map::ReflectivityLayer>(cloud);

  sensor_msgs::Image color;
  color.header.frame_id = "camera_color_optical_frame";
  color.width = 2u;
  color.height = 1u;
  color.encoding = sensor_msgs::image_encodings::RGB8;
  color.step = 6u;
  color.data = {255u, 0u, 0u, 0u, 128u, 255u};

  sensor_msgs::Image depth;
  depth.header.frame_id = color.header.frame_id;
  depth.width = color.width;
  depth.height = color.height;
  depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth.step = 4u;
  depth.data.resize(4u);
  write<uint16_t>(depth.data, 0u, 1000u);
  write<uint16_t>(depth.data, 2u, 2000u);

  sensor_msgs::CameraInfo camera_info;
  camera_info.header.frame_id = color.header.frame_id;
  camera_info.width = color.width;
  camera_info.height = color.height;
  camera_info.K[0] = 1.0;
  camera_info.K[4] = 1.0;
  camera_info.K[8] = 1.0;

  const auto colors = wavemap::convert::rgbdToColorObservations(
      color, depth, camera_info);

  wavemap::undistortion::StampedPointcloud timestamped_cloud(0u, "sensor", 2u);
  timestamped_cloud.emplace(1.f, 0.f, 0.f, 20u, 0u);
  timestamped_cloud.emplace(2.f, 0.f, 0.f, 10u, 1u);
  timestamped_cloud.sort();

  bool ok = true;
  std::cout << "Native sensor layer adapters Phase 5 experiment\n";
  ok &= expect(
      "Ouster reflectivity decoded and normalized",
      reflectivity && reflectivity.header.frame_id == "os_sensor" &&
          reflectivity.observations.size() == 2u &&
          approximatelyEqual(reflectivity.observations[0].value,
                             128.f / 255.f) &&
          approximatelyEqual(reflectivity.observations[1].value, 1.f));
  ok &= expect(
      "single-pass endpoint attributes remain index-aligned",
      endpoint_cloud && endpoint_cloud.rejected == 1u &&
          endpoint_cloud.pointcloud.size() == 2u &&
          endpoint_cloud.pointcloud.hasConsistentSizes() &&
          endpoint_cloud.pointcloud.point(0u) ==
              wavemap::Point3D(1.f, 2.f, 3.f) &&
          approximatelyEqual(
              endpoint_cloud.pointcloud.attribute<
                  wavemap::examples::field_map::ReflectivityLayer>(0u),
              128.f / 255.f) &&
          endpoint_cloud.pointcloud.point(1u) ==
              wavemap::Point3D(-1.f, -2.f, -3.f) &&
          approximatelyEqual(
              endpoint_cloud.pointcloud.attribute<
                  wavemap::examples::field_map::ReflectivityLayer>(1u),
              1.f));
  ok &= expect(
      "motion-undistortion sorting preserves the attribute permutation",
      timestamped_cloud[0u].source_index == 1u &&
          timestamped_cloud[1u].source_index == 0u);
  ok &= expect(
      "LiDAR positions remain in sensor frame",
      reflectivity.observations[0].position ==
          wavemap::Point3D(1.f, 2.f, 3.f));
  ok &= expect(
      "RGB-D color decoded and normalized",
      colors && colors.observations.size() == 2u &&
          colors.observations[0].value ==
              wavemap::layered::Rgb{1.f, 0.f, 0.f} &&
          approximatelyEqual(colors.observations[1].value.g, 128.f / 255.f) &&
          approximatelyEqual(colors.observations[1].value.b, 1.f));
  ok &= expect(
      "aligned depth back-projected with CameraInfo",
      colors.observations[0].position == wavemap::Point3D(0.f, 0.f, 1.f) &&
          colors.observations[1].position == wavemap::Point3D(2.f, 0.f, 2.f));
  ok &= expect("RGB-D positions remain in camera frame",
               colors.header.frame_id == "camera_color_optical_frame");
  return ok ? 0 : 1;
}

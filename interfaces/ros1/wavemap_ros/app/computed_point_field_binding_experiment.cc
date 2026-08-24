#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <wavemap/layered/pointcloud_layer_integration_mode.h>
#include <wavemap/layered/layered_map_definition.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>

#include "wavemap_ros/inputs/pointcloud_endpoint_adapter.h"
#include "wavemap_ros/layered_ros_server_extension.h"

namespace {
struct AveragedLayer
    : wavemap::layered::schema::StatefulContinuousLayer<
          float, wavemap::layered::WeightedMeanState,
          wavemap::layered::WeightedMeanLayerUpdatePolicy> {
  static constexpr std::string_view name = "averaged";
};

struct ClassLayer
    : wavemap::layered::schema::DiscreteLayer<
          int, wavemap::layered::ReplaceLayerUpdatePolicy<int>> {
  static constexpr std::string_view name = "class";
};

using AveragedDefinition = wavemap::layered::LayeredMapDefinition<
    wavemap::layered::schema::LayerSchema<AveragedLayer, ClassLayer>>;
using AveragedRosConverter = wavemap::convert::DescriptorContinuousRosConverter<
    AveragedDefinition::Voxel>;
using TestExtension = wavemap::LayeredRosServerExtension<
    AveragedDefinition::Map, AveragedDefinition::MapIo,
    AveragedRosConverter>;

[[maybe_unused]] bool instantiateDiscreteEndpointIntegration(
    TestExtension& extension,
    const wavemap::PosedPointcloud<>& pointcloud,
    const std::vector<int>& classes) {
  return extension.integrateEndpointLayer<ClassLayer>(pointcloud, classes);
}

sensor_msgs::PointField makeField(const std::string& name, uint32_t offset,
                                  uint8_t datatype) {
  sensor_msgs::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1u;
  return field;
}

template <typename ValueT>
void write(std::vector<uint8_t>& data, size_t offset, ValueT value) {
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

bool approximatelyEqual(float lhs, float rhs, float tolerance = 1e-5f) {
  return std::abs(lhs - rhs) <= tolerance;
}
}  // namespace

int main() {
  sensor_msgs::PointCloud2 cloud;
  cloud.height = 1u;
  cloud.width = 3u;
  cloud.point_step = 12u;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.fields = {
      makeField("raw", 0u, sensor_msgs::PointField::UINT16),
      makeField("gain", 4u, sensor_msgs::PointField::FLOAT32),
      makeField("class", 8u, sensor_msgs::PointField::INT32)};
  cloud.data.resize(cloud.row_step);

  write<uint16_t>(cloud.data, 0u, 100u);
  write<float>(cloud.data, 4u, 0.5f);
  write<int32_t>(cloud.data, 8u, 4);
  write<uint16_t>(cloud.data, 12u, 200u);
  write<float>(cloud.data, 16u,
               std::numeric_limits<float>::quiet_NaN());
  write<int32_t>(cloud.data, 20u, 2);
  write<uint16_t>(cloud.data, 24u, 50u);
  write<float>(cloud.data, 28u, 2.f);
  write<int32_t>(cloud.data, 32u, 7);

  std::vector<std::optional<float>> integrated_values;
  wavemap::ComputedNumericPointcloudEndpointAdapter<float> adapter(
      {"raw", "gain"},
      [](const wavemap::NumericPointFieldValues& fields)
          -> std::optional<float> {
        const float result = fields[0u] * 0.01f * fields[1u];
        return result <= 1.f ? std::optional<float>(result) : std::nullopt;
      },
      [&integrated_values](
          const wavemap::PosedPointcloud<>&,
          const std::vector<std::optional<float>>& values) {
        integrated_values = values;
      });
  std::vector<int> integrated_classes;
  wavemap::NumericPointcloudEndpointAdapter<int> class_adapter(
      "class", 1.f, 0.f, 0, 255,
      [&integrated_classes](const wavemap::PosedPointcloud<>&,
                            const std::vector<int>& values) {
        integrated_classes = values;
      });

  auto decoder = adapter.prepare(cloud, cloud.width);
  if (!decoder) {
    std::cerr << "Could not prepare computed point-field decoder.\n";
    return 1;
  }
  for (size_t index = 0u; index < cloud.width; ++index) {
    if (!decoder->decodePoint(cloud, index * cloud.point_step)) {
      return 1;
    }
  }

  auto channel = decoder->finish();
  auto class_decoder = class_adapter.prepare(cloud, cloud.width);
  if (!class_decoder) {
    std::cerr << "Could not prepare discrete point-field decoder.\n";
    return 1;
  }
  for (size_t index = 0u; index < cloud.width; ++index) {
    if (!class_decoder->decodePoint(cloud, index * cloud.point_step)) {
      return 1;
    }
  }
  auto class_channel = class_decoder->finish();
  wavemap::PosedPointcloud<> posed_cloud;
  posed_cloud.resize(3u);
  std::vector<wavemap::undistortion::StampedPoint> sorted_points;
  sorted_points.emplace_back(0.f, 0.f, 0.f, 0u, 2u);
  sorted_points.emplace_back(0.f, 0.f, 0.f, 1u, 0u);
  sorted_points.emplace_back(0.f, 0.f, 0.f, 2u, 1u);

  const bool integrated = channel->integrate(posed_cloud, sorted_points);
  const bool class_integrated =
      class_channel->integrate(posed_cloud, sorted_points);
  AveragedDefinition::Voxel averaged_voxel;
  averaged_voxel.data.get<AveragedLayer>() = {2.6f, 4.f};
  auto averaged_layers = AveragedRosConverter::makeLayers();
  AveragedRosConverter::appendLayerValues(averaged_voxel, averaged_layers);
  const auto restored_voxel =
      AveragedRosConverter::readCellData(0.f, averaged_layers, 0u);
  const auto restored_state = restored_voxel.data.get<AveragedLayer>();
  std::vector<wavemap::Index3D> endpoint_indices;
  wavemap::layered::forEachPointcloudIntegrationIndex<
      wavemap::layered::PointcloudLayerIntegrationMode::kEndpointOnly>(
      wavemap::Point3D::Zero(), wavemap::Point3D(2.f, 0.f, 0.f), 1.f,
      [&endpoint_indices](const wavemap::Index3D& index) {
        endpoint_indices.emplace_back(index);
      });
  std::vector<wavemap::Index3D> ray_indices;
  wavemap::layered::forEachPointcloudIntegrationIndex<
      wavemap::layered::PointcloudLayerIntegrationMode::kAlongRay>(
      wavemap::Point3D::Zero(), wavemap::Point3D(2.f, 0.f, 0.f), 1.f,
      [&ray_indices](const wavemap::Index3D& index) {
        ray_indices.emplace_back(index);
      });
  const wavemap::Point3D origin = wavemap::Point3D::Zero();
  const wavemap::Index3D expected_origin =
      wavemap::convert::pointToNearestIndex(origin, 1.f);
  const wavemap::Index3D expected_endpoint = wavemap::convert::pointToNearestIndex(
      wavemap::Point3D(2.f, 0.f, 0.f), 1.f);

  const bool passed =
      integrated && class_integrated && integrated_values.size() == 3u &&
      integrated_values[0u] &&
      approximatelyEqual(*integrated_values[0u], 1.f) &&
      integrated_values[1u] &&
      approximatelyEqual(*integrated_values[1u], 0.5f) &&
      !integrated_values[2u] && integrated_classes == std::vector<int>{7, 4, 2} &&
      endpoint_indices.size() == 1u &&
      endpoint_indices.front() == expected_endpoint &&
      ray_indices.size() > endpoint_indices.size() &&
      ray_indices.front() == expected_origin &&
      ray_indices.back() == expected_endpoint && averaged_layers.size() == 1u &&
      averaged_layers[0].type == "float32" &&
      averaged_layers[0].float32_values.size() == 1u &&
      approximatelyEqual(averaged_layers[0].float32_values[0], 0.65f) &&
      approximatelyEqual(restored_state.weighted_sum, 0.65f) &&
      approximatelyEqual(restored_state.total_weight, 1.f);

  std::cout << "Computed PointCloud2 field binding experiment: "
            << (passed ? "pass" : "FAIL") << "\n";
  if (!passed) {
    std::cout << "endpoint indices: " << endpoint_indices.size();
    for (const auto& index : endpoint_indices) {
      std::cout << " [" << index.transpose() << "]";
    }
    std::cout << "\nray indices: " << ray_indices.size();
    for (const auto& index : ray_indices) {
      std::cout << " [" << index.transpose() << "]";
    }
    std::cout << "\n";
  }
  return passed ? 0 : 1;
}

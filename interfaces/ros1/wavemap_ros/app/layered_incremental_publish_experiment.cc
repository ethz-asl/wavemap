#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <wavemap_msgs/LayeredMap.h>

#include "field_layered_map_ros_config.h"
#include <wavemap/pipeline/map_operations/threshold_map_operation.h>
#include "wavemap_ros/map_operations/layered_publish_map_operation.h"

namespace field_map = wavemap::examples::field_map;

int main(int argc, char** argv) {
  ros::init(argc, argv, "layered_incremental_publish_experiment");
  ros::NodeHandle nh;

  // Exercise the same type-erased threshold operation used by the original
  // Pipeline. Virtual dispatch must retain the typed layer configuration.
  bool threshold_valid = false;
  {
    field_map::Map threshold_map(field_map::makeDefaultConfig());
    const wavemap::Index3D threshold_index(2, 3, 4);
    threshold_map.continuousMap().setVoxelValue(
        threshold_index,
        field_map::Voxel(
            8.f, field_map::makeContinuousLayers(0.65f)));
    wavemap::ThresholdMapOperationConfig threshold_config;
    wavemap::ThresholdMapOperation threshold_operation(
        threshold_config, threshold_map.continuousMapPtr());
    threshold_operation.run(true);
    const auto thresholded =
        threshold_map.continuousMap().getVoxelValue(threshold_index);
    threshold_valid =
        std::abs(thresholded.occupancy - 4.f) < 1e-5f &&
        std::abs(thresholded.data.get<field_map::ReflectivityLayer>() -
                 0.4f) < 1e-5f;
  }

  std::mutex mutex;
  std::vector<wavemap_msgs::LayeredMap> messages;
  auto subscriber = nh.subscribe<wavemap_msgs::LayeredMap>(
      "/layered_incremental_test", 10,
      [&mutex, &messages](const wavemap_msgs::LayeredMap::ConstPtr& msg) {
        std::scoped_lock lock(mutex);
        messages.emplace_back(*msg);
      });

  const auto map_config = field_map::makeDefaultConfig();
  field_map::Map map(map_config);
  wavemap::PublishMapOperationConfig config;
  config.once_every = 0.01f;
  config.max_num_blocks_per_msg = 1;
  config.topic = "/layered_incremental_test";
  using Operation = wavemap::UnifiedLayeredPublishMapOperation<
      field_map::Map, field_map::ros_config::VoxelRosConverter>;
  Operation operation(config, &map, std::make_shared<wavemap::ThreadPool>(1),
                      "map", nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(0.1).sleep();

  map.continuousMap().setVoxelValue(
      wavemap::Index3D(0, 0, 0),
      field_map::Voxel(0.5f, field_map::makeContinuousLayers(0.25f)));
  auto& class_layer =
      map.discreteLayers().get<field_map::ClassLayer>();
  class_layer.setValue(wavemap::Index3D(0, 0, 0), 1);
  class_layer.setValue(wavemap::Index3D(1, 0, 0), 1);
  class_layer.setValue(wavemap::Index3D(0, 1, 0), 2);
  // Rewriting the same compressed parent must still produce one patch cell.
  class_layer.setValue(wavemap::Index3D(0, 1, 0), 3);
  operation.run(true);
  ros::WallDuration(0.1).sleep();

  // Tree height 6 means an offset of 64 lies in a different hashed block.
  map.continuousMap().setVoxelValue(
      wavemap::Index3D(64, 0, 0),
      field_map::Voxel(0.7f, field_map::makeContinuousLayers(0.75f)));
  class_layer.eraseValue(wavemap::Index3D(0, 0, 0));
  class_layer.eraseValue(wavemap::Index3D(1, 0, 0));
  class_layer.eraseValue(wavemap::Index3D(0, 1, 0));
  operation.run(true);
  ros::WallDuration(0.1).sleep();

  // A discrete-only edit must publish without touching a continuous block.
  class_layer.setValue(wavemap::Index3D(4, 0, 0), 2);
  operation.run(true);
  ros::WallDuration(0.1).sleep();
  spinner.stop();

  std::scoped_lock lock(mutex);
  bool valid = threshold_valid && messages.size() == 3u;
  if (valid) {
    const auto& first =
        messages[0].continuous_map.layered_hashed_wavelet_octree;
    const auto& second =
        messages[1].continuous_map.layered_hashed_wavelet_octree;
    const auto& third =
        messages[2].continuous_map.layered_hashed_wavelet_octree;
    valid = first.size() == 1u && second.size() == 1u && third.size() == 1u &&
            !messages[0].is_full_update && !messages[1].is_full_update &&
            !messages[2].is_full_update &&
            first[0].blocks.size() == 1u &&
            first[0].allocated_block_indices.size() == 1u &&
            second[0].blocks.size() == 1u &&
            second[0].allocated_block_indices.size() == 2u &&
            second[0].layer_names == std::vector<std::string>{"reflectivity"} &&
            second[0].layer_types == std::vector<std::string>{"float32"} &&
            second[0].layer_min_values.size() == 1u &&
            second[0].layer_min_values[0].float32_values ==
                std::vector<float>{0.f} &&
            second[0].layer_max_values.size() == 1u &&
            second[0].layer_max_values[0].float32_values ==
                std::vector<float>{0.4f} &&
            messages[0].discrete_layers.size() == 1u &&
            messages[0].discrete_layers[0].name == "class" &&
            messages[0].discrete_layers[0].cells.size() == 1u &&
            messages[0].discrete_layers[0].cells[0].dominant_int32 == 1 &&
            messages[0].discrete_layers[0].cells[0].exception_offsets.size() ==
                1u &&
            messages[0].discrete_layers[0]
                    .cells[0]
                    .exception_int32_values == std::vector<int>{3} &&
            messages[1].discrete_layers.size() == 1u &&
            messages[1].discrete_layers[0].cells.empty() &&
            messages[1].discrete_layers[0].deleted_parent_indices.size() == 1u &&
            third[0].blocks.empty() &&
            third[0].allocated_block_indices.size() == 2u &&
            messages[2].discrete_layers.size() == 1u &&
            messages[2].discrete_layers[0].cells.size() == 1u;
  }

  std::cout << "Layered incremental publish experiment\n"
            << "  messages received: " << messages.size() << "\n"
            << "  bounded continuous blocks and discrete patches: "
            << (valid ? "pass" : "FAIL") << "\n"
            << "  typed occupancy/reflectivity thresholding: "
            << (threshold_valid ? "pass" : "FAIL") << "\n";
  return valid ? 0 : 1;
}

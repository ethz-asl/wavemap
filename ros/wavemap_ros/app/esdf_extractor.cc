#include <iostream>

#include <glog/logging.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <wavemap/common.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/utils/esdf/esdf_generator.h>
#include <wavemap/utils/random_number_generator.h>
#include <wavemap_io/file_conversions.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

using namespace wavemap;  // NOLINT
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "esdf_generator");
  ros::NodeHandle nh("~");
  ros::Publisher occupancy_pub =
      nh.advertise<wavemap_msgs::Map>("map", 10, true);
  ros::Publisher esdf_pub = nh.advertise<wavemap_msgs::Map>("esdf", 10, true);
  ros::Publisher collision_free_position_pub =
      nh.advertise<visualization_msgs::Marker>("collision_free_position", 10,
                                               true);

  VolumetricDataStructureBase::Ptr occupancy_map;
  io::fileToMap("/home/victor/data/wavemaps/j_to_balcony.wvmp", occupancy_map);
  wavemap_msgs::Map occupancy_map_msg;
  convert::mapToRosMsg(*occupancy_map, "odom", ros::Time::now(),
                       occupancy_map_msg);
  occupancy_pub.publish(occupancy_map_msg);
  ros::spinOnce();

  if (const auto hashed_map =
          std::dynamic_pointer_cast<HashedWaveletOctree>(occupancy_map);
      hashed_map) {
    const auto esdf = generateEsdf(*hashed_map, 0.f, 2.f);

    wavemap_msgs::Map msg;
    convert::mapToRosMsg(esdf, "odom", ros::Time::now(), msg);
    esdf_pub.publish(msg);

    RandomNumberGenerator rng;
    for (int sample_idx = 0; sample_idx < 1000; ++sample_idx) {
      if (!ros::ok()) {
        break;
      }

      constexpr FloatingPoint kRobotRadius = 1.f;
      Point3D collision_free_position = Point3D::Constant(kNaN);
      while (true) {
        const size_t nth_block =
            rng.getRandomInteger(0ul, esdf.getBlocks().size() - 1ul);
        auto it = esdf.getBlocks().begin();
        std::advance(it, nth_block);
        if (it == esdf.getBlocks().end()) {
          continue;
        }

        const LinearIndex linear_cell_index =
            rng.getRandomInteger(0, HashedBlocks::kCellsPerBlock - 1);

        const Index3D& block_index = it->first;
        const Index3D cell_index =
            convert::linearIndexToIndex<HashedBlocks::kCellsPerSide, 3>(
                linear_cell_index);
        const Index3D global_index =
            esdf.computeIndexFromBlockIndexAndCellIndex(block_index,
                                                        cell_index);

        const FloatingPoint occupancy_value =
            occupancy_map->getCellValue(global_index);
        const bool is_unobserved = std::abs(occupancy_value) < 1e-3f;
        if (is_unobserved) {
          continue;
        }

        const auto& block = it->second;
        const FloatingPoint esdf_value = block[linear_cell_index];
        if (esdf_value < kRobotRadius) {
          continue;
        }

        collision_free_position =
            convert::indexToCenterPoint(global_index, esdf.getMinCellWidth());
        break;
      }

      visualization_msgs::Marker marker;
      marker.pose.orientation.w = 1.0;
      marker.pose.position.x = collision_free_position.x();
      marker.pose.position.y = collision_free_position.y();
      marker.pose.position.z = collision_free_position.z();
      marker.id = 100;
      marker.ns = "collision_free_pos_" + std::to_string(sample_idx);
      marker.header.frame_id = "odom";
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = kRobotRadius;
      marker.scale.y = kRobotRadius;
      marker.scale.z = kRobotRadius;
      marker.color.r = 1.0;
      marker.color.a = 1.0;

      collision_free_position_pub.publish(marker);
    }

    ros::spin();
  }
}

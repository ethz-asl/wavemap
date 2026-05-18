#include <filesystem>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/io/file_conversions.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_publish_experiment");
  ros::NodeHandle nh;

  const std::filesystem::path input_path = "/home/ci/data/maps/wavemap1.wvmp";
  const std::string map_topic = "/wavemap/map";
  const std::string map_frame = "map";
  const std::filesystem::path output_path =
      "/home/ci/data/maps/wavemap1_updated.wvmp";

  wavemap::MapBase::Ptr map;
  if (!wavemap::io::fileToMap(input_path, map)) {
    std::cerr << "Failed to load map from: " << input_path << "\n";
    return 1;
  }

  if (!map) {
    std::cerr << "Loaded map pointer is null.\n";
    return 1;
  }

  wavemap_msgs::Map map_msg;
  if (!wavemap::convert::mapToRosMsg(*map, map_frame, ros::Time::now(),
                                     map_msg)) {
    std::cerr << "Failed to convert loaded map to wavemap_msgs::Map.\n";
    return 1;
  }

  const int queue_size = 1;
  const bool latch = true;
  ros::Publisher map_pub =
      nh.advertise<wavemap_msgs::Map>(map_topic, queue_size, latch);

  ros::Duration(0.5).sleep();
  map_pub.publish(map_msg);
  ros::spinOnce();

  std::cout << "Before changes " << input_path << "\n";

  ros::Duration(5.0).sleep();
  for (int x = 0; x <= 10; ++x) {
    for (int y = 0; y <= 10; ++y) {
      for (int z = 0; z <= 10; ++z) {
        wavemap::Index3D index(x, y, z);
        map->setCellValue(index, 5.0f);
      }
    }
  }

  wavemap_msgs::Map updated_msg;
  if (!wavemap::convert::mapToRosMsg(*map, map_frame, ros::Time::now(),
                                     updated_msg)) {
    std::cerr << "Failed to convert updated map to wavemap_msgs::Map.\n";
    return 1;
  }
  map_pub.publish(updated_msg);

  std::cout << "After changes " << input_path << "\n";

  std::cout << "Loaded map from: " << input_path << "\n";
  std::cout << "Published map on: " << map_topic << "\n";
  std::cout << "RViz fixed frame should be: " << map_frame << "\n";

  if (!wavemap::io::mapToFile(*map, output_path)) {
    std::cerr << "Failed to save map to: " << output_path << "\n";
    return 1;
  }

  std::cout << "Saved modified map to: " << output_path << "\n";

  ros::spin();

  return 0;
}

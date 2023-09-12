#include <iostream>

#include <glog/logging.h>
#include <ros/ros.h>
#include <wavemap/common.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/utils/esdf/esdf_generator.h>
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

  VolumetricDataStructureBase::Ptr occupancy_map;
  io::fileToMap("/home/victor/data/wavemaps/leo.wvmp", occupancy_map);
  wavemap_msgs::Map occupancy_map_msg;
  convert::mapToRosMsg(*occupancy_map, "odom", ros::Time::now(),
                       occupancy_map_msg);
  occupancy_pub.publish(occupancy_map_msg);
  ros::spinOnce();

  if (const auto hashed_map =
          std::dynamic_pointer_cast<HashedWaveletOctree>(occupancy_map);
      hashed_map) {
    const auto esdf = generateEsdf(*hashed_map);

    wavemap_msgs::Map msg;
    convert::mapToRosMsg(esdf, "odom", ros::Time::now(), msg);
    esdf_pub.publish(msg);

    ros::spin();
  }
}

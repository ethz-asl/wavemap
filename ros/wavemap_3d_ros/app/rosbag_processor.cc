#include <rosgraph_msgs/Clock.h>
#include <tf/tfMessage.h>
#include <wavemap_common_ros/rosbag_processor.h>

#include "wavemap_3d_ros/input_handler/depth_image_input_handler.h"
#include "wavemap_3d_ros/input_handler/pointcloud_input_handler.h"
#include "wavemap_3d_ros/wavemap_3d_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_3d_rosbag_processor");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  // Setup the wavemap server node
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  wavemap::Wavemap3DServer wavemap_server(nh, nh_private);

  // Read the required ROS params
  std::string rosbag_paths_str;
  nh_private.param("rosbag_path", rosbag_paths_str, rosbag_paths_str);
  std::string input_pointcloud_republishing_topic;

  // Create the rosbag processor and load the rosbags
  wavemap::RosbagProcessor rosbag_processor;
  std::istringstream rosbag_paths_ss(rosbag_paths_str);
  if (!rosbag_processor.addRosbags(rosbag_paths_ss)) {
    return -1;
  }

  // Setup input handlers
  const wavemap::param::Array integrator_params_array =
      wavemap::param::convert::toParamArray(nh_private, "integrators");
  for (const auto& integrator_params : integrator_params_array) {
    if (integrator_params.holds<wavemap::param::Map>()) {
      const auto param_map = integrator_params.get<wavemap::param::Map>();
      wavemap::InputHandler* input_handler =
          wavemap_server.addInput(param_map, nh, nh_private);
      if (input_handler) {
        switch (input_handler->getType().toTypeId()) {
          case wavemap::InputHandlerType::kPointcloud:
            rosbag_processor.addCallback(
                input_handler->getConfig().topic_name,
                &wavemap::PointcloudInputHandler::pointcloudCallback,
                dynamic_cast<wavemap::PointcloudInputHandler*>(input_handler));
            continue;
          case wavemap::InputHandlerType::kDepthImage:
            rosbag_processor.addCallback<const sensor_msgs::Image&>(
                input_handler->getConfig().topic_name,
                &wavemap::DepthImageInputHandler::depthImageCallback,
                dynamic_cast<wavemap::DepthImageInputHandler*>(input_handler));
            continue;
        }
      }
    }
  }

  // Republish TFs
  rosbag_processor.addRepublisher<tf::tfMessage>("/tf", "/tf", nh, 10);
  rosbag_processor.addRepublisher<tf::tfMessage>("/tf_static", "/tf_static", nh,
                                                 10);
  if (rosbag_processor.bagsContainTopic("/clock")) {
    rosbag_processor.addRepublisher<rosgraph_msgs::Clock>("/clock", "/clock",
                                                          nh, 1);
  } else {
    rosbag_processor.enableSimulatedClock(nh);
  }

  // Process the rosbag
  if (!rosbag_processor.processAll()) {
    return -1;
  }

  wavemap_server.getMap()->prune();
  wavemap_server.visualizeMap();

  if (nh_private.param("keep_alive", false)) {
    ros::spin();
  }

  return 0;
}

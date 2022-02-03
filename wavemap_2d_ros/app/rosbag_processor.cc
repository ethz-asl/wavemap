#include <gflags/gflags.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/tfMessage.h>

#include "wavemap_2d_ros/wavemap_2d_server.h"

template <typename M>
std::function<void(const rosbag::MessageInstance&)> callbackAdapter(
    std::function<void(M)> function_ptr) {
  return [function_ptr](const rosbag::MessageInstance& msg) {
    typedef typename ros::ParameterAdapter<M>::Message MessageType;
    auto msg_instance = msg.instantiate<MessageType>();
    function_ptr(*msg_instance);
  };
}

template <typename M, typename T>
std::function<void(const rosbag::MessageInstance&)> callbackAdapter(
    void (T::*function_ptr)(M), T* object_ptr) {
  return [function_ptr, object_ptr](const rosbag::MessageInstance& msg) {
    typedef typename ros::ParameterAdapter<M>::Message MessageType;
    auto msg_instance = msg.instantiate<MessageType>();
    ((*object_ptr).*function_ptr)(*msg_instance);
  };
}

bool bagHasTopic(const std::string& topic, rosbag::View* bag_view) {
  CHECK_NOTNULL(bag_view);
  std::vector<const rosbag::ConnectionInfo*> connections =
      bag_view->getConnections();
  return std::any_of(connections.begin(), connections.end(),
                     [&](const rosbag::ConnectionInfo* info) {
                       return info && info->topic == topic;
                     });
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_2d");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  // Setup the wavemap server node
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  wavemap_2d::Wavemap2DServer wavemap_server(nh, nh_private);

  // Read the required ROS params
  std::string pointcloud_topic;
  nh_private.param<std::string>("pointcloud_topic", pointcloud_topic,
                                "/pointcloud");
  std::string rosbag_paths_str;
  nh_private.param<std::string>("rosbag_path", rosbag_paths_str, "");

  // Load the rosbags
  std::list<rosbag::Bag> opened_rosbags;
  rosbag::View bag_view;
  {
    std::istringstream rosbag_paths_ss(rosbag_paths_str);
    std::string rosbag_path;
    while (rosbag_paths_ss >> rosbag_path) {
      opened_rosbags.emplace_back(rosbag_path);
      bag_view.addQuery(opened_rosbags.back());
      LOG(INFO) << "Loaded rosbag " << rosbag_path;
    }
  }

  // Set the callbacks for the rosbag's topics of interest
  std::map<std::string, std::function<void(const rosbag::MessageInstance&)>>
      callbacks;
  // Pointclouds
  callbacks.emplace(
      pointcloud_topic,
      callbackAdapter(&wavemap_2d::Wavemap2DServer::pointcloudCallback,
                      &wavemap_server));
  // TFs
  ros::Publisher tf_pub = nh.advertise<tf::tfMessage>("/tf", 10);
  callbacks.emplace("/tf", [tf_pub](const rosbag::MessageInstance& msg) {
    tf_pub.publish(msg);
  });
  ros::Publisher tf_static_pub = nh.advertise<tf::tfMessage>("/tf_static", 10);
  callbacks.emplace("/tf_static",
                    [tf_static_pub](const rosbag::MessageInstance& msg) {
                      tf_static_pub.publish(msg);
                    });
  // Clock
  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
  const bool bag_has_clock = bagHasTopic("/clock", &bag_view);
  if (bag_has_clock) {
    callbacks.emplace("/clock",
                      [clock_pub](const rosbag::MessageInstance& msg) {
                        clock_pub.publish(msg);
                      });
  }

  // Playback the bag
  ros::Time side_tasks_last_timestamp(0);
  const ros::Duration kSideTasksDt(0.005);
  for (const rosbag::MessageInstance& msg : bag_view) {
    // Exit if CTRL+C was pressed
    if (!ros::ok()) {
      ROS_INFO("Shutting down...");
      for (rosbag::Bag& opened_rosbag : opened_rosbags) {
        opened_rosbag.close();
      }
      return -1;
    }

    // Handle callbacks
    auto it = callbacks.find(msg.getTopic());
    if (it != callbacks.end()) {
      it->second(msg);
    }

    // Catch up on some periodic tasks
    if (msg.getTime() < side_tasks_last_timestamp ||
        side_tasks_last_timestamp + kSideTasksDt < msg.getTime()) {
      side_tasks_last_timestamp = msg.getTime();

      // Publish clock substitute if needed
      if (!bag_has_clock) {
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = msg.getTime();
        clock_pub.publish(clock_msg);
      }

      // Process node callbacks (publishers, timers,...)
      ros::spinOnce();
    }
  }

  for (rosbag::Bag& opened_rosbag : opened_rosbags) {
    opened_rosbag.close();
  }

  wavemap_server.visualizeMap();

  ros::spin();
  return 0;
}

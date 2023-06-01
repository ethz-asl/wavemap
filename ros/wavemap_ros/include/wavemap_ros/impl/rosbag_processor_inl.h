#ifndef WAVEMAP_ROS_IMPL_ROSBAG_PROCESSOR_INL_H_
#define WAVEMAP_ROS_IMPL_ROSBAG_PROCESSOR_INL_H_

#include <string>

namespace wavemap {
template <typename MessageT>
void RosbagProcessor::addCallback(const std::string& ros_topic_name,
                                  std::function<void(MessageT)> function_ptr) {
  using MessageAdaptor = typename ros::ParameterAdapter<MessageT>::Message;
  auto rosbag_callback = [function_ptr](const rosbag::MessageInstance& msg) {
    auto msg_instance = msg.instantiate<MessageAdaptor>();
    function_ptr(*msg_instance);
  };
  callbacks_.try_emplace(ros_topic_name, rosbag_callback);
}

template <typename MessageT, typename CallbackObjectT>
void RosbagProcessor::addCallback(
    const std::string& ros_topic_name,
    void (CallbackObjectT::*function_ptr)(MessageT),
    CallbackObjectT* object_ptr) {
  using MessageAdaptor = typename ros::ParameterAdapter<MessageT>::Message;
  auto rosbag_callback = [function_ptr,
                          object_ptr](const rosbag::MessageInstance& msg) {
    auto msg_instance = msg.instantiate<MessageAdaptor>();
    ((*object_ptr).*function_ptr)(*msg_instance);
  };
  callbacks_.try_emplace(ros_topic_name, rosbag_callback);
}

template <typename MessageT>
void RosbagProcessor::addRepublisher(const std::string& rosbag_topic_name,
                                     const std::string& republished_topic_name,
                                     ros::NodeHandle& nh,
                                     unsigned int queue_size) {
  republishers_.try_emplace(
      rosbag_topic_name,
      nh.advertise<MessageT>(republished_topic_name, queue_size));
}
}  // namespace wavemap

#endif  // WAVEMAP_ROS_IMPL_ROSBAG_PROCESSOR_INL_H_

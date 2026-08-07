#ifndef WAVEMAP_ROS_LAYERED_ROS_SERVER_EXTENSION_H_
#define WAVEMAP_ROS_LAYERED_ROS_SERVER_EXTENSION_H_

#include <filesystem>
#include <mutex>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <wavemap_msgs/FilePath.h>
#include <wavemap_msgs/LayeredMap.h>
#include <wavemap_ros_conversions/layered_map_msg_conversions.h>

namespace wavemap {
class LayeredRosServerExtensionBase {
 public:
  virtual ~LayeredRosServerExtensionBase() = default;

  virtual bool saveLayeredMap(const std::filesystem::path& file_path) const = 0;
  virtual bool loadLayeredMap(const std::filesystem::path& file_path,
                              std::string* error_message = nullptr) = 0;
  virtual bool publishLayeredMap(const ros::Time& stamp = ros::Time::now()) = 0;
};

template <typename LayeredMapT, typename LayeredMapIoT, typename RosConverterT>
class LayeredRosServerExtension : public LayeredRosServerExtensionBase {
 public:
  LayeredRosServerExtension(ros::NodeHandle nh_private, std::string world_frame,
                            LayeredMapT layered_map,
                            std::string layered_map_topic = "layered_map")
      : nh_private_(std::move(nh_private)),
        world_frame_(std::move(world_frame)),
        layered_map_(std::move(layered_map)) {
    layered_map_pub_ = nh_private_.advertise<wavemap_msgs::LayeredMap>(
        layered_map_topic, kQueueSize, kLatchPublisher);
    advertiseServices();
  }

  LayeredMapT& getLayeredMap() { return layered_map_; }
  const LayeredMapT& getLayeredMap() const { return layered_map_; }

  bool saveLayeredMap(const std::filesystem::path& file_path) const override {
    std::scoped_lock lock(mutex_);
    return LayeredMapIoT::save(file_path, layered_map_);
  }

  bool loadLayeredMap(const std::filesystem::path& file_path,
                      std::string* error_message = nullptr) override {
    LayeredMapT loaded_map = layered_map_;
    if (!LayeredMapIoT::load(file_path, loaded_map, error_message)) {
      return false;
    }

    {
      std::scoped_lock lock(mutex_);
      layered_map_ = std::move(loaded_map);
    }
    return publishLayeredMap();
  }

  bool publishLayeredMap(const ros::Time& stamp = ros::Time::now()) override {
    wavemap_msgs::LayeredMap msg;
    {
      std::scoped_lock lock(mutex_);
      if (!convert::layeredMapToRosMsg<LayeredMapT, RosConverterT>(
              layered_map_, world_frame_, stamp, msg)) {
        return false;
      }
    }

    layered_map_pub_.publish(msg);
    return true;
  }

 private:
  void advertiseServices() {
    save_layered_map_srv_ =
        nh_private_.advertiseService<wavemap_msgs::FilePath::Request,
                                     wavemap_msgs::FilePath::Response>(
            "save_layered_map", [this](auto& request, auto& response) {
              response.success = saveLayeredMap(request.file_path);
              return true;
            });

    load_layered_map_srv_ =
        nh_private_.advertiseService<wavemap_msgs::FilePath::Request,
                                     wavemap_msgs::FilePath::Response>(
            "load_layered_map", [this](auto& request, auto& response) {
              std::string error_message;
              response.success = loadLayeredMap(request.file_path, &error_message);
              if (!response.success && !error_message.empty()) {
                ROS_ERROR_STREAM(error_message);
              }
              return true;
            });

    request_full_layered_map_srv_ =
        nh_private_.advertiseService<std_srvs::Empty::Request,
                                     std_srvs::Empty::Response>(
            "layered_map_request_full", [this](auto&, auto&) {
              return publishLayeredMap();
            });
  }

  static constexpr int kQueueSize = 1;
  static constexpr bool kLatchPublisher = true;

  ros::NodeHandle nh_private_;
  std::string world_frame_;

  mutable std::mutex mutex_;
  LayeredMapT layered_map_;

  ros::Publisher layered_map_pub_;
  ros::ServiceServer load_layered_map_srv_;
  ros::ServiceServer save_layered_map_srv_;
  ros::ServiceServer request_full_layered_map_srv_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_LAYERED_ROS_SERVER_EXTENSION_H_

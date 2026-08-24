#include <ros/ros.h>
#include <wavemap_msgs/LayeredMapUpdate.h>

namespace {
geometry_msgs::Point32 point(float x, float y, float z) {
  geometry_msgs::Point32 result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_field_layer_updates_experiment");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string world_frame = "map";
  nh_private.param("world_frame", world_frame, world_frame);

  ros::Publisher publisher =
      nh.advertise<wavemap_msgs::LayeredMapUpdate>(
          "/wavemap/layered_map_updates", 1, true);

  wavemap_msgs::LayeredMapUpdate update;
  update.header.frame_id = world_frame;
  update.header.stamp = ros::Time::now();
  update.positions = {
      point(0.05f, 0.05f, 0.05f),
      point(0.30f, 0.05f, 0.05f),
      point(0.05f, 0.30f, 0.05f),
      point(1.05f, 0.05f, 0.05f)};

  wavemap_msgs::Layer class_layer;
  class_layer.name = "class";
  class_layer.type = "int32";
  class_layer.int32_values = {1, 1, 2, 3};
  update.layers.emplace_back(class_layer);

  publisher.publish(update);
  ROS_INFO_STREAM("Published " << update.positions.size()
                               << " class-only observations on "
                               << publisher.getTopic() << ".");
  ros::spin();
  return 0;
}

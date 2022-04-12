#ifndef WAVEMAP_2D_ROS_IMPL_WAVEMAP_2D_SERVER_INL_H_
#define WAVEMAP_2D_ROS_IMPL_WAVEMAP_2D_SERVER_INL_H_

#include <string>

#include "wavemap_2d/iterator/grid_iterator.h"

namespace wavemap_2d {
template <typename Map>
visualization_msgs::Marker Wavemap2DServer::gridToMarker(
    const Map& grid, const std::string& world_frame,
    const std::string& marker_namespace,
    const std::function<RGBAColor(FloatingPoint)>& color_map) {
  const FloatingPoint resolution = grid.getResolution();

  visualization_msgs::Marker grid_marker;
  grid_marker.header.frame_id = world_frame;
  grid_marker.header.stamp = ros::Time();
  grid_marker.ns = marker_namespace;
  grid_marker.id = 0;
  grid_marker.type = visualization_msgs::Marker::POINTS;
  grid_marker.action = visualization_msgs::Marker::MODIFY;
  grid_marker.pose.orientation.w = 1.0;
  grid_marker.scale.x = resolution;
  grid_marker.scale.y = resolution;
  grid_marker.scale.z = resolution;
  grid_marker.color.a = 1.0;

  for (const Index& index : Grid(grid.getMinIndex(), grid.getMaxIndex())) {
    const FloatingPoint cell_value = grid.getCellValue(index);
    if (kEpsilon < std::abs(cell_value)) {
      const Point cell_position = index.cast<FloatingPoint>() * resolution;
      geometry_msgs::Point position_msg;
      position_msg.x = cell_position.x();
      position_msg.y = cell_position.y();
      position_msg.z = 0.0;
      grid_marker.points.emplace_back(position_msg);

      const RGBAColor color = color_map(cell_value);
      std_msgs::ColorRGBA color_msg;
      color_msg.a = color.a;
      color_msg.r = color.r;
      color_msg.g = color.g;
      color_msg.b = color.b;
      grid_marker.colors.emplace_back(color_msg);
    }
  }

  return grid_marker;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_ROS_IMPL_WAVEMAP_2D_SERVER_INL_H_

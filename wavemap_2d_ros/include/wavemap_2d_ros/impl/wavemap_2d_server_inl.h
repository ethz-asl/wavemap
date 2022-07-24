#ifndef WAVEMAP_2D_ROS_IMPL_WAVEMAP_2D_SERVER_INL_H_
#define WAVEMAP_2D_ROS_IMPL_WAVEMAP_2D_SERVER_INL_H_

#include <string>

#include <wavemap_2d/data_structure/volumetric/cell_types/occupancy_state.h>

namespace wavemap_2d {
template <typename Map, typename ScalarToRGBAFunction>
visualization_msgs::MarkerArray Wavemap2DServer::gridToMarkerArray(
    const Map& grid, const std::string& world_frame,
    const std::string& marker_namespace, ScalarToRGBAFunction color_map) {
  const FloatingPoint min_cell_width = grid.getMinCellWidth();
  constexpr QuadtreeIndex::Element kMaxHeight = 14;

  // Default marker
  visualization_msgs::Marker default_marker;
  default_marker.header.frame_id = world_frame;
  default_marker.header.stamp = ros::Time();
  default_marker.ns = marker_namespace;
  default_marker.id = 0;
  default_marker.type = visualization_msgs::Marker::CUBE_LIST;
  default_marker.pose.orientation.w = 1.0;
  default_marker.pose.position.x = 0.0;
  default_marker.pose.position.y = 0.0;
  default_marker.pose.position.z = 0.0;
  default_marker.scale.z = 0.1;

  // Delete the previous visuals
  visualization_msgs::MarkerArray marker_array;
  default_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.emplace_back(default_marker);

  // Add a marker for each scale
  default_marker.action = visualization_msgs::Marker::MODIFY;
  for (QuadtreeIndex::Element height = 0; height <= kMaxHeight; ++height) {
    ++default_marker.id;
    const FloatingPoint cell_width =
        convert::heightToCellWidth(min_cell_width, height);
    default_marker.scale.x = cell_width;
    default_marker.scale.y = cell_width;
    marker_array.markers.emplace_back(default_marker);
  }

  // Add a colored square for each leaf
  grid.forEachLeaf([&marker_array, &color_map, min_cell_width](
                       const QuadtreeIndex& cell_index,
                       FloatingPoint cell_value) {
    // Skip fully transparent cells
    // NOTE: This provides a flexible way to prescribe which cells are shown
    //       and hidden through the color_map. For example, unobserved
    //       occupancy cells can be skipped by mapping them to color.a = 0.
    const RGBAColor color = color_map(cell_value);
    if (std::abs(color.a) < kEpsilon) {
      return;
    }

    // Determine the cell's position
    const Point cell_center =
        convert::nodeIndexToCenterPoint(cell_index, min_cell_width);

    // Create the colored square
    geometry_msgs::Point position_msg;
    std_msgs::ColorRGBA color_msg;
    position_msg.x = cell_center.x();
    position_msg.y = cell_center.y();
    color_msg.a = color.a;
    color_msg.r = color.r;
    color_msg.g = color.g;
    color_msg.b = color.b;

    // Add the colored square at the right scale
    // NOTE: We add 1 to the height since the cube list markers for each scale
    //       start at 1 (marker 0 only clears the previous visuals).
    const size_t scale_marker_idx = cell_index.height + 1;
    marker_array.markers[scale_marker_idx].points.emplace_back(position_msg);
    marker_array.markers[scale_marker_idx].colors.emplace_back(color_msg);
  });

  // Prune away empty scale markers
  marker_array.markers.erase(
      std::remove_if(std::next(marker_array.markers.begin()),
                     marker_array.markers.end(),
                     [](const auto& marker) { return marker.points.empty(); }),
      marker_array.markers.end());

  return marker_array;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_ROS_IMPL_WAVEMAP_2D_SERVER_INL_H_

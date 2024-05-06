#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/query/map_interpolator.h>

#include "wavemap_examples/common.h"

using namespace wavemap;
int main(int, char**) {
  // Create an empty map for illustration purposes
  // NOTE: See the other tutorials on how to load maps from files or ROS topics,
  //       such as the map topic published by the wavemap ROS server.
  MapBase::Ptr map;

  // Declare the point to query [in map frame]
  const Point3D query_point = Point3D::Zero();

  // Query the map and compute the interpolated value
  const FloatingPoint occupancy_log_odds =
      interpolate::trilinear(*map, query_point);
  examples::doSomething(occupancy_log_odds);
}

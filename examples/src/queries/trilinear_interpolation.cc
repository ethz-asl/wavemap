#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/utils/query/map_interpolator.h>

#include "wavemap_examples/common.h"

using namespace wavemap;
int main(int, char**) {
  // Create an empty map for illustration purposes
  // NOTE: See the other tutorials on how to load maps from files or ROS topics,
  //       such as the map topic published by the wavemap ROS server.
  VolumetricDataStructureBase::Ptr map;

  // Declare the point to query [in map frame]
  const Point3D query_point = Point3D::Zero();

  // Query the map and compute the interpolated value
  const FloatingPoint value = interpolate::trilinear(*map, query_point);
  examples::doSomething(value);
}

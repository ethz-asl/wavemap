#include <wavemap/core/map/map_base.h>

#include "wavemap_examples/common.h"

using namespace wavemap;
int main(int, char**) {
  // Declare a map pointer for illustration purposes
  // NOTE: See the other tutorials on how to load maps from files or ROS topics,
  //       such as the map topic published by the wavemap ROS server.
  MapBase::Ptr map;

  // Declare the index to query
  // NOTE: See wavemap/indexing/index_conversions.h for helper methods
  //       to compute and convert indices.
  const Index3D query_index = Index3D::Zero();

  // Query the map
  const FloatingPoint occupancy_log_odds = map->getCellValue(query_index);
  examples::doSomething(occupancy_log_odds);
}

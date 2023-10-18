#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/indexing/index_conversions.h>

#include "wavemap_examples/common.h"

using namespace wavemap;
int main(int, char**) {
  // Declare a map pointer for illustration purposes
  // NOTE: See the other tutorials on how to load maps from files or ROS topics,
  //       such as the map topic published by the wavemap ROS server.
  HashedWaveletOctree::Ptr map;

  // Define the center point and the minimum width of the octree cell to query
  const Point3D query_point = Point3D::Zero();
  const FloatingPoint query_min_cell_width = 0.5f;  // in meters

  // Convert it to an octree node index
  const FloatingPoint map_min_cell_width = map->getMinCellWidth();
  const IndexElement query_height = convert::cellWidthToHeight(
      query_min_cell_width, 1.f / map_min_cell_width);
  const OctreeIndex query_index =
      convert::pointToNodeIndex(query_point, map_min_cell_width, query_height);

  // Query the map
  const FloatingPoint occupancy_log_odds = map->getCellValue(query_index);
  examples::doSomething(occupancy_log_odds);
}

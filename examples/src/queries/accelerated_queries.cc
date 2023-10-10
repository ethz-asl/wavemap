#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/utils/iterate/grid_iterator.h>
#include <wavemap/utils/query/query_accelerator.h>

using namespace wavemap;
int main(int argc, char** argv) {
  // Declare a map pointer for illustration purposes
  // NOTE: See the other tutorials on how to load maps from files or ROS topics,
  //       such as the map topic published by the wavemap ROS server.
  HashedWaveletOctree::Ptr map;

  // Create the query accelerator
  QueryAccelerator query_accelerator(*map);

  // Query all points within a grid
  for (const auto& query_index :
       Grid<3>(Index3D::Constant(-10), Index3D::Constant(10))) {
    const FloatingPoint value = query_accelerator.getCellValue(query_index);
  }
}

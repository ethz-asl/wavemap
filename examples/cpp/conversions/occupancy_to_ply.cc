#include <filesystem>
#include <fstream>
#include <vector>

#include <glog/logging.h>
#include <wavemap/core/common.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/io/file_conversions.h>

using namespace wavemap;  // NOLINT
int main(int argc, char** argv) {
  // Initialize GLOG
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Read params
  if (argc != 2) {
    LOG(ERROR)
        << "Please supply a path to an occupancy map as the first argument.";
    return EXIT_FAILURE;
  }
  const std::filesystem::path map_file_path = argv[1];

  // Load the occupancy map
  MapBase::Ptr occupancy_map;
  if (!io::fileToMap(map_file_path, occupancy_map)) {
    LOG(WARNING) << "Could not open map for reading: " << map_file_path;
    return EXIT_FAILURE;
  }

  // Convert each occupied leaf node to high-res points and add to pointcloud
  std::vector<Point3D> pointcloud;
  constexpr FloatingPoint kOccupancyThreshold = 0.01f;
  const FloatingPoint min_cell_width = occupancy_map->getMinCellWidth();
  occupancy_map->forEachLeaf([min_cell_width, &pointcloud](
                                 const OctreeIndex& node_index,
                                 FloatingPoint node_log_odds) {
    if (kOccupancyThreshold < node_log_odds) {
      if (node_index.height == 0) {
        const Point3D center =
            convert::indexToCenterPoint(node_index.position, min_cell_width);
        pointcloud.emplace_back(center);
      } else {
        const Index3D node_min_corner =
            convert::nodeIndexToMinCornerIndex(node_index);
        const Index3D node_max_corner =
            convert::nodeIndexToMaxCornerIndex(node_index);
        for (const Index3D& index : Grid(node_min_corner, node_max_corner)) {
          const Point3D center =
              convert::indexToCenterPoint(index, min_cell_width);
          pointcloud.emplace_back(center);
        }
      }
    }
  });

  // Create the PLY output file
  const std::filesystem::path ply_file_path =
      std::filesystem::path(map_file_path).replace_extension(".ply");
  LOG(INFO) << "Creating PLY file: " << ply_file_path;
  std::ofstream ply_file(ply_file_path,
                         std::ofstream::out | std::ofstream::binary);
  if (!ply_file.is_open()) {
    LOG(WARNING) << "Could not open file for writing. Error: "
                 << strerror(errno);
    return EXIT_FAILURE;
  }

  // Write the PLY header
  // clang-format off
  ply_file << "ply\n"
              "format ascii 1.0\n"
              "comment The voxel size is " << min_cell_width << " meters.\n"
              "element vertex " << pointcloud.size() << "\n"
              "property float x\n"
              "property float y\n"
              "property float z\n"
              "end_header"
           << std::endl;
  // clang-format on

  // Add the points
  for (const Point3D& point : pointcloud) {
    ply_file << point.x() << " " << point.y() << " " << point.z() << "\n";
  }
  ply_file.flush();

  // Close the file and communicate whether writing succeeded
  ply_file.close();
  return static_cast<bool>(ply_file) ? EXIT_SUCCESS : EXIT_FAILURE;
}

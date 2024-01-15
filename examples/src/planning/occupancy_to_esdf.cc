#include <filesystem>

#include <glog/logging.h>
#include <wavemap/common.h>
#include <wavemap/map/hashed_wavelet_octree.h>
#include <wavemap/map/map_base.h>
#include <wavemap/utils/sdf/quasi_euclidean_sdf_generator.h>
#include <wavemap_io/file_conversions.h>

using namespace wavemap;  // NOLINT
int main(int argc, char** argv) {
  // Initialize GLOG
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
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
  io::fileToMap(map_file_path, occupancy_map);
  CHECK_NOTNULL(occupancy_map);

  // Currently, only hashed wavelet octree maps are supported as input
  const auto hashed_map =
      std::dynamic_pointer_cast<HashedWaveletOctree>(occupancy_map);
  if (!hashed_map) {
    LOG(ERROR)
        << "Only hashed wavelet octree occupancy maps are currently supported.";
  }

  // Generate the ESDF
  const std::filesystem::path esdf_file_path =
      std::filesystem::path(map_file_path).replace_extension(".sdf.wvmp");
  LOG(INFO) << "Generating ESDF";
  constexpr FloatingPoint kOccupancyThreshold = 0.01f;
  constexpr FloatingPoint kMaxDistance = 10.f;
  const QuasiEuclideanSDFGenerator sdf_generator{kMaxDistance,
                                                 kOccupancyThreshold};
  const auto esdf = sdf_generator.generate(*hashed_map);

  // Save the ESDF
  LOG(INFO) << "Saving ESDF to path: " << esdf_file_path;
  if (!io::mapToFile(esdf, esdf_file_path)) {
    LOG(ERROR) << "Could not save ESDF";
    return EXIT_FAILURE;
  }
}

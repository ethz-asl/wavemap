#include <sstream>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "wavemap_2d/data_structure/volumetric/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/volumetric/dense_grid.h"
#include "wavemap_2d/integrator/point_integrator/beam_integrator.h"
#include "wavemap_2d/integrator/point_integrator/ray_integrator.h"

DEFINE_string(carmen_log_file_path, "",
              "Path to the carmen log file to get the input data from.");
DEFINE_string(output_log_dir, "",
              "Path to the directory where the logs should be stored. Leave "
              "blank to disable logging.");
DEFINE_double(map_min_cell_width, 0.01,
              "Grid map minimum cell width in meters.");

using namespace wavemap_2d;  // NOLINT
int main(int argc, char** argv) {
  using DataStructureType = DenseGrid<SaturatingOccupancyCell>;
  using PointcloudIntegratorType = BeamIntegrator;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  // Load and check the input arguments
  const std::string carmen_log_file_path = FLAGS_carmen_log_file_path;
  CHECK(!carmen_log_file_path.empty())
      << "The carmen_log_file_path flag must be set to a non-empty string.";
  const std::string output_log_dir = FLAGS_output_log_dir;
  const auto map_min_cell_width =
      static_cast<FloatingPoint>(FLAGS_map_min_cell_width);
  CHECK_GT(map_min_cell_width, 0.f)
      << "The map_min_cell_width flag must be set to a positive number.";

  // Set up the mapper
  VolumetricDataStructure::Ptr occupancy_map =
      std::make_shared<DataStructureType>(map_min_cell_width);
  PointcloudIntegratorType pointcloud_integrator(occupancy_map);

  // Open the log file
  std::ifstream log_file(carmen_log_file_path);

  // Setup progress reporting
  size_t line_idx = 0u;
  const size_t lines_total =
      std::count(std::istreambuf_iterator<char>(log_file),
                 std::istreambuf_iterator<char>(), '\n');
  log_file.clear();
  log_file.seekg(0);

  // Parse and process the log file
  float max_distance = std::numeric_limits<float>::lowest();
  std::string line;
  std::cout << std::fixed << std::setprecision(0);
  while (std::getline(log_file, line)) {
    ++line_idx;
    std::cout << "\rProcessing line nr: " << line_idx << " ("
              << static_cast<float>(line_idx) /
                     static_cast<float>(lines_total) * 100.f
              << "%)" << std::flush;

    std::istringstream iss(line);
    std::string msg_type;
    if ((iss >> msg_type) && (msg_type == "FLASER")) {
      int num_beams;
      if (iss >> num_beams) {
        // Parse the pointcloud
        Pointcloud pointcloud;
        pointcloud.resize(num_beams);
        {
          bool success = true;
          constexpr auto PI = static_cast<float>(M_PI);
          const float angle_increment = PI / static_cast<float>(num_beams);
          for (int beam_idx = 0; beam_idx < num_beams; ++beam_idx) {
            float distance;
            if (!(iss >> distance)) {
              success = false;
              break;
            }
            max_distance = std::max(distance, max_distance);

            const float angle =
                static_cast<float>(beam_idx) * angle_increment + (PI / 2.f);
            float x = distance * std::cos(angle);
            float y = distance * std::sin(angle);
            pointcloud[beam_idx] << x, y;
          }
          if (!success) {
            LOG(WARNING) << "Could not parse pointcloud... skipping.";
            continue;
          }
        }

        // Parse the position
        Transformation pose;
        if (!(iss >> pose.getPosition().x() >> pose.getPosition().y() >>
              pose.getRotation().angle())) {
          LOG(WARNING) << "Could not parse pose... skipping.";
          continue;
        }
        pose.getRotation().angle() -= M_PIf32;

        // Integrate the pointcloud
        PosedPointcloud posed_pointcloud(pose, pointcloud);
        pointcloud_integrator.integratePointcloud(posed_pointcloud);
      }
    }
  }
  std::cout << std::endl;

  // Save images of the map
  for (const bool use_color : {false, true}) {
    const std::string image_path =
        output_log_dir + (use_color ? "color.png" : "bnw.png");
    occupancy_map->saveImage(image_path, /*use_color*/ use_color);
  }
  for (const bool use_floating_precision : {true, false}) {
    const std::string map_path_prefix =
        output_log_dir + "map" +
        (use_floating_precision ? "_floating" : "_fixed");
    occupancy_map->save(map_path_prefix, use_floating_precision);
  }

  // Print stats
  LOG(INFO) << "Max ray length: " << max_distance;
  occupancy_map->printSize();
  occupancy_map->showImage(/*use_color*/ true);
  cv::waitKey(1000 /*ms*/);

  // Exit cleanly
  return 0;
}

#include <fstream>
#include <sstream>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "wavemap_2d/pointcloud_integrator.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  // Setup the mapper
  constexpr float kResolution = 0.01f;
  auto occupancy_map = std::make_shared<wavemap_2d::OccupancyMap>(kResolution);
  wavemap_2d::PointcloudIntegrator pointcloud_integrator(occupancy_map);

  // Open the log file
  std::string log_file_path = std::string("/home/victor/data/2d_datasets/") +
                              "fr-campus-20040714.carmen.gfs.log";
  //  "fr079-complete.gfs.log";
  //  "mexico.gfs.log";
  //  "csail.corrected.log";
  //  "intel.gfs.log";
  //  "orebro.gfs.log";
  std::ifstream log_file(log_file_path);

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
      int n_beams;
      if (iss >> n_beams) {
        // Parse the pointcloud
        wavemap_2d::Pointcloud pointcloud;
        pointcloud.resize(2, n_beams);
        {
          bool success = true;
          constexpr auto PI = static_cast<float>(M_PI);
          const float angle_increment = PI / static_cast<float>(n_beams);
          for (int beam_idx = 0; beam_idx < n_beams; ++beam_idx) {
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
            pointcloud.col(beam_idx) << x, y;
          }
          if (!success) {
            LOG(WARNING) << "Could not parse pointcloud... skipping.";
            continue;
          }
        }

        // Parse the position
        wavemap_2d::Transformation pose;
        if (!(iss >> pose.getPosition().x() >> pose.getPosition().y() >>
              pose.getRotation().angle())) {
          LOG(WARNING) << "Could not parse pose... skipping.";
          continue;
        }
        pose.getRotation().angle() -= M_PI;

        // Integrate the pointcloud
        wavemap_2d::PosedPointcloud posed_pointcloud(pose, pointcloud);
        pointcloud_integrator.integratePointcloud(posed_pointcloud);
      }
    }
  }

  // Save images of the map
  std::ostringstream image_path_oss;
  image_path_oss << log_file_path + " - res " + "" << kResolution;
  for (const bool use_color : {false, true}) {
    std::string image_path = image_path_oss.str() +
                             std::string(use_color ? "color" : "raw") + ".png";
    occupancy_map->saveImage(image_path, /*use_color*/ use_color);
  }

  // Print stats
  LOG(INFO) << "Max ray length: " << max_distance;
  pointcloud_integrator.printAabbBounds();
  occupancy_map->printSize();
  occupancy_map->showImage(/*use_color*/ true);
  cv::waitKey();

  // Exit cleanly
  return 0;
}

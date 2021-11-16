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
  auto occupancy_map = std::make_shared<wavemap_2d::OccupancyMap>();
  wavemap_2d::PointcloudIntegrator pointcloud_integrator(occupancy_map);

  // Open the log file
  std::string log_file_path = std::string("/home/victor/data/2d_datasets/") +
                              "fr-campus-20040714.carmen.gfs.log";
  //                              "fr079-complete.gfs.log";
  //                              "intel.gfs.log";
  std::ifstream log_file(log_file_path);

  // Parse and process the log file
  std::string line;
  while (std::getline(log_file, line)) {
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

        // Integrate the pointcloud
        wavemap_2d::PosedPointcloud posed_pointcloud(pose, pointcloud);
        pointcloud_integrator.integratePointcloud(posed_pointcloud);
      }
    }
  }

  pointcloud_integrator.printSize();
  pointcloud_integrator.printAabbBounds();

  // Exit cleanly
  return 0;
}

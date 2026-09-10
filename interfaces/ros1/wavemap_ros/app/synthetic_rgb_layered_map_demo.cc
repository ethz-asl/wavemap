#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <glog/logging.h>
#include <ros/ros.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap_msgs/LayeredMap.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>

#include "ouster_layered/rgb_map_layers.h"

namespace wavemap::examples::rgb_map {
namespace {

struct SceneVoxel {
  Index3D index;
  layered::Rgb color;
};

constexpr FloatingPoint kOccupiedLogOdds = 4.f;
constexpr int kFloorCells = 52;
constexpr int kPathHalfWidth = 3;

const layered::Rgb kGrass{0.20f, 0.55f, 0.16f};
const layered::Rgb kPath{0.46f, 0.27f, 0.12f};
const layered::Rgb kTrunk{0.34f, 0.16f, 0.06f};

void appendFloor(std::vector<SceneVoxel>& scene) {
  const int min_coordinate = -kFloorCells / 2;
  const int max_coordinate = min_coordinate + kFloorCells;
  scene.reserve(kFloorCells * kFloorCells);
  for (int y = min_coordinate; y < max_coordinate; ++y) {
    const int path_center =
        static_cast<int>(std::round(2.5 * std::sin(y / 9.0)));
    for (int x = min_coordinate; x < max_coordinate; ++x) {
      const bool is_path = std::abs(x - path_center) <= kPathHalfWidth;
      scene.push_back({Index3D{x, y, 0}, is_path ? kPath : kGrass});
    }
  }
}

void appendTree(const Index2D& center, std::mt19937& generator,
                std::vector<SceneVoxel>& scene) {
  for (int z = 1; z <= 8; ++z) {
    for (int dx = 0; dx < 2; ++dx) {
      for (int dy = 0; dy < 2; ++dy) {
        scene.push_back(
            {Index3D{center.x() + dx, center.y() + dy, z}, kTrunk});
      }
    }
  }

  std::uniform_real_distribution<float> random_value(0.f, 1.f);
  std::uniform_real_distribution<float> green_variation(-0.10f, 0.10f);
  std::unordered_set<Index3D, Index3DHash> added_leaf_indices;
  for (int dz = -2; dz <= 3; ++dz) {
    for (int dx = -4; dx <= 5; ++dx) {
      for (int dy = -4; dy <= 5; ++dy) {
        const float nx = (static_cast<float>(dx) - 0.5f) / 4.6f;
        const float ny = (static_cast<float>(dy) - 0.5f) / 4.6f;
        const float nz = (static_cast<float>(dz) - 0.5f) / 3.2f;
        const float radius = nx * nx + ny * ny + nz * nz;
        const float keep_probability =
            std::clamp(1.25f - radius, 0.f, 0.92f);
        if (random_value(generator) > keep_probability) {
          continue;
        }

        const Index3D index{center.x() + dx, center.y() + dy, 9 + dz};
        if (!added_leaf_indices.emplace(index).second) {
          continue;
        }
        const float variation = green_variation(generator);
        scene.push_back(
            {index,
             layered::Rgb{std::clamp(0.12f + 0.4f * variation, 0.f, 1.f),
                          std::clamp(0.48f + variation, 0.f, 1.f),
                          std::clamp(0.10f + 0.25f * variation, 0.f, 1.f)}});
      }
    }
  }
}

std::vector<SceneVoxel> makeScene() {
  std::vector<SceneVoxel> scene;
  appendFloor(scene);

  std::mt19937 generator(42u);
  for (const Index2D& center : std::vector<Index2D>{
           {-18, -17}, {-17, 14}, {-11, -4}, {12, -17}, {16, 13}, {10, 4}}) {
    appendTree(center, generator, scene);
  }
  return scene;
}

}  // namespace

class SyntheticRgbLayeredMapDemo {
 public:
  SyntheticRgbLayeredMapDemo()
      : private_nh_("~"), map_(makeMapConfig()), scene_(makeScene()) {
    private_nh_.param("voxels_per_update", voxels_per_update_, 120);
    private_nh_.param("update_period", update_period_, 0.08);
    private_nh_.param("save_path", save_path_, std::string{});
    voxels_per_update_ = std::max(1, voxels_per_update_);
    update_period_ = std::max(0.01, update_period_);

    publisher_ = private_nh_.advertise<wavemap_msgs::LayeredMap>(
        "layered_map", 1, true);
    timer_ = private_nh_.createTimer(
        ros::Duration(update_period_),
        &SyntheticRgbLayeredMapDemo::insertNextVoxels, this);

    ROS_INFO_STREAM("Synthetic RGB Layered Wavemap contains " << scene_.size()
                    << " voxels and will be revealed in batches of "
                    << voxels_per_update_ << ".");
  }

 private:
  void insertNextVoxels(const ros::TimerEvent&) {
    const size_t next_end = std::min(
        next_voxel_ + static_cast<size_t>(voxels_per_update_), scene_.size());
    for (; next_voxel_ < next_end; ++next_voxel_) {
      Definition::ContinuousLayers layers;
      layers.get<RgbLayer>() = scene_[next_voxel_].color;
      map_.continuousMap().setVoxelValue(
          scene_[next_voxel_].index,
          Definition::Voxel{kOccupiedLogOdds, layers});
    }
    publishMap();

    if (next_voxel_ == scene_.size()) {
      timer_.stop();
      map_.continuousMap().threshold();
      map_.continuousMap().prune();
      publishMap();
      saveMapIfRequested();
      ROS_INFO("Synthetic RGB Layered Wavemap is complete.");
    }
  }

  void publishMap() {
    wavemap_msgs::LayeredMap message;
    using RosConverter =
        convert::DescriptorContinuousRosConverter<Definition::Voxel>;
    if (!convert::layeredMapToRosMsg<Definition::Map, RosConverter>(
            map_, "map", ros::Time::now(), message)) {
      ROS_ERROR("Could not convert the synthetic RGB map to a ROS message.");
      return;
    }
    publisher_.publish(message);
  }

  void saveMapIfRequested() {
    if (save_path_.empty()) {
      return;
    }
    const std::filesystem::path output_path(save_path_);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
    if (Definition::MapIo::save(output_path, map_)) {
      ROS_INFO_STREAM("Saved synthetic RGB Layered Wavemap to "
                      << output_path);
    } else {
      ROS_ERROR_STREAM("Could not save synthetic RGB Layered Wavemap to "
                       << output_path);
    }
  }

  ros::NodeHandle private_nh_;
  Definition::Map map_;
  std::vector<SceneVoxel> scene_;
  size_t next_voxel_ = 0u;
  int voxels_per_update_ = 120;
  double update_period_ = 0.08;
  std::string save_path_;
  ros::Publisher publisher_;
  ros::Timer timer_;
};

}  // namespace wavemap::examples::rgb_map

int main(int argc, char** argv) {
  ros::init(argc, argv, "synthetic_rgb_layered_map");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  wavemap::examples::rgb_map::SyntheticRgbLayeredMapDemo demo;
  ros::spin();
  return 0;
}

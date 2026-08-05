#include <iostream>
#include <map>
#include <string>
#include <tuple>

#include "../../common/layered_voxel_config.h"

struct IndexKey {
  int x = 0;
  int y = 0;
  int z = 0;

  explicit IndexKey(const wavemap::Index3D& index)
      : x(index.x()), y(index.y()), z(index.z()) {}

  bool operator<(const IndexKey& other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

void printVoxelWithLabel(const std::string& name, const LayeredVoxel& voxel,
                         int label) {
  std::cout << name << ": occ=" << voxel.occupancy
            << " trav=" << voxel.data.traversability << " rgb=("
            << voxel.data.rgb.r << ", " << voxel.data.rgb.g << ", "
            << voxel.data.rgb.b << ") label=" << label << "\n";
}

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 3;

  ContinuousWaveletMap map(config);
  std::map<IndexKey, int> labels;

  const wavemap::Index3D a(1, 2, 3);
  const wavemap::Index3D b(2, 2, 3);
  const wavemap::Index3D c(3, 2, 3);

  map.setVoxelValue(a,
                    makeLayeredVoxel(10.f, rgb(1.f, 0.f, 0.f),
                                      0.1f));
  map.setVoxelValue(b,
                    makeLayeredVoxel(20.f, rgb(0.f, 1.f, 0.f),
                                      0.5f));
  map.setVoxelValue(c,
                    makeLayeredVoxel(30.f, rgb(0.f, 0.f, 1.f),
                                      0.9f));
  labels[IndexKey(a)] = 1;
  labels[IndexKey(b)] = 5;
  labels[IndexKey(c)] = 9;

  std::cout << "Before threshold/prune\n";
  printVoxelWithLabel("a", map.getVoxelValue(a), labels[IndexKey(a)]);
  printVoxelWithLabel("b", map.getVoxelValue(b), labels[IndexKey(b)]);
  printVoxelWithLabel("c", map.getVoxelValue(c), labels[IndexKey(c)]);
  std::cout << "label entries: " << labels.size() << "\n";
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  map.threshold();
  map.prune();

  std::cout << "After threshold/prune\n";
  printVoxelWithLabel("a", map.getVoxelValue(a), labels[IndexKey(a)]);
  printVoxelWithLabel("b", map.getVoxelValue(b), labels[IndexKey(b)]);
  printVoxelWithLabel("c", map.getVoxelValue(c), labels[IndexKey(c)]);
  std::cout << "label entries: " << labels.size() << "\n";
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  return 0;
}

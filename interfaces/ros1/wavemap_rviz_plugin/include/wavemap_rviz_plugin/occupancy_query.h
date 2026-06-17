#ifndef WAVEMAP_RVIZ_PLUGIN_OCCUPANCY_QUERY_H_
#define WAVEMAP_RVIZ_PLUGIN_OCCUPANCY_QUERY_H_

#include <memory>

#include <wavemap/core/common.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/query/query_accelerator.h>

namespace wavemap::rviz_plugin {
// Small runtime interface for map occupancy queries. The cell selector only
// needs this API, so it does not need to know whether the backing map is a
// legacy wavemap map or a future layered map adapter.
class OccupancyQuery {
 public:
  virtual ~OccupancyQuery() = default;

  virtual FloatingPoint getCellValue(const OctreeIndex& index) = 0;
};

class HashedWaveletOctreeOccupancyQuery : public OccupancyQuery {
 public:
  explicit HashedWaveletOctreeOccupancyQuery(
      std::shared_ptr<const HashedWaveletOctree> map)
      : map_(std::move(map)), query_accelerator_(*map_) {}

  FloatingPoint getCellValue(const OctreeIndex& index) override {
    return query_accelerator_.getCellValue(index);
  }

 private:
  std::shared_ptr<const HashedWaveletOctree> map_;
  QueryAccelerator<HashedWaveletOctree> query_accelerator_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_OCCUPANCY_QUERY_H_

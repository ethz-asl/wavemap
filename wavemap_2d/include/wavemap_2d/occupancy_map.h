#ifndef WAVEMAP_2D_OCCUPANCY_MAP_H_
#define WAVEMAP_2D_OCCUPANCY_MAP_H_

#include <memory>

namespace wavemap_2d {
class OccupancyMap {
 public:
  using Ptr = std::shared_ptr<OccupancyMap>;

  OccupancyMap() = default;

  bool empty() { return !grid_map_.size(); }
  Index size() { return Index{grid_map_.rows(), grid_map_.cols()}; }

  void updateCell(const Index& index, const FloatingPoint update) {
    if (empty()) {
      grid_map_min_index_ = index;
    }

    // TODO(victorr): Add check for overflows
    Index relative_index = index - grid_map_min_index_;

    // TODO(victorr): Clean up this section
    if ((relative_index.array() < 0).any() ||
        grid_map_.rows() < relative_index.x() + 1 ||
        grid_map_.cols() < relative_index.y() + 1) {
      const Index grid_map_size{grid_map_.rows(), grid_map_.cols()};
      const Index grid_map_max_index = grid_map_min_index_ + grid_map_size;

      const Index new_grid_map_max_index = grid_map_max_index.cwiseMax(index);
      const Index new_grid_map_min_index = grid_map_min_index_.cwiseMin(index);
      const Index min_index_diff = grid_map_min_index_ - new_grid_map_min_index;

      const Index new_grid_map_size =
          new_grid_map_max_index - new_grid_map_min_index + Index::Ones();

      GridMapType new_grid_map =
          GridMapType::Zero(new_grid_map_size.x(), new_grid_map_size.y());

      new_grid_map.block(min_index_diff.x(), min_index_diff.y(),
                         grid_map_size.x(), grid_map_size.y()) = grid_map_;

      grid_map_.swap(new_grid_map);
      grid_map_min_index_ = new_grid_map_min_index;
      relative_index = index - new_grid_map_min_index;
    }

    grid_map_.coeffRef(relative_index.x(), relative_index.y()) += update;
  }

 protected:
  Index grid_map_min_index_;
  using GridMapType =
      Eigen::Matrix<FloatingPoint, Eigen::Dynamic, Eigen::Dynamic>;
  GridMapType grid_map_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_OCCUPANCY_MAP_H_

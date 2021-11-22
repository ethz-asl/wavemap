#ifndef WAVEMAP_2D_MAP_H_
#define WAVEMAP_2D_MAP_H_

#include <algorithm>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
class GridMap {
 public:
  using Ptr = std::shared_ptr<GridMap>;

  explicit GridMap(const FloatingPoint resolution)
      : resolution_(resolution),
        grid_map_min_index_(Index::Constant(NAN)),
        grid_map_max_index_(Index::Constant(NAN)) {}

  bool empty() const { return !data_.size(); }
  Index size() const { return {data_.rows(), data_.cols()}; }
  void clear();

  Index getMinIndex() const { return grid_map_min_index_; }
  Index getMaxIndex() const { return grid_map_max_index_; }
  bool mapContains(const Index& index) const {
    return (grid_map_min_index_.array() <= index.array() &&
            index.array() <= grid_map_max_index_.array())
        .all();
  }
  FloatingPoint getResolution() const { return resolution_; }

  void updateCell(const Index& index, const FloatingPoint update);
  FloatingPoint getCellValue(const Index& index) const;

  void printSize() const { LOG(INFO) << "Size:\n" << size(); }

  cv::Mat getImage(bool use_color = false) const;
  void showImage(bool use_color = false) const;
  void saveImage(const std::string& file_path, bool use_color = false) const;

 protected:
  const FloatingPoint resolution_;

  Index grid_map_min_index_;
  Index grid_map_max_index_;
  Index getDataIndex(const Index& index) const {
    // TODO(victorr): Add check for overflows
    // TODO(victorr): Consider making a separate DataIndex type s.t. mixing up
    //                relative/absolute indices throws a compiler error
    return index - grid_map_min_index_;
  }

  using GridDataStructure =
      Eigen::Matrix<FloatingPoint, Eigen::Dynamic, Eigen::Dynamic>;
  GridDataStructure data_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_MAP_H_

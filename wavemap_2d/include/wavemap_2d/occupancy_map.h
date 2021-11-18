#ifndef WAVEMAP_2D_OCCUPANCY_MAP_H_
#define WAVEMAP_2D_OCCUPANCY_MAP_H_

#include <algorithm>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
class OccupancyMap {
 public:
  using Ptr = std::shared_ptr<OccupancyMap>;

  explicit OccupancyMap(const FloatingPoint resolution)
      : resolution_(resolution) {}

  bool empty() const { return !grid_map_.size(); }
  Index size() const { return Index{grid_map_.rows(), grid_map_.cols()}; }
  FloatingPoint getResolution() const { return resolution_; }

  void updateCell(const Index& index, const FloatingPoint update);

  void printSize() const { LOG(INFO) << "Size:\n" << size(); }

  cv::Mat getImage(bool use_color = false) const;
  void showImage(bool use_color = false) const;
  void saveImage(const std::string& file_path, bool use_color = false) const;

 protected:
  const FloatingPoint resolution_;

  Index grid_map_min_index_;
  using GridMapType =
      Eigen::Matrix<FloatingPoint, Eigen::Dynamic, Eigen::Dynamic>;
  GridMapType grid_map_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_OCCUPANCY_MAP_H_

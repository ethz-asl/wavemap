#include "wavemap_2d/occupancy_map.h"

namespace wavemap_2d {
void OccupancyMap::updateCell(const Index& index, const FloatingPoint update) {
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
    const Index grid_map_max_index =
        grid_map_min_index_ + grid_map_size - Index::Ones();

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

cv::Mat OccupancyMap::getImage(bool use_color) const {
  cv::Mat image;
  if (use_color) {
    constexpr FloatingPoint kLogOddsMin = -1e2;  //  -100.f;
    constexpr FloatingPoint kLogOddsMax = 2e2;   //   200.f;
    const FloatingPoint min = std::max(grid_map_.minCoeff(), kLogOddsMin);
    const FloatingPoint max = std::min(grid_map_.maxCoeff(), kLogOddsMax);
    GridMapType grid_map_clamped =
        grid_map_.cwiseMin(kLogOddsMax).cwiseMax(kLogOddsMin);

    cv::eigen2cv(grid_map_clamped, image);
    image.convertTo(image, CV_8UC1, 255 / (max - min), -min);
    cv::applyColorMap(image, image, cv::ColormapTypes::COLORMAP_JET);
  } else {
    cv::eigen2cv(grid_map_, image);
  }

  return image;
}

void OccupancyMap::showImage(bool use_color) const {
  cv::namedWindow("Grid map", cv::WINDOW_NORMAL);
  cv::setWindowProperty("Grid map", CV_WND_PROP_FULLSCREEN,
                        CV_WINDOW_FULLSCREEN);
  cv::imshow("Grid map", getImage(use_color));
  cv::waitKey(1 /* ms */);
}
void OccupancyMap::saveImage(const std::string& file_path,
                             bool use_color) const {
  cv::imwrite(file_path, getImage(use_color));
}
}  // namespace wavemap_2d

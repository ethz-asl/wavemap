#include "wavemap_2d/map.h"

namespace wavemap_2d {
void GridMap::updateCell(const Index& index, const FloatingPoint update) {
  if (empty()) {
    grid_map_min_index_ = index;
    grid_map_max_index_ = index;
    data_ = GridDataStructure::Zero(1, 1);
  }

  if (!mapContains(index)) {
    const Index new_grid_map_max_index = grid_map_max_index_.cwiseMax(index);
    const Index new_grid_map_min_index = grid_map_min_index_.cwiseMin(index);
    const Index min_index_diff = grid_map_min_index_ - new_grid_map_min_index;

    const Index new_grid_map_size =
        new_grid_map_max_index - new_grid_map_min_index + Index::Ones();
    GridDataStructure new_grid_map =
        GridDataStructure::Zero(new_grid_map_size.x(), new_grid_map_size.y());

    new_grid_map.block(min_index_diff.x(), min_index_diff.y(), size().x(),
                       size().y()) = data_;

    data_.swap(new_grid_map);
    grid_map_min_index_ = new_grid_map_min_index;
    grid_map_max_index_ = new_grid_map_max_index;
  }

  const Index data_index = index - grid_map_min_index_;
  data_.coeffRef(data_index.x(), data_index.y()) += update;
}

FloatingPoint GridMap::getCellValue(const Index& index) const {
  if (mapContains(index)) {
    const Index data_index = getDataIndex(index);
    return data_(data_index.x(), data_index.y());
  } else {
    return 0.f;
  }
}

cv::Mat GridMap::getImage(bool use_color) const {
  cv::Mat image;
  if (use_color) {
    constexpr FloatingPoint kLogOddsMin = -4.f;
    constexpr FloatingPoint kLogOddsMax = 4.f;
    GridDataStructure grid_map_clamped =
        data_.cwiseMin(kLogOddsMax).cwiseMax(kLogOddsMin);

    cv::eigen2cv(grid_map_clamped, image);
    image.convertTo(image, CV_8UC1, 255 / (kLogOddsMax - kLogOddsMin),
                    -kLogOddsMin);
    cv::applyColorMap(image, image, cv::ColormapTypes::COLORMAP_JET);
  } else {
    cv::eigen2cv(data_, image);
  }

  return image;
}

void GridMap::showImage(bool use_color) const {
  cv::namedWindow("Grid map", cv::WINDOW_NORMAL);
  cv::setWindowProperty("Grid map", CV_WND_PROP_FULLSCREEN,
                        CV_WINDOW_FULLSCREEN);
  cv::imshow("Grid map", getImage(use_color));
  cv::waitKey(1 /* ms */);
}
void GridMap::saveImage(const std::string& file_path, bool use_color) const {
  cv::imwrite(file_path, getImage(use_color));
}
}  // namespace wavemap_2d

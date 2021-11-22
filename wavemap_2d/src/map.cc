#include "wavemap_2d/map.h"

namespace wavemap_2d {
void GridMap::clear() {
  data_.resize(0, 0);
  grid_map_min_index_ = Index::Constant(NAN);
  grid_map_max_index_ = Index::Constant(NAN);
}

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

void GridMap::saveImage(const std::string& file_path,
                        const bool use_color) const {
  cv::imwrite(file_path, getImage(/*use_color*/ true));
}

bool GridMap::saveMap(const std::string& file_path_prefix) const {
  const std::string header_file_path =
      getHeaderFilePathFromPrefix(file_path_prefix);
  const std::string data_file_path =
      getDataFilePathFromPrefix(file_path_prefix);

  std::ofstream header_file;
  header_file.open(header_file_path);
  if (!header_file.is_open()) {
    LOG(ERROR) << "Could not open header file \"" << header_file_path
               << "\" for writing.";
    return false;
  }
  header_file << resolution_ << "\n"
              << grid_map_min_index_ << "\n"
              << grid_map_max_index_;
  header_file.close();

  cv::Mat image;
  cv::eigen2cv(data_, image);
  cv::imwrite(data_file_path, image);

  return true;
}

bool GridMap::loadMap(const std::string& file_path_prefix) {
  const std::string header_file_path =
      getHeaderFilePathFromPrefix(file_path_prefix);
  const std::string data_file_path =
      getDataFilePathFromPrefix(file_path_prefix);

  std::ifstream header_file;
  header_file.open(header_file_path);
  if (!header_file.is_open()) {
    LOG(ERROR) << "Could not open header file \"" << header_file_path
               << "\" for reading.";
    return false;
  }

  FloatingPoint resolution;
  header_file >> resolution;
  if (1e-3 < std::abs(resolution - resolution_)) {
    LOG(ERROR) << "Tried to load a map whose resolution (" << resolution
               << ") does not match the configured resolution (" << resolution_
               << ").";
    return false;
  }
  header_file >> grid_map_min_index_.x() >> grid_map_min_index_.y();
  header_file >> grid_map_max_index_.x() >> grid_map_max_index_.y();
  header_file.close();

  cv::Mat image =
      cv::imread(data_file_path, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
  if (image.empty()) {
    LOG(ERROR) << "Could not read map data file \"" << data_file_path << "\".";
    return false;
  }
  cv::cv2eigen(image, data_);

  return true;
}
}  // namespace wavemap_2d

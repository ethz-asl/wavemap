#ifndef WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_DENSE_GRID_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_DENSE_GRID_INL_H_

#include <algorithm>
#include <string>

#include "wavemap_2d/datastructure/datastructure_base.h"

namespace wavemap_2d {
template <typename CellTypeT>
void DenseGrid<CellTypeT>::updateCell(const Index& index,
                                      const FloatingPoint update) {
  if (empty()) {
    min_index_ = index;
    max_index_ = index;
    data_ = DataGridSpecialized::Zero(1, 1);
  }

  if (!containsIndex(index)) {
    const Index new_grid_map_max_index = max_index_.cwiseMax(index);
    const Index new_grid_map_min_index = min_index_.cwiseMin(index);
    const Index min_index_diff = min_index_ - new_grid_map_min_index;

    const Index new_grid_map_size =
        new_grid_map_max_index - new_grid_map_min_index + Index::Ones();
    DataGridSpecialized new_grid_map =
        DataGridSpecialized::Zero(new_grid_map_size.x(), new_grid_map_size.y());

    new_grid_map.block(min_index_diff.x(), min_index_diff.y(), size().x(),
                       size().y()) = data_;

    data_.swap(new_grid_map);
    min_index_ = new_grid_map_min_index;
    max_index_ = new_grid_map_max_index;
  }

  auto& cell_data = accessCellData(index);
  if (CellTypeT::hasLowerBound && CellTypeT::hasUpperBound) {
    cell_data = std::max(CellTypeT::kLowerBound,
                         std::min(cell_data + update, CellTypeT::kUpperBound));
  } else if (CellTypeT::hasUpperBound) {
    cell_data = std::min(cell_data + update, CellTypeT::kUpperBound);
  } else if (CellTypeT::hasLowerBound) {
    cell_data = std::max(CellTypeT::kLowerBound, cell_data + update);
  } else {
    cell_data += update;
  }
}

template <typename CellTypeT>
FloatingPoint DenseGrid<CellTypeT>::getCellValue(const Index& index) const {
  if (containsIndex(index)) {
    return static_cast<FloatingPoint>(accessCellData(index));
  } else {
    return 0.f;
  }
}

template <typename CellTypeT>
cv::Mat DenseGrid<CellTypeT>::getImage(bool use_color) const {
  cv::Mat image;
  constexpr FloatingPoint kLogOddsMin =
      CellTypeT::hasLowerBound ? CellTypeT::kLowerBound : -2.f;
  constexpr FloatingPoint kLogOddsMax =
      CellTypeT::hasUpperBound ? CellTypeT::kUpperBound : 4.f;
  const DataGridBaseFloat grid_map_tmp =
      data_.template cast<CellDataBaseFloat>();
  cv::eigen2cv(grid_map_tmp, image);
  image.convertTo(image, CV_8UC1, 255.f / (kLogOddsMax - kLogOddsMin),
                  -kLogOddsMin);
  if (use_color) {
    cv::applyColorMap(image, image, cv::ColormapTypes::COLORMAP_PARULA);
  }

  return image;
}

template <typename CellTypeT>
bool DenseGrid<CellTypeT>::save(const std::string& file_path_prefix,
                                bool use_floating_precision) const {
  const std::string header_file_path = getHeaderFilePath(file_path_prefix);
  const std::string data_file_path =
      getDataFilePath(file_path_prefix, use_floating_precision);

  std::ofstream header_file;
  header_file.open(header_file_path);
  if (!header_file.is_open()) {
    LOG(ERROR) << "Could not open header file \"" << header_file_path
               << "\" for writing.";
    return false;
  }
  header_file << resolution_ << "\n" << min_index_ << "\n" << max_index_;
  header_file.close();

  cv::Mat image;
  if (use_floating_precision) {
    const DataGridBaseFloat data_tmp = data_.template cast<CellDataBaseFloat>();
    cv::eigen2cv(data_tmp, image);
  } else {
    if (CellTypeT::isFullyBounded) {
      const DataGridBaseInt data_rescaled =
          ((data_.array() - CellTypeT::kLowerBound) *
           CellTypeT::kSpecializedToBaseIntScalingFactor)
              .template cast<CellDataBaseInt>();
      cv::eigen2cv(data_rescaled, image);
    } else {
      const DataGridBaseInt data_tmp =
          data_.array().round().template cast<CellDataBaseInt>();
      cv::eigen2cv(data_tmp, image);
    }
  }
  cv::imwrite(data_file_path, image);

  return true;
}

template <typename CellTypeT>
bool DenseGrid<CellTypeT>::load(const std::string& file_path_prefix,
                                bool used_floating_precision) {
  const std::string header_file_path = getHeaderFilePath(file_path_prefix);
  const std::string data_file_path =
      getDataFilePath(file_path_prefix, used_floating_precision);

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
  header_file >> min_index_.x() >> min_index_.y();
  header_file >> max_index_.x() >> max_index_.y();
  header_file.close();

  cv::Mat image = cv::imread(data_file_path, cv::IMREAD_ANYDEPTH);
  if (image.empty()) {
    LOG(ERROR) << "Could not read map data file \"" << data_file_path << "\".";
    return false;
  }
  if (used_floating_precision) {
    DataGridBaseFloat data_tmp;
    cv::cv2eigen(image, data_tmp);
    data_ = data_tmp.template cast<CellDataSpecialized>();

  } else {
    DataGridBaseInt data_tmp;
    cv::cv2eigen(image, data_tmp);
    if (CellTypeT::isFullyBounded) {
      data_ = (data_tmp.template cast<CellDataSpecialized>().array() /
               CellTypeT::kSpecializedToBaseIntScalingFactor) +
              CellTypeT::kLowerBound;
    } else {
      data_ = data_tmp.template cast<CellDataSpecialized>();
    }
  }

  return true;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_DENSE_GRID_INL_H_

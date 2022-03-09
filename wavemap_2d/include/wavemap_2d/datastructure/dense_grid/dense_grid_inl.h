#ifndef WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_DENSE_GRID_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_DENSE_GRID_INL_H_

#include <algorithm>
#include <string>

#include <opencv2/core/eigen.hpp>

#include "wavemap_2d/datastructure/datastructure_base.h"
#include "wavemap_2d/utils/image_utils.h"

namespace wavemap_2d {
template <typename CellT>
void DenseGrid<CellT>::clear() {
  data_.resize(0, 0);
  min_index_ = Index::Zero();
  max_index_ = Index::Zero();
}

template <typename CellT>
bool DenseGrid<CellT>::hasCell(const Index& index) const {
  if (!empty()) {
    return (min_index_.array() <= index.array() &&
            index.array() <= max_index_.array())
        .all();
  }
  return false;
}

template <typename CellT>
FloatingPoint DenseGrid<CellT>::getCellValue(const Index& index) const {
  const CellDataSpecialized* cell_data = accessCellData(index);
  if (cell_data) {
    return static_cast<FloatingPoint>(*cell_data);
  } else {
    return 0.f;
  }
}

template <typename CellT>
void DenseGrid<CellT>::setCellValue(const Index& index,
                                    FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  CellDataSpecialized* cell_data = accessCellData(index, kAutoAllocate);
  if (cell_data) {
    // TODO(victorr): Decide whether truncation should be applied here as well
    *cell_data = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellT>
void DenseGrid<CellT>::addToCellValue(const Index& index,
                                      FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  CellDataSpecialized* cell_data = accessCellData(index, kAutoAllocate);
  if (cell_data) {
    *cell_data = CellT::add(*cell_data, update);
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellT>
cv::Mat DenseGrid<CellT>::getImage(bool use_color) const {
  constexpr FloatingPoint kLogOddsMin =
      CellT::hasLowerBound ? CellT::kLowerBound : -2.f;
  constexpr FloatingPoint kLogOddsMax =
      CellT::hasUpperBound ? CellT::kUpperBound : 4.f;
  const DataGridBaseFloat grid_map_tmp =
      data_.template cast<CellDataBaseFloat>();
  return MatrixToImage(grid_map_tmp, kLogOddsMin, kLogOddsMax, use_color);
}

template <typename CellT>
bool DenseGrid<CellT>::save(const std::string& file_path_prefix,
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
    if (CellT::isFullyBounded) {
      const DataGridBaseInt data_rescaled =
          ((data_.array() - CellT::kLowerBound) *
           CellT::kSpecializedToBaseIntScalingFactor)
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

template <typename CellT>
bool DenseGrid<CellT>::load(const std::string& file_path_prefix,
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
    if (CellT::isFullyBounded) {
      data_ = (data_tmp.template cast<CellDataSpecialized>().array() /
               CellT::kSpecializedToBaseIntScalingFactor) +
              CellT::kLowerBound;
    } else {
      data_ = data_tmp.template cast<CellDataSpecialized>();
    }
  }

  return true;
}

template <typename CellT>
typename CellT::Specialized* DenseGrid<CellT>::accessCellData(
    const Index& index, bool auto_allocate) {
  if (empty()) {
    if (auto_allocate) {
      min_index_ = index;
      max_index_ = index;
      data_ = DataGridSpecialized::Zero(1, 1);
    } else {
      // TODO(victorr): Add unit test
      return nullptr;
    }
  }

  if (!hasCell(index)) {
    if (auto_allocate) {
      const Index new_grid_map_max_index = max_index_.cwiseMax(index);
      const Index new_grid_map_min_index = min_index_.cwiseMin(index);
      const Index min_index_diff = min_index_ - new_grid_map_min_index;

      const Index new_grid_map_dim =
          new_grid_map_max_index - new_grid_map_min_index + Index::Ones();
      DataGridSpecialized new_grid_map =
          DataGridSpecialized::Zero(new_grid_map_dim.x(), new_grid_map_dim.y());

      new_grid_map.block(min_index_diff.x(), min_index_diff.y(), data_.rows(),
                         data_.cols()) = data_;

      data_.swap(new_grid_map);
      min_index_ = new_grid_map_min_index;
      max_index_ = new_grid_map_max_index;
    } else {
      // TODO(victorr): Add unit test
      return nullptr;
    }
  }

  // TODO(victorr): Add check for overflows
  const Index data_index = index - min_index_;
  return &data_.coeffRef(data_index.x(), data_index.y());
}

template <typename CellT>
const typename CellT::Specialized* DenseGrid<CellT>::accessCellData(
    const Index& index) const {
  if (empty() || !hasCell(index)) {
    // TODO(victorr): Add unit test
    return nullptr;
  }
  // TODO(victorr): Add check for overflows
  const Index data_index = index - min_index_;
  return &data_.coeff(data_index.x(), data_index.y());
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_DENSE_GRID_INL_H_

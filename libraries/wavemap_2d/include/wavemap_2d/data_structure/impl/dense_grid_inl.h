#ifndef WAVEMAP_2D_DATA_STRUCTURE_IMPL_DENSE_GRID_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_IMPL_DENSE_GRID_INL_H_

#include <algorithm>
#include <string>
#include <utility>

#include <opencv2/core/eigen.hpp>
#include <wavemap_common/indexing/index_conversions.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"
#include "wavemap_2d/iterator/grid_iterator.h"
#include "wavemap_2d/utils/image_utils.h"

namespace wavemap {
template <typename CellT>
void DenseGrid<CellT>::clear() {
  data_.resize(0, 0);
  min_external_index_ = Index2D::Zero();
  max_external_index_ = Index2D::Zero();
}

template <typename CellT>
void DenseGrid<CellT>::prune() {
  if (empty()) {
    return;
  }

  bool has_non_zero_cell = false;
  Index2D min_non_zero_index = getMaxInternalIndex();
  Index2D max_non_zero_index = getMinInternalIndex();
  for (const Index2D& cell_index :
       Grid(getMinInternalIndex(), getMaxInternalIndex())) {
    const CellDataSpecialized& cell_value =
        data_(cell_index.x(), cell_index.y());
    if (cell_value != typename CellT::Specialized{}) {
      has_non_zero_cell = true;
      min_non_zero_index = min_non_zero_index.cwiseMin(cell_index);
      max_non_zero_index = max_non_zero_index.cwiseMax(cell_index);
    }
  }

  if (has_non_zero_cell) {
    const Index2D new_size =
        max_non_zero_index - min_non_zero_index + Index2D::Ones();
    DataGridSpecialized new_grid_map =
        data_.block(min_non_zero_index.x(), min_non_zero_index.y(),
                    new_size.x(), new_size.y());
    data_.template swap(new_grid_map);
    min_external_index_ += min_non_zero_index;
    max_external_index_ = min_external_index_ + max_non_zero_index;
  } else {
    clear();
  }
}

template <typename CellT>
bool DenseGrid<CellT>::hasCell(const Index2D& index) const {
  if (!empty()) {
    return (min_external_index_.array() <= index.array() &&
            index.array() <= max_external_index_.array())
        .all();
  }
  return false;
}

template <typename CellT>
FloatingPoint DenseGrid<CellT>::getCellValue(const Index2D& index) const {
  const CellDataSpecialized* cell_data = accessCellData(index);
  if (cell_data) {
    return static_cast<FloatingPoint>(*cell_data);
  } else {
    return 0.f;
  }
}

template <typename CellT>
void DenseGrid<CellT>::setCellValue(const Index2D& index,
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
void DenseGrid<CellT>::addToCellValue(const Index2D& index,
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
void DenseGrid<CellT>::forEachLeaf(
    VolumetricDataStructure2D::IndexedLeafVisitorFunction visitor_fn) const {
  for (const Index2D& internal_cell_index :
       Grid(getMinInternalIndex(), getMaxInternalIndex())) {
    const CellDataSpecialized& cell_data =
        data_(internal_cell_index.x(), internal_cell_index.y());
    const Index2D cell_index = toExternal(internal_cell_index);
    const QuadtreeIndex hierarchical_cell_index =
        convert::indexAndHeightToNodeIndex(cell_index, 0);
    visitor_fn(hierarchical_cell_index, cell_data);
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
  header_file << min_cell_width_ << "\n"
              << min_external_index_ << "\n"
              << max_external_index_;
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

  FloatingPoint min_cell_width;
  header_file >> min_cell_width;
  if (1e-3f < std::abs(min_cell_width - min_cell_width_)) {
    LOG(ERROR) << "Tried to load a map whose minimum cell width ("
               << min_cell_width
               << ") does not match the configured minimum cell width ("
               << min_cell_width_ << ").";
    return false;
  }
  header_file >> min_external_index_.x() >> min_external_index_.y();
  header_file >> max_external_index_.x() >> max_external_index_.y();
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
    const Index2D& index, bool auto_allocate) {
  if (empty()) {
    if (auto_allocate) {
      min_external_index_ = index;
      max_external_index_ = index;
      data_ = DataGridSpecialized::Zero(1, 1);
    } else {
      // TODO(victorr): Add unit test
      return nullptr;
    }
  }

  if (!hasCell(index)) {
    if (auto_allocate) {
      const Index2D new_grid_map_max_index =
          max_external_index_.cwiseMax(index);
      const Index2D new_grid_map_min_index =
          min_external_index_.cwiseMin(index);
      const Index2D min_index_diff =
          min_external_index_ - new_grid_map_min_index;

      const Index2D new_grid_map_dim =
          new_grid_map_max_index - new_grid_map_min_index + Index2D::Ones();
      DataGridSpecialized new_grid_map =
          DataGridSpecialized::Zero(new_grid_map_dim.x(), new_grid_map_dim.y());

      new_grid_map.block(min_index_diff.x(), min_index_diff.y(), data_.rows(),
                         data_.cols()) = data_;

      data_.swap(new_grid_map);
      min_external_index_ = new_grid_map_min_index;
      max_external_index_ = new_grid_map_max_index;
    } else {
      return nullptr;
    }
  }

  const Index2D internal_index = toInternal(index);
  return &data_.coeffRef(internal_index.x(), internal_index.y());
}

template <typename CellT>
const typename CellT::Specialized* DenseGrid<CellT>::accessCellData(
    const Index2D& index) const {
  if (empty() || !hasCell(index)) {
    return nullptr;
  }
  const Index2D internal_index = toInternal(index);
  return &data_.coeff(internal_index.x(), internal_index.y());
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_IMPL_DENSE_GRID_INL_H_

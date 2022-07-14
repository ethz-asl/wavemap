#ifndef WAVEMAP_2D_DATA_STRUCTURE_DENSE_GRID_H_
#define WAVEMAP_2D_DATA_STRUCTURE_DENSE_GRID_H_

#include <string>

#include <wavemap_common/common.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"

namespace wavemap {
template <typename CellT>
class DenseGrid : public VolumetricDataStructure2D {
 public:
  using CellType = CellT;
  using CellDataSpecialized = typename CellT::Specialized;
  using CellDataBaseFloat = typename CellT::BaseFloat;
  using CellDataBaseInt = typename CellT::BaseInt;
  static constexpr bool kRequiresPruningForThresholding = false;

  template <typename T>
  using DataGrid = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
  using DataGridSpecialized = DataGrid<CellDataSpecialized>;
  using DataGridBaseFloat = DataGrid<CellDataBaseFloat>;
  using DataGridBaseInt = DataGrid<CellDataBaseInt>;

  // Use the base class' constructor
  using VolumetricDataStructure2D::VolumetricDataStructure2D;

  bool empty() const override { return !data_.size(); }
  size_t size() const override { return data_.size(); }
  void prune() override;
  void clear() override;

  size_t getMemoryUsage() const override {
    return size() * sizeof(CellDataSpecialized);
  }

  Index2D dimensions() const { return {data_.rows(), data_.cols()}; }
  Index2D getMinIndex() const override { return min_external_index_; }
  Index2D getMaxIndex() const override { return max_external_index_; }

  bool hasCell(const Index2D& index) const;
  FloatingPoint getCellValue(const Index2D& index) const override;
  void setCellValue(const Index2D& index, FloatingPoint new_value) override;
  void addToCellValue(const Index2D& index, FloatingPoint update) override;

  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const override;

  DataGridSpecialized& getData() { return data_; }
  const DataGridSpecialized& getData() const { return data_; }

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

 private:
  Index2D min_external_index_ = Index2D::Zero();
  Index2D max_external_index_ = Index2D::Zero();
  DataGridSpecialized data_;

  Index2D getMinInternalIndex() const { return Index2D::Zero(); }
  Index2D getMaxInternalIndex() const {
    return {data_.rows() - 1, data_.cols() - 1};
  }

  // TODO(victorr): Add check for overflows
  Index2D toInternal(const Index2D& index) const {
    return index - min_external_index_;
  }
  Index2D toExternal(const Index2D& index) const {
    return index + min_external_index_;
  }

  CellDataSpecialized* accessCellData(const Index2D& index,
                                      bool auto_allocate = false);
  const CellDataSpecialized* accessCellData(const Index2D& index) const;

  static std::string getHeaderFilePath(const std::string& file_path_prefix) {
    return file_path_prefix + "_header";
  }
  static std::string getDataFilePath(const std::string& file_path_prefix,
                                     const bool use_floating_precision) {
    return file_path_prefix + "_data." +
           (use_floating_precision ? "tiff" : "jp2");
  }
};
}  // namespace wavemap

#include "wavemap_2d/data_structure/impl/dense_grid_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_DENSE_GRID_H_

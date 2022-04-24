#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DENSE_GRID_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DENSE_GRID_H_

#include <string>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/volumetric/volumetric_data_structure.h"
#include "wavemap_2d/indexing/index.h"

namespace wavemap_2d {
template <typename CellT>
class DenseGrid : public VolumetricDataStructure {
 public:
  using CellType = CellT;
  using CellDataSpecialized = typename CellT::Specialized;
  using CellDataBaseFloat = typename CellT::BaseFloat;
  using CellDataBaseInt = typename CellT::BaseInt;

  template <typename T>
  using DataGrid = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
  using DataGridSpecialized = DataGrid<CellDataSpecialized>;
  using DataGridBaseFloat = DataGrid<CellDataBaseFloat>;
  using DataGridBaseInt = DataGrid<CellDataBaseInt>;

  explicit DenseGrid(const FloatingPoint resolution)
      : VolumetricDataStructure(resolution),
        min_index_(Index::Zero()),
        max_index_(Index::Zero()) {}

  bool empty() const override { return !data_.size(); }
  size_t size() const override { return data_.size(); }
  void clear() override;

  size_t getMemoryUsage() const override {
    return size() * sizeof(CellDataSpecialized);
  }

  Index dimensions() const { return {data_.rows(), data_.cols()}; }
  Index getMinIndex() const override { return min_index_; }
  Index getMaxIndex() const override { return max_index_; }

  bool hasCell(const Index& index) const override;
  FloatingPoint getCellValue(const Index& index) const override;
  void setCellValue(const Index& index, FloatingPoint new_value) override;
  void addToCellValue(const Index& index, FloatingPoint update) override;
  DataGridSpecialized& getData() { return data_; }
  const DataGridSpecialized& getData() const { return data_; }

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

 private:
  Index min_index_;
  Index max_index_;
  DataGridSpecialized data_;

  CellDataSpecialized* accessCellData(const Index& index,
                                      bool auto_allocate = false);
  const CellDataSpecialized* accessCellData(const Index& index) const;
};
}  // namespace wavemap_2d

#include "wavemap_2d/data_structure/volumetric/impl/dense_grid_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DENSE_GRID_H_

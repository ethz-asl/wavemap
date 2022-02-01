#ifndef WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_DENSE_GRID_H_
#define WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_DENSE_GRID_H_

#include <string>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/datastructure_base.h"

namespace wavemap_2d {
template <typename CellTypeT>
class DenseGrid : public DataStructureBase {
 public:
  using CellType = CellTypeT;

  explicit DenseGrid(const FloatingPoint resolution)
      : DataStructureBase(resolution),
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

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

 protected:
  using CellDataSpecialized = typename CellTypeT::Specialized;
  using CellDataBaseFloat = typename CellTypeT::BaseFloat;
  using CellDataBaseInt = typename CellTypeT::BaseInt;

  template <typename T>
  using DataGrid = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
  using DataGridSpecialized = DataGrid<CellDataSpecialized>;
  using DataGridBaseFloat = DataGrid<CellDataBaseFloat>;
  using DataGridBaseInt = DataGrid<CellDataBaseInt>;

  Index min_index_;
  Index max_index_;

  CellDataSpecialized* accessCellData(const Index& index,
                                      bool auto_allocate = false);
  const CellDataSpecialized* accessCellData(const Index& index) const;

  DataGridSpecialized data_;
};
}  // namespace wavemap_2d

#include "wavemap_2d/datastructure/dense_grid/dense_grid_inl.h"

#endif  // WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_DENSE_GRID_H_

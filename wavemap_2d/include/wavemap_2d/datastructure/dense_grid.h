#ifndef WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_H_
#define WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_H_

#include <string>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/datastructure_base.h"

namespace wavemap_2d {
template <typename CellDataType>
class DenseGrid : public DataStructureBase {
 public:
  using CellType = CellDataType;

  explicit DenseGrid(const FloatingPoint resolution)
      : DataStructureBase(resolution),
        min_index_(Index::Zero()),
        max_index_(Index::Zero()) {}

  bool empty() const override { return !data_.size(); }
  Index size() const override { return {data_.rows(), data_.cols()}; }
  void clear() override {
    data_.resize(0, 0);
    min_index_ = Index::Zero();
    max_index_ = Index::Zero();
  }

  Index getMinIndex() const override { return min_index_; }
  Index getMaxIndex() const override { return max_index_; }
  bool containsIndex(const Index& index) const override {
    return (min_index_.array() <= index.array() &&
            index.array() <= max_index_.array())
        .all();
  }

  void updateCell(const Index& index, const FloatingPoint update) override;
  FloatingPoint getCellValue(const Index& index) const override;

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix) const override;
  bool load(const std::string& file_path_prefix) override;

 protected:
  Index min_index_;
  Index max_index_;

  CellDataType& accessCellData(const Index& index) {
    // TODO(victorr): Add check for overflows
    const Index data_index = index - min_index_;
    return data_.coeffRef(data_index.x(), data_index.y());
  }
  const CellDataType& accessCellData(const Index& index) const {
    // TODO(victorr): Add check for overflows
    const Index data_index = index - min_index_;
    return data_.coeff(data_index.x(), data_index.y());
  }
  using GridDataStructure =
      Eigen::Matrix<CellDataType, Eigen::Dynamic, Eigen::Dynamic>;
  GridDataStructure data_;

  std::string getDataFilePathFromPrefix(
      const std::string& file_path_prefix) const override {
    const std::string extension =
        std::is_floating_point<CellDataType>::value ? "exr" : "jp2";
    return file_path_prefix + "_data." + extension;
  }
};
}  // namespace wavemap_2d

#include "wavemap_2d/datastructure/dense_grid_inl.h"

#endif  // WAVEMAP_2D_DATASTRUCTURE_DENSE_GRID_H_

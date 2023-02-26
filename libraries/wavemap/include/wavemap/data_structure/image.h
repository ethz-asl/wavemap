#ifndef WAVEMAP_DATA_STRUCTURE_IMAGE_H_
#define WAVEMAP_DATA_STRUCTURE_IMAGE_H_

#include "wavemap/common.h"
#include "wavemap/data_structure/posed_object.h"
#include "wavemap/utils/fill_utils.h"

namespace wavemap {
template <typename DataT = FloatingPoint>
class Image {
 public:
  using Data = Eigen::Matrix<DataT, Eigen::Dynamic, Eigen::Dynamic>;

  explicit Image(const Index2D& dimensions,
                 DataT initial_value = fill::zero<DataT>())
      : Image(dimensions.x(), dimensions.y(), initial_value) {}
  Image(IndexElement num_rows, IndexElement num_columns,
        DataT initial_value = fill::zero<DataT>())
      : initial_value_(initial_value),
        data_(Data::Constant(num_rows, num_columns, initial_value)) {}

  bool empty() const { return !size(); }
  size_t size() const { return data_.size(); }
  void resize(IndexElement num_rows, IndexElement num_columns) {
    data_.resize(num_rows, num_columns);
  }
  void clear() { resize(0, 0); }

  void setToConstant(DataT value) { data_.setConstant(value); }
  void resetToInitialValue() { setToConstant(initial_value_); }

  IndexElement getNumRows() const {
    return static_cast<IndexElement>(data_.rows());
  }
  IndexElement getNumColumns() const {
    return static_cast<IndexElement>(data_.cols());
  }
  Index2D getDimensions() const { return {getNumRows(), getNumColumns()}; }
  Data& getData() { return data_; }
  const Data& getData() const { return data_; }

  bool isIndexWithinBounds(const Index2D& index) const {
    return (0 <= index.array() && index.array() < getDimensions().array())
        .all();
  }

  DataT& at(Index2D index) {
    DCHECK((0 <= index.array()).all());
    DCHECK_LT(index.x(), data_.rows());
    DCHECK_LT(index.y(), data_.cols());
    return data_(index.x(), index.y());
  }
  const DataT& at(Index2D index) const {
    DCHECK((0 <= index.array()).all());
    DCHECK_LT(index.x(), data_.rows());
    DCHECK_LT(index.y(), data_.cols());
    return data_(index.x(), index.y());
  }

 private:
  DataT initial_value_;
  Data data_;
};

template <typename DataT = FloatingPoint>
using PosedImage = PosedObject<Image<DataT>>;
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_IMAGE_H_

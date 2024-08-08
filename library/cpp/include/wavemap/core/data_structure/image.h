#ifndef WAVEMAP_CORE_DATA_STRUCTURE_IMAGE_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_IMAGE_H_

#include <algorithm>
#include <memory>

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/posed_object.h"
#include "wavemap/core/utils/data/fill.h"

namespace wavemap {
template <typename PixelT = FloatingPoint>
class Image {
 public:
  using Ptr = std::shared_ptr<Image<PixelT>>;
  using ConstPtr = std::shared_ptr<const Image<PixelT>>;
  using Data = Eigen::Matrix<PixelT, Eigen::Dynamic, Eigen::Dynamic>;

  explicit Image(const Index2D& dimensions,
                 PixelT initial_value = data::fill::zero<PixelT>())
      : Image(dimensions.x(), dimensions.y(), initial_value) {}
  Image(IndexElement num_rows, IndexElement num_columns,
        PixelT initial_value = data::fill::zero<PixelT>())
      : initial_value_(initial_value),
        data_(Data::Constant(num_rows, num_columns, initial_value)) {}
  explicit Image(const Data& data,
                 PixelT initial_value = data::fill::zero<PixelT>())
      : initial_value_(initial_value), data_(data) {}

  bool empty() const { return !size(); }
  size_t size() const { return data_.size(); }
  void resize(IndexElement num_rows, IndexElement num_columns) {
    data_.resize(num_rows, num_columns);
  }
  void clear() { resize(0, 0); }

  void setToConstant(PixelT value) { data_.setConstant(value); }
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

  PixelT& at(const Index2D& index) {
    DCHECK((0 <= index.array()).all());
    DCHECK_LT(index.x(), data_.rows());
    DCHECK_LT(index.y(), data_.cols());
    return data_(index.x(), index.y());
  }
  const PixelT& at(const Index2D& index) const {
    DCHECK((0 <= index.array()).all());
    DCHECK_LT(index.x(), data_.rows());
    DCHECK_LT(index.y(), data_.cols());
    return data_(index.x(), index.y());
  }

  template <typename CwiseFunctor,
            typename R = std::invoke_result_t<CwiseFunctor, PixelT>>
  Image<R> transform(CwiseFunctor functor) const {
    return Image<R>(typename Image<R>::Data(data_.array().unaryExpr(functor)),
                    initial_value_);
  }

 private:
  PixelT initial_value_;
  Data data_;
};

template <typename PixelT = FloatingPoint>
using PosedImage = PosedObject<Image<PixelT>>;
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_IMAGE_H_

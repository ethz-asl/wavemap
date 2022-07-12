#ifndef WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_H_
#define WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_H_

#include <algorithm>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <wavemap_common/common.h>
#include <wavemap_common/indexing/ndtree_index.h>

namespace wavemap {
class VolumetricDataStructure {
 public:
  using Ptr = std::shared_ptr<VolumetricDataStructure>;

  explicit VolumetricDataStructure(const FloatingPoint min_cell_width)
      : min_cell_width_(min_cell_width) {}
  virtual ~VolumetricDataStructure() = default;

  virtual bool empty() const = 0;
  virtual size_t size() const = 0;
  virtual void prune() = 0;
  virtual void clear() = 0;

  FloatingPoint getMinCellWidth() const { return min_cell_width_; }
  virtual size_t getMemoryUsage() const = 0;

  virtual Index3D getMinIndex() const = 0;
  virtual Index3D getMaxIndex() const = 0;

  virtual FloatingPoint getCellValue(const Index3D& index) const = 0;
  virtual void setCellValue(const Index3D& index, FloatingPoint new_value) = 0;
  virtual void addToCellValue(const Index3D& index, FloatingPoint update) = 0;

  using IndexedLeafVisitorFunction =
      std::function<void(const OctreeIndex& index, FloatingPoint value)>;
  virtual void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const = 0;

  void printSize() const { LOG(INFO) << "Size:\n" << size(); }

  virtual bool save(const std::string& file_path_prefix,
                    bool use_floating_precision) const = 0;
  // TODO(victorr): Automatically determine whether floating or fixed precision
  //                was used from the file format once it has been designed
  virtual bool load(const std::string& file_path_prefix,
                    bool used_floating_precision) = 0;

 protected:
  const FloatingPoint min_cell_width_;
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_H_

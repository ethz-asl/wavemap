#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_BASE_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_BASE_H_

#include <algorithm>
#include <memory>
#include <string>

#include "wavemap_common/common.h"
#include "wavemap_common/data_structure/volumetric/volumetric_data_structure_config.h"
#include "wavemap_common/indexing/ndtree_index.h"

namespace wavemap {
template <int dim>
class VolumetricDataStructureBase {
 public:
  static constexpr int kDim = dim;
  using Ptr = std::shared_ptr<VolumetricDataStructureBase>;

  explicit VolumetricDataStructureBase(VolumetricDataStructureConfig config)
      : config_(config) {}
  virtual ~VolumetricDataStructureBase() = default;

  virtual bool empty() const = 0;
  virtual size_t size() const = 0;
  virtual void prune() = 0;
  virtual void clear() = 0;

  virtual const VolumetricDataStructureConfig& getConfig() const {
    return config_;
  }
  FloatingPoint getMinCellWidth() const { return config_.min_cell_width; }
  virtual size_t getMemoryUsage() const = 0;

  virtual Index<dim> getMinIndex() const = 0;
  virtual Index<dim> getMaxIndex() const = 0;

  virtual FloatingPoint getCellValue(const Index<dim>& index) const = 0;
  virtual void setCellValue(const Index<dim>& index,
                            FloatingPoint new_value) = 0;
  virtual void addToCellValue(const Index<dim>& index,
                              FloatingPoint update) = 0;

  using IndexedLeafVisitorFunction =
      std::function<void(const NdtreeIndex<dim>& index, FloatingPoint value)>;
  virtual void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const = 0;

  virtual bool save(const std::string& file_path_prefix,
                    bool use_floating_precision) const = 0;
  // TODO(victorr): Automatically determine whether floating or fixed precision
  //                was used from the file format once it has been designed
  virtual bool load(const std::string& file_path_prefix,
                    bool used_floating_precision) = 0;

 protected:
  const VolumetricDataStructureConfig config_;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_BASE_H_

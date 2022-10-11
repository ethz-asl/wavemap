#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_BASE_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_BASE_H_

#include <algorithm>
#include <memory>
#include <string>

#include "wavemap_common/common.h"
#include "wavemap_common/indexing/ndtree_index.h"
#include "wavemap_common/utils/config_utils.h"

namespace wavemap {
template <int dim>
class VolumetricDataStructureBase {
 public:
  static constexpr int kDim = dim;
  using Ptr = std::shared_ptr<VolumetricDataStructureBase>;

  struct Config {
    FloatingPoint min_cell_width = 0.1f;

    static Config fromParams(const param::Map& params) {
      Config config;
      for (const auto& [param_name, param_value] : params) {
        if (param_name == "min_cell_width") {
          config.min_cell_width = param::convert::toMeters(param_value);
        } else {
          LOG(WARNING) << "Ignoring unknown param with name " << param_name;
        }
      }
      return config;
    }
  };

  explicit VolumetricDataStructureBase(Config config) : config_(config) {}
  virtual ~VolumetricDataStructureBase() = default;

  virtual bool empty() const = 0;
  virtual size_t size() const = 0;
  virtual void prune() = 0;
  virtual void clear() = 0;

  virtual const Config& getConfig() const { return config_; }
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
  const Config config_;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_BASE_H_

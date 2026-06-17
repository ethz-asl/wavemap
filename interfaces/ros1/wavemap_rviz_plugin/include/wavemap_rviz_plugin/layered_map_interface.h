#ifndef WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_INTERFACE_H_
#define WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <OGRE/OgreColourValue.h>
#include <wavemap/core/common.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>

#include "wavemap_rviz_plugin/occupancy_query.h"

namespace wavemap::rviz_plugin {
struct LayerMetadata {
  std::string name;
  std::string type;
};

// Runtime interface used by the RViz plugin for layered maps. Concrete user
// voxel types remain hidden behind this API, keeping the display code modular.
class LayeredMapInterface : public OccupancyQuery {
 public:
  using IndexedOccupancyVisitor =
      std::function<void(const OctreeIndex&, FloatingPoint)>;

  ~LayeredMapInterface() override = default;

  virtual IndexElement getTreeHeight() const = 0;
  virtual FloatingPoint getMinCellWidth() const = 0;
  virtual const std::vector<LayerMetadata>& getLayers() const = 0;

  virtual std::vector<Index3D> getBlockIndices() const = 0;
  virtual bool hasBlock(const Index3D& block_index) const = 0;
  virtual void forEachLeaf(const IndexedOccupancyVisitor& visitor_fn,
                           IndexElement termination_height = 0) const = 0;
  virtual void forEachBlockLeaf(const Index3D& block_index,
                                const IndexedOccupancyVisitor& visitor_fn,
                                IndexElement termination_height = 0) const = 0;
  virtual bool getLayerColor(const std::string& layer_name,
                             const OctreeIndex& index,
                             FloatingPoint occupancy,
                             Ogre::ColourValue& color) const = 0;
};

struct NoLayerColorProvider {
  template <typename CellDataT>
  static bool getLayerColor(const std::string& /*layer_name*/,
                            const CellDataT& /*voxel*/,
                            FloatingPoint /*occupancy*/,
                            Ogre::ColourValue& /*color*/) {
    return false;
  }
};

template <typename LayeredMapT, typename LayerColorProviderT = NoLayerColorProvider>
class HashedWaveletOctreeLayeredMapAdapter : public LayeredMapInterface {
 public:
  HashedWaveletOctreeLayeredMapAdapter(
      std::shared_ptr<const LayeredMapT> map,
      std::vector<LayerMetadata> layers)
      : map_(std::move(map)), layers_(std::move(layers)) {}

  IndexElement getTreeHeight() const override { return map_->getTreeHeight(); }

  FloatingPoint getMinCellWidth() const override {
    return map_->getMinCellWidth();
  }

  const std::vector<LayerMetadata>& getLayers() const override {
    return layers_;
  }

  FloatingPoint getCellValue(const OctreeIndex& index) override {
    return map_->getCellValue(index);
  }

  std::vector<Index3D> getBlockIndices() const override {
    std::vector<Index3D> block_indices;
    block_indices.reserve(map_->getHashMap().size());
    map_->forEachBlock([&block_indices](const Index3D& block_index,
                                        const auto& /*block*/) {
      block_indices.push_back(block_index);
    });
    return block_indices;
  }

  bool hasBlock(const Index3D& block_index) const override {
    return map_->hasBlock(block_index);
  }

  void forEachLeaf(const IndexedOccupancyVisitor& visitor_fn,
                   IndexElement termination_height = 0) const override {
    map_->forEachBlock([&visitor_fn, termination_height](
                           const Index3D& block_index, const auto& block) {
      block.forEachLeaf(block_index, visitor_fn, termination_height);
    });
  }

  void forEachBlockLeaf(const Index3D& block_index,
                        const IndexedOccupancyVisitor& visitor_fn,
                        IndexElement termination_height = 0) const override {
    if (const auto* block = map_->getBlock(block_index); block) {
      block->forEachLeaf(block_index, visitor_fn, termination_height);
    }
  }

  bool getLayerColor(const std::string& layer_name, const OctreeIndex& index,
                     FloatingPoint occupancy,
                     Ogre::ColourValue& color) const override {
    const auto voxel = map_->getVoxelValue(index);
    return LayerColorProviderT::getLayerColor(layer_name, voxel, occupancy,
                                              color);
  }

 private:
  std::shared_ptr<const LayeredMapT> map_;
  std::vector<LayerMetadata> layers_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_LAYERED_MAP_INTERFACE_H_

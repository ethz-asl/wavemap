#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_WAVELET_OCTREE_INTERFACE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_WAVELET_OCTREE_INTERFACE_H_

#include <memory>

#include "wavemap/data_structure/ndtree/ndtree_node.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_transform.h"
#include "wavemap/data_structure/volumetric/volumetric_octree_interface.h"

namespace wavemap {
class WaveletOctreeInterface : public virtual VolumetricOctreeInterface {
 public:
  using Ptr = std::shared_ptr<WaveletOctreeInterface>;

  using Coefficients = HaarCoefficients<FloatingPoint, kDim>;
  using Transform = HaarTransform<FloatingPoint, kDim>;
  using NodeType = NdtreeNode<typename Coefficients::Details, kDim>;

  using VolumetricOctreeInterface::VolumetricOctreeInterface;

  virtual typename Coefficients::Scale& getRootScale() = 0;
  virtual const typename Coefficients::Scale& getRootScale() const = 0;
  virtual NodeType& getRootNode() = 0;
  virtual const NodeType& getRootNode() const = 0;
  virtual NodeType* getNode(const OctreeIndex& node_index) = 0;
  virtual const NodeType* getNode(const OctreeIndex& node_index) const = 0;
};
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_WAVELET_OCTREE_INTERFACE_H_

#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_WAVELET_NDTREE_INTERFACE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_WAVELET_NDTREE_INTERFACE_H_

#include <memory>

#include "wavemap/data_structure/ndtree/ndtree_node.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_transform.h"
#include "wavemap/data_structure/volumetric/volumetric_ndtree_interface.h"

namespace wavemap {
template <int dim>
class WaveletNdtreeInterface : public virtual VolumetricNdtreeInterface<dim> {
 public:
  using Ptr = std::shared_ptr<WaveletNdtreeInterface>;

  using Coefficients = HaarCoefficients<FloatingPoint, dim>;
  using Transform = HaarTransform<FloatingPoint, dim>;
  using NodeType = NdtreeNode<typename Coefficients::Details, dim>;

  using VolumetricNdtreeInterface<dim>::VolumetricNdtreeInterface;

  virtual typename Coefficients::Scale& getRootScale() = 0;
  virtual const typename Coefficients::Scale& getRootScale() const = 0;
  virtual NodeType& getRootNode() = 0;
  virtual const NodeType& getRootNode() const = 0;
  virtual NodeType* getNode(const NdtreeIndex<dim>& node_index) = 0;
  virtual const NodeType* getNode(const NdtreeIndex<dim>& node_index) const = 0;
};
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_WAVELET_NDTREE_INTERFACE_H_

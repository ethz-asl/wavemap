#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_WAVELET_TREE_INTERFACE_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_WAVELET_TREE_INTERFACE_H_

#include <memory>

#include "wavemap_common/data_structure/ndtree/ndtree_node.h"
#include "wavemap_common/data_structure/volumetric/cell_types/haar_transform.h"
#include "wavemap_common/data_structure/volumetric/volumetric_ndtree_interface.h"

namespace wavemap {
template <int dim>
class WaveletTreeInterface : public virtual VolumetricNdtreeInterface<dim> {
 public:
  using Ptr = std::shared_ptr<WaveletTreeInterface>;

  using Transform = HaarTransform<FloatingPoint, dim>;
  using ScaleCoefficient = typename HaarCoefficients<FloatingPoint, dim>::Scale;
  using DetailCoefficients =
      typename HaarCoefficients<FloatingPoint, dim>::Details;
  using ParentCoefficients =
      typename HaarCoefficients<FloatingPoint, dim>::Parent;
  using ChildScaleCoefficients =
      typename HaarCoefficients<FloatingPoint, dim>::CoefficientsArray;
  using NodeType = NdtreeNode<DetailCoefficients, 2>;

  using VolumetricNdtreeInterface<dim>::VolumetricNdtreeInterface;

  virtual ScaleCoefficient& getRootScale() = 0;
  virtual const ScaleCoefficient& getRootScale() const = 0;
  virtual NodeType& getRootNode() = 0;
  virtual const NodeType& getRootNode() const = 0;
  virtual NodeType* getNode(const NdtreeIndex<dim>& node_index) = 0;
  virtual const NodeType* getNode(const NdtreeIndex<dim>& node_index) const = 0;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_WAVELET_TREE_INTERFACE_H_

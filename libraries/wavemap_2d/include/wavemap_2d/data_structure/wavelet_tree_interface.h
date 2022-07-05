#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_WAVELET_TREE_INTERFACE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_WAVELET_TREE_INTERFACE_H_

#include <wavemap_common/data_structure/ndtree/ndtree_node.h>

#include "wavemap_2d/data_structure/cell_types/haar_wavelet.h"
#include "wavemap_2d/data_structure/volumetric_quadtree_interface.h"

namespace wavemap {
class WaveletTreeInterface : public VolumetricQuadtreeInterface {
 public:
  using HaarWaveletType = HaarWavelet<FloatingPoint>;
  using ScaleCoefficient = typename HaarWaveletType::Coefficients::Scale;
  using DetailCoefficients = typename HaarWaveletType::Coefficients::Details;
  using ChildScaleCoefficients =
      typename HaarWaveletType::ChildScaleCoefficients;
  using ParentCoefficients = typename HaarWaveletType::ParentCoefficients;
  using NodeType = NdtreeNode<DetailCoefficients, 2>;

  using VolumetricQuadtreeInterface::VolumetricQuadtreeInterface;

  virtual ScaleCoefficient& getRootScale() = 0;
  virtual const ScaleCoefficient& getRootScale() const = 0;
  virtual NodeType& getRootNode() = 0;
  virtual const NodeType& getRootNode() const = 0;
  virtual NodeType* getNode(const QuadtreeIndex& node_index) = 0;
  virtual const NodeType* getNode(const QuadtreeIndex& node_index) const = 0;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_WAVELET_TREE_INTERFACE_H_

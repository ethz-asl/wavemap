#ifndef WAVEMAP_2D_INDEXING_INDEX_H_
#define WAVEMAP_2D_INDEXING_INDEX_H_

#include "wavemap_2d/common.h"

namespace wavemap_2d {
using IndexElement = int;
using Index = Eigen::Matrix<IndexElement, 2, 1>;

inline Index computeNearestIndexForPoint(const Point& point,
                                         FloatingPoint resolution_inv) {
  return (point * resolution_inv).array().round().cast<IndexElement>();
}
inline Index computeNearestIndexForScaledPoint(const Point& point) {
  return point.array().round().cast<IndexElement>();
}

inline Index computeFloorIndexForPoint(const Point& point,
                                       FloatingPoint resolution_inv) {
  return (point * resolution_inv).array().floor().cast<IndexElement>();
}
inline Index computeCeilIndexForPoint(const Point& point,
                                      FloatingPoint resolution_inv) {
  return (point * resolution_inv).array().ceil().cast<IndexElement>();
}

inline Point computeCenterFromIndex(const Index& index,
                                    FloatingPoint resolution) {
  return index.cast<FloatingPoint>() * resolution;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INDEXING_INDEX_H_

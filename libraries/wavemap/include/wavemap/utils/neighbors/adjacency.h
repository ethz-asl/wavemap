#ifndef WAVEMAP_UTILS_NEIGHBORS_ADJACENCY_H_
#define WAVEMAP_UTILS_NEIGHBORS_ADJACENCY_H_

#include "wavemap/common.h"

namespace wavemap {
enum class AdjacencyType : int { kVertex = 0, kEdge, kFace, kCube };

using AdjacencyMask = uint8_t;

constexpr AdjacencyMask kAdjacencyNone = AdjacencyMask{0};

template <int dim>
constexpr AdjacencyMask kAdjacencyAnyDisjoint =
    static_cast<AdjacencyMask>((1 << dim) - 1);

constexpr AdjacencyMask kAdjacencyAny = static_cast<AdjacencyMask>(-1);
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_NEIGHBORS_ADJACENCY_H_

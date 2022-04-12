#include "wavemap_2d/indexing/quadtree_index.h"

#include <string>

namespace wavemap_2d {
std::string QuadtreeIndex::toString() const {
  std::stringstream ss;
  ss << "[depth=";
  ss << depth << ", position=[";
  for (int i = 0; i < kMapDimension; ++i) {
    if (i) {
      ss << ", ";
    }
    ss << position[i];
  }
  ss << "]]";
  return ss.str();
}

}  // namespace wavemap_2d

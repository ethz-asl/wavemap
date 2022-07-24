#ifndef WAVEMAP_2D_UTILS_CONTAINER_PRINT_UTILS_H_
#define WAVEMAP_2D_UTILS_CONTAINER_PRINT_UTILS_H_

#include <string>
#include <utility>

namespace wavemap_2d {
template <typename SequenceContainerT>
inline std::string ToString(const SequenceContainerT& container) {
  return std::accumulate(std::next(container.cbegin()), container.cend(),
                         std::to_string(container[0]),
                         [](auto str, const auto& el) -> std::string {
                           return std::move(str) + ", " + std::to_string(el);
                         });
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_UTILS_CONTAINER_PRINT_UTILS_H_

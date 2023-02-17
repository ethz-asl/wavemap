#ifndef WAVEMAP_UTILS_CONTAINER_PRINT_UTILS_H_
#define WAVEMAP_UTILS_CONTAINER_PRINT_UTILS_H_

#include <numeric>
#include <string>
#include <utility>

namespace wavemap {
template <typename SequenceContainerT>
inline std::string ToString(const SequenceContainerT& container) {
  return std::accumulate(std::next(container.cbegin()), container.cend(),
                         std::to_string(container[0]),
                         [](auto str, const auto& el) -> std::string {
                           return std::move(str) + ", " + std::to_string(el);
                         });
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_CONTAINER_PRINT_UTILS_H_

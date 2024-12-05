#ifndef WAVEMAP_CORE_UTILS_PRINT_CONTAINER_H_
#define WAVEMAP_CORE_UTILS_PRINT_CONTAINER_H_

#include <numeric>
#include <string>
#include <utility>

namespace wavemap::print {
template <typename ElementT>
inline auto element(const ElementT& element)
    -> std::enable_if_t<std::is_arithmetic_v<ElementT>, std::string> {
  return std::to_string(element);
}

template <typename ElementT>
inline auto element(const ElementT& element)
    -> std::enable_if_t<!std::is_arithmetic_v<ElementT>, std::string> {
  return std::string(element);
}

template <typename SequenceT>
inline std::string sequence(const SequenceT& sequence,
                            const std::string& separator = ", ") {
  return std::accumulate(std::next(sequence.cbegin()), sequence.cend(),
                         print::element(sequence[0]),
                         [separator](auto str, const auto& el) -> std::string {
                           return str + separator + print::element(el);
                         });
}
}  // namespace wavemap::print

#endif  // WAVEMAP_CORE_UTILS_PRINT_CONTAINER_H_

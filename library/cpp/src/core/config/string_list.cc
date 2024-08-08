#include "wavemap/core/config/string_list.h"

#include "wavemap/core/utils/print/container.h"

namespace wavemap {
std::optional<StringList> StringList::from(const param::Value& param) {
  ValueType string_list;

  if (const auto single_string = param.as<std::string>(); single_string) {
    string_list.emplace_back(single_string.value());
  } else if (const auto array = param.as<param::Array>(); array) {
    size_t idx = 0u;
    for (const auto& element : array.value()) {
      if (const auto element_string = element.as<std::string>();
          element_string) {
        string_list.emplace_back(element_string.value());
      } else {
        LOG(WARNING) << "Skipping element at index " << idx
                     << ". Could not be parsed as string.";
      }
      ++idx;
    }
  } else {
    LOG(ERROR) << "Param should be a string or list of strings.";
    return std::nullopt;
  }

  return string_list;
}

std::string StringList::toStr() const {
  return "[" + print::sequence(value) + "]";
}
}  // namespace wavemap

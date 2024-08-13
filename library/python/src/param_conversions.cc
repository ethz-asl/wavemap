#include "pywavemap/param_conversions.h"

namespace wavemap::convert {
param::Map toParamMap(const nb::handle& py_value) {  // NOLINT
  nb::dict py_dict;
  if (!nb::try_cast(py_value, py_dict)) {
    LOG(WARNING) << "Expected python dict, but got "
                 << nb::repr(py_value).c_str() << ".";
    return {};
  }

  param::Map param_map;
  for (const auto& [py_key, py_dict_value] : py_dict) {
    nb::str py_key_str;
    if (nb::try_cast(py_key, py_key_str)) {
      param_map.emplace(py_key_str.c_str(), toParamValue(py_dict_value));
    } else {
      LOG(WARNING) << "Ignoring dict entry. Key not convertible to string for "
                      "element with key "
                   << nb::repr(py_key).c_str() << " and value "
                   << nb::repr(py_dict_value).c_str() << ".";
    }
  }
  return param_map;
}

param::Array toParamArray(const nb::handle& py_value) {  // NOLINT
  nb::list py_list;
  if (!nb::try_cast(py_value, py_list)) {
    LOG(WARNING) << "Expected python list, but got "
                 << nb::repr(py_value).c_str() << ".";
    return {};
  }

  param::Array array;
  array.reserve(nb::len(py_list));
  for (const auto& py_element : py_list) {  // NOLINT
    array.template emplace_back(toParamValue(py_element));
  }
  return array;
}

param::Value toParamValue(const nb::handle& py_value) {  // NOLINT
  if (nb::bool_ py_bool; nb::try_cast(py_value, py_bool)) {
    return param::Value{static_cast<bool>(py_bool)};
  }
  if (nb::int_ py_int; nb::try_cast(py_value, py_int)) {
    return param::Value{static_cast<int>(py_int)};
  }
  if (nb::float_ py_float; nb::try_cast(py_value, py_float)) {
    return param::Value{static_cast<double>(py_float)};
  }
  if (nb::str py_str; nb::try_cast(py_value, py_str)) {
    return param::Value(py_str.c_str());
  }
  if (nb::isinstance<nb::list>(py_value)) {
    return param::Value(toParamArray(py_value));
  }
  if (nb::isinstance<nb::dict>(py_value)) {
    return param::Value(toParamMap(py_value));
  }

  // On error, return an empty array
  LOG(ERROR) << "Encountered unsupported type while parsing python param "
             << nb::repr(py_value).c_str() << ".";
  return param::Value{param::Array{}};
}
}  // namespace wavemap::convert

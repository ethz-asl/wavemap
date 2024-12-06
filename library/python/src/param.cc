#include "pywavemap/param.h"

#include <string>

namespace wavemap {
namespace convert {
param::Map pyToParamMap(const nb::handle& py_value) {  // NOLINT
  nb::dict py_dict;
  if (!nb::try_cast(py_value, py_dict)) {
    LOG(WARNING) << "Expected python dict, but got "
                 << nb::repr(py_value).c_str() << ".";
    return {};
  }

  param::Map param_map;
  for (const auto& [py_key, py_dict_value] : py_dict) {
    if (nb::str py_key_str; nb::try_cast(py_key, py_key_str)) {
      param_map.emplace(py_key_str.c_str(), pyToParams(py_dict_value));
    } else {
      LOG(WARNING) << "Ignoring dict entry. Key not convertible to string for "
                      "element with key "
                   << nb::repr(py_key).c_str() << " and value "
                   << nb::repr(py_dict_value).c_str() << ".";
    }
  }
  return param_map;
}

param::Array pyToParamArray(const nb::handle& py_value) {  // NOLINT
  nb::list py_list;
  if (!nb::try_cast(py_value, py_list)) {
    LOG(WARNING) << "Expected python list, but got "
                 << nb::repr(py_value).c_str() << ".";
    return {};
  }

  param::Array array;
  array.reserve(nb::len(py_list));
  for (const auto& py_element : py_list) {
    array.emplace_back(pyToParams(py_element));
  }
  return array;
}

param::Value pyToParams(const nb::handle& py_value) {  // NOLINT
  if (nb::isinstance<nb::dict>(py_value)) {
    return param::Value(pyToParamMap(py_value));
  }
  if (nb::isinstance<nb::list>(py_value)) {
    return param::Value(pyToParamArray(py_value));
  }
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
    return param::Value{std::string{py_str.c_str()}};
  }

  // On error, return an empty array
  LOG(ERROR) << "Encountered unsupported type while parsing python param "
             << nb::repr(py_value).c_str() << ".";
  return param::Value{param::Array{}};
}
}  // namespace convert

void add_param_module(nb::module_& m_param) {
  nb::class_<param::Value>(
      m_param, "Value",
      "A class that holds parameter values. Note that one Value can hold a "
      "primitive type, a list of Values, or a dictionary of Values. One Value "
      "can therefore hold the information needed to initialize an entire "
      "config, or even a hierarchy of nested configs.")
      .def("__init__", [](param::Value* t, nb::handle py_value) {
        new (t) param::Value{convert::pyToParams(py_value)};
      });

  nb::implicitly_convertible<nb::handle, param::Value>();
}
}  // namespace wavemap

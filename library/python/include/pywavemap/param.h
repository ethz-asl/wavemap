#ifndef PYWAVEMAP_PARAM_H_
#define PYWAVEMAP_PARAM_H_

#include <nanobind/nanobind.h>
#include <wavemap/core/config/param.h>

namespace nb = nanobind;

namespace wavemap {
namespace convert {
param::Map toParamMap(const nb::handle& py_value);
param::Array toParamArray(const nb::handle& py_value);
param::Value toParamValue(const nb::handle& py_value);
}  // namespace convert

void add_param_module(nb::module_& m_param);
}  // namespace wavemap

#endif  // PYWAVEMAP_PARAM_H_

#ifndef PYWAVEMAP_PARAM_CONVERSIONS_H_
#define PYWAVEMAP_PARAM_CONVERSIONS_H_

#include <nanobind/nanobind.h>
#include <wavemap/core/config/param.h>

namespace nb = nanobind;

namespace wavemap::convert {
param::Map toParamMap(const nb::handle& py_value);
param::Array toParamArray(const nb::handle& py_value);
param::Value toParamValue(const nb::handle& py_value);
}  // namespace wavemap::convert

#endif  // PYWAVEMAP_PARAM_CONVERSIONS_H_

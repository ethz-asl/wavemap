#ifndef PYWAVEMAP_CONVERT_H_
#define PYWAVEMAP_CONVERT_H_

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace wavemap {
void add_convert_module(nb::module_& m_convert);
}  // namespace wavemap

#endif  // PYWAVEMAP_CONVERT_H_

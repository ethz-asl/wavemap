#ifndef PYWAVEMAP_EDIT_H_
#define PYWAVEMAP_EDIT_H_

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace wavemap {
void add_edit_module(nb::module_& m_edit);
}  // namespace wavemap

#endif  // PYWAVEMAP_EDIT_H_

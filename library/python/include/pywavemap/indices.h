#ifndef PYWAVEMAP_INDICES_H_
#define PYWAVEMAP_INDICES_H_

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace wavemap {
void add_index_bindings(nb::module_& m);
}  // namespace wavemap

#endif  // PYWAVEMAP_INDICES_H_

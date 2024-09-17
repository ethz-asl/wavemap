#ifndef PYWAVEMAP_MAPS_H_
#define PYWAVEMAP_MAPS_H_

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace wavemap {
void add_map_bindings(nb::module_& m);
}  // namespace wavemap

#endif  // PYWAVEMAP_MAPS_H_

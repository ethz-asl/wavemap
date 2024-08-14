#ifndef PYWAVEMAP_MAP_H_
#define PYWAVEMAP_MAP_H_

#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>

namespace nb = nanobind;

namespace wavemap {
void add_map_bindings(nb::module_& m);
}  // namespace wavemap

#endif  // PYWAVEMAP_MAP_H_

#ifndef PYWAVEMAP_GEOMETRY_H_
#define PYWAVEMAP_GEOMETRY_H_

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace wavemap {
void add_geometry_bindings(nb::module_& m);
}

#endif  // PYWAVEMAP_GEOMETRY_H_

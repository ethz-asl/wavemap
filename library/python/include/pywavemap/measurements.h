#ifndef PYWAVEMAP_MEASUREMENTS_H_
#define PYWAVEMAP_MEASUREMENTS_H_

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace wavemap {
void add_measurement_bindings(nb::module_& m);
}

#endif  // PYWAVEMAP_MEASUREMENTS_H_

#include "pywavemap/map.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/map/map_factory.h>
#include <wavemap/io/file_conversions.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_map_bindings(nb::module_& m) {
  nb::class_<MapBase>(m, "Map")
      .def_prop_ro("empty", &MapBase::empty)
      .def_prop_ro("size", &MapBase::size)
      .def("threshold", &MapBase::threshold)
      .def("prune", &MapBase::prune)
      .def("pruneSmart", &MapBase::pruneSmart)
      .def("clear", &MapBase::clear)
      .def_prop_ro("min_cell_width", &MapBase::getMinCellWidth)
      .def_prop_ro("min_log_odds", &MapBase::getMinLogOdds)
      .def_prop_ro("max_log_odds", &MapBase::getMaxLogOdds)
      .def_prop_ro("memory_usage", &MapBase::getMemoryUsage)
      .def_prop_ro("tree_height", &MapBase::getTreeHeight)
      .def_prop_ro("min_index", &MapBase::getMinIndex)
      .def_prop_ro("max_index", &MapBase::getMaxIndex)
      .def("getCellValue", &MapBase::getCellValue)
      .def("setCellValue", &MapBase::setCellValue)
      .def("addToCellValue", &MapBase::addToCellValue)
      .def_static("create",
                  [](const param::Value& params) -> std::shared_ptr<MapBase> {
                    return MapFactory::create(params);
                  })
      .def_static(
          "load",
          [](const std::filesystem::path& file_path)
              -> std::shared_ptr<MapBase> {
            std::shared_ptr<MapBase> map;
            if (wavemap::io::fileToMap(file_path, map)) {
              return map;
            }
            return nullptr;
          },
          "file_path"_a, "Load a wavemap map from a .wvmp file.");
}
}  // namespace wavemap

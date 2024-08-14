#include "pywavemap/pipeline.h"

#include <nanobind/stl/shared_ptr.h>

namespace wavemap {
void add_pipeline_bindings(nb::module_& m) {
  nb::class_<Pipeline>(m, "Pipeline")
      .def(nb::init<std::shared_ptr<MapBase>>())
      .def("clear", &Pipeline::clear)
      .def("hasIntegrator", &Pipeline::hasIntegrator)
      .def("eraseIntegrator", &Pipeline::eraseIntegrator)
      .def("addIntegrator",
           [](Pipeline& self, const std::string& integrator_name,
              const param::Value& params) -> void {
             self.addIntegrator(integrator_name, params);
           })
      .def("clearIntegrators", &Pipeline::clearIntegrators)
      .def("addOperation",
           [](Pipeline& self, const param::Value& params) -> void {
             self.addOperation(params);
           })
      .def("clearOperations", &Pipeline::clearOperations)
      .def("runIntegrators", &Pipeline::runIntegrators<PosedPointcloud<>>)
      .def("runIntegrators", &Pipeline::runIntegrators<PosedImage<>>)
      .def("runOperations", &Pipeline::runOperations)
      .def("runPipeline", &Pipeline::runPipeline<PosedPointcloud<>>)
      .def("runPipeline", &Pipeline::runPipeline<PosedImage<>>);
}
}  // namespace wavemap

# Convenience function to generate stubs for a nanobind-based (sub)module
function(pywavemap_add_stub library_target python_module)
  string(REPLACE "." "_" module_stub_name "${python_module}_stub")
  string(REPLACE "." "/" module_subpath ${python_module})
  nanobind_add_stub(${module_stub_name}
      MODULE ${python_module}
      OUTPUT "${CMAKE_SOURCE_DIR}/src/${module_subpath}/__init__.pyi"
      PYTHON_PATH $<TARGET_FILE_DIR:${library_target}>
      DEPENDS ${library_target})
endfunction()

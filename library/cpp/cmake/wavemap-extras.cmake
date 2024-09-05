# User options
option(DCHECK_ALWAYS_ON
    "Enable GLOG DCHECKs even when not compiling in debug mode" OFF)
option(USE_UBSAN "Compile with undefined behavior sanitizer enabled" OFF)
option(USE_ASAN "Compile with address sanitizer enabled" OFF)
option(USE_TSAN "Compile with thread sanitizer enabled" OFF)
option(ENABLE_COVERAGE_TESTING
    "Compile with necessary flags for coverage testing" OFF)
option(USE_CLANG_TIDY "Generate necessary files to run clang-tidy" OFF)

# Adds the include paths of the wavemap library to the given target.
function(add_wavemap_include_directories target)
  # Configure the include dirs
  set(include_dirs ${PROJECT_SOURCE_DIR}/include/wavemap)
  get_filename_component(include_dirs ${include_dirs} PATH)
  target_include_directories(${target} PUBLIC
      $<BUILD_INTERFACE:${include_dirs}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
endfunction()

# Adds the default wavemap library compile features, definitions and options
# to the given target. We use it to ensure that all targets in the wavemap
# library are configured consistently. The function can also be called for
# targets that depend on wavemap (e.g. in a downstream package), to ensure
# their configuration is compatible.
function(set_wavemap_target_properties target)
  # Require C++17
  target_compile_features(${target} PUBLIC cxx_std_17)
  set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)

  # General compilation options
  set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_compile_options(${target} PUBLIC -march=native)
  target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic -Wno-deprecated-copy -Wno-class-memaccess)

  # General C++ defines
  target_compile_definitions(${target} PUBLIC EIGEN_INITIALIZE_MATRICES_BY_NAN)

  # Conditional compilation options
  if (DCHECK_ALWAYS_ON)
    target_compile_definitions(${target} PUBLIC DCHECK_ALWAYS_ON)
  endif ()
  if (USE_UBSAN)
    target_compile_options(${target} PUBLIC
        -fsanitize=undefined
        -fsanitize=shift
        -fsanitize=integer-divide-by-zero
        -fsanitize=null
        -fsanitize=return
        -fsanitize=signed-integer-overflow
        -fsanitize=bounds-strict
        -fsanitize=alignment
        -fsanitize=float-divide-by-zero
        -fsanitize=float-cast-overflow
        -fsanitize=enum
        -fsanitize=vptr
        -fsanitize=pointer-overflow
        -fsanitize=builtin
        -fno-omit-frame-pointer
        -g)
    target_link_options(${target} PUBLIC -fsanitize=undefined)
  endif ()
  if (USE_ASAN)
    target_compile_options(${target} PUBLIC
        -fsanitize=address -fsanitize-address-use-after-scope
        -fno-omit-frame-pointer -g)
    target_link_options(${target} PUBLIC -fsanitize=address)
  endif ()
  if (USE_TSAN)
    target_compile_options(${target} PUBLIC
        -fsanitize=thread -fno-omit-frame-pointer -g)
    target_link_options(${target} PUBLIC -fsanitize=thread)
  endif ()
  if (ENABLE_COVERAGE_TESTING)
    target_compile_options(${target} PUBLIC
        -fprofile-arcs -ftest-coverage -O0 -g)
    target_link_options(${target} PUBLIC -fprofile-arcs)
  endif ()
endfunction()

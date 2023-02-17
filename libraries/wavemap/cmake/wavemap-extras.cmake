# NOTE: For useful info about catkin and using CFG_EXTRAS:
# http://wiki.ros.org/catkin/CMakeLists.txt
# https://docs.ros.org/en/api/catkin/html/dev_guide/generated_cmake_api.html

# Add the compiler definitions and options consistently across all wavemap pkgs
macro (ADD_WAVEMAP_COMPILE_DEFINITIONS_AND_OPTIONS)
  option(USE_UBSAN "Compile with undefined behavior sanitizer enabled" OFF)
  option(USE_ASAN "Compile with address sanitizer enabled" OFF)
  option(USE_TSAN "Compile with thread sanitizer enabled" OFF)
  option(USE_CLANG_TIDY "Generate necessary files to run clang-tidy" OFF)
  option(ENABLE_BENCHMARKING "Compile benchmarking targets" OFF)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_compile_definitions(EIGEN_INITIALIZE_MATRICES_BY_NAN)
  add_compile_options(
    -march=native -Wall -Wextra -Wpedantic -Wsuggest-attribute=const
    -Wno-deprecated-copy -Wno-class-memaccess)

  if (USE_UBSAN)
    add_compile_options(
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
    add_link_options(-fsanitize=undefined)
  endif ()
  if (USE_ASAN)
    add_compile_options(-fsanitize=address -fsanitize-address-use-after-scope
                        -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=address)
  endif ()
  if (USE_TSAN)
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=thread)
  endif ()

  if (USE_CLANG_TIDY)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  endif ()

  if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    add_compile_options(-fprofile-arcs -ftest-coverage -O0 -g)
    add_link_options(-fprofile-arcs)
  endif ()
endmacro ()

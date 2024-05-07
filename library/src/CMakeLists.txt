# Dependencies
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

# Optional dependencies
find_package(tracy QUIET)
# TODO(victorr): Check if tracy is correctly found and linked in when it is
#                available as an installed (system) package.

add_subdirectory(core)
add_subdirectory(io)

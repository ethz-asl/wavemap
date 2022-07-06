cmake_minimum_required(VERSION 3.0.2)
project(wavemap_msgs)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Export
cs_install()
cs_export()

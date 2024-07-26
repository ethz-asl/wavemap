cmake_minimum_required(VERSION 3.0.2)
project(wavemap_examples_cpp)

# Load the library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../library
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME})

# Add each set of examples
add_subdirectory(io)
add_subdirectory(queries)
add_subdirectory(planning)

add_executable(benchmark_haar_transforms
    benchmark/benchmark_haar_transforms.cc)
target_link_libraries(benchmark_haar_transforms
    wavemap_core benchmark::benchmark)

add_executable(benchmark_sparse_vector benchmark/benchmark_sparse_vector.cc)
target_link_libraries(benchmark_sparse_vector
    wavemap_core benchmark::benchmark)

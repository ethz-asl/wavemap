add_executable(benchmark_haar_transforms
    benchmark/benchmark_haar_transforms.cc)
target_link_libraries(benchmark_haar_transforms ${PROJECT_NAME}
    benchmark::benchmark)

add_executable(benchmark_sparse_vector benchmark/benchmark_sparse_vector.cc)
target_link_libraries(benchmark_sparse_vector ${PROJECT_NAME}
    benchmark::benchmark)

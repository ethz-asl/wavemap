#ifndef WAVEMAP_EXAMPLES_COMMON_H_
#define WAVEMAP_EXAMPLES_COMMON_H_

namespace wavemap::examples {
/**
 * A placeholder method used to illustrate where the user would use a value.
 * Concretely, this method is also used to suppress 'unused variable' warnings
 * issued by GCC when compiling the usage examples.
 */
template <typename... T>
void doSomething([[maybe_unused]] T... t) {}
}  // namespace wavemap::examples

#endif  // WAVEMAP_EXAMPLES_COMMON_H_

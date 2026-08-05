#include <iostream>
#include <optional>
#include <string>

#include <wavemap/layered/discrete_layer.h>

using wavemap::layered::DiscreteCompressionConfig;
using wavemap::layered::DiscreteLayer;
using wavemap::layered::RawDiscreteLayer;

namespace {
template <typename LayerT, typename ValueT>
void fillBlock(LayerT& layer, const wavemap::Index3D& origin, int side_length,
               const ValueT& value) {
  for (int dx = 0; dx < side_length; ++dx) {
    for (int dy = 0; dy < side_length; ++dy) {
      for (int dz = 0; dz < side_length; ++dz) {
        layer.setValue(origin + wavemap::Index3D(dx, dy, dz), value);
      }
    }
  }
}

template <typename ValueT>
bool sameValue(const std::optional<ValueT>& read, const ValueT& expected) {
  return read && *read == expected;
}

template <typename ValueT>
size_t countPreservedValues(const RawDiscreteLayer<ValueT>& raw,
                            const DiscreteLayer<ValueT>& compressed) {
  size_t preserved = 0u;
  for (const auto& [key, expected] : raw.values()) {
    if (sameValue(compressed.getValue(indexFromKey(key)), expected)) {
      ++preserved;
    }
  }
  return preserved;
}

template <typename ValueT>
void printCompressionStats(const std::string& name,
                           const RawDiscreteLayer<ValueT>& raw,
                           const DiscreteLayer<ValueT>& compressed) {
  std::cout << name << "\n";
  std::cout << "  raw entries: " << raw.size() << "\n";
  std::cout << "  compressed parents: " << compressed.parentCount() << "\n";
  std::cout << "  exceptions: " << compressed.exceptionCount() << "\n";
  std::cout << "  preserved values: "
            << countPreservedValues(raw, compressed) << "/" << raw.size()
            << "\n";
}
}  // namespace

int main() {
  RawDiscreteLayer<int> semantic_layer;
  RawDiscreteLayer<bool> changed_layer;

  const wavemap::Index3D uniform_block(0, 0, 0);
  fillBlock(semantic_layer, uniform_block, 2, 1);
  fillBlock(changed_layer, uniform_block, 2, false);

  const wavemap::Index3D one_exception_block(4, 0, 0);
  fillBlock(semantic_layer, one_exception_block, 2, 2);
  fillBlock(changed_layer, one_exception_block, 2, false);
  semantic_layer.setValue(one_exception_block + wavemap::Index3D(1, 1, 1), 5);
  changed_layer.setValue(one_exception_block + wavemap::Index3D(1, 1, 1), true);

  const wavemap::Index3D mixed_block(8, 0, 0);
  fillBlock(semantic_layer, mixed_block, 2, 3);
  fillBlock(changed_layer, mixed_block, 2, false);
  semantic_layer.setValue(mixed_block + wavemap::Index3D(0, 1, 0), 4);
  semantic_layer.setValue(mixed_block + wavemap::Index3D(1, 0, 1), 4);
  semantic_layer.setValue(mixed_block + wavemap::Index3D(1, 1, 0), 6);
  changed_layer.setValue(mixed_block + wavemap::Index3D(0, 1, 0), true);
  changed_layer.setValue(mixed_block + wavemap::Index3D(1, 0, 1), true);
  changed_layer.setValue(mixed_block + wavemap::Index3D(1, 1, 0), true);

  const DiscreteCompressionConfig one_octree_level{1};
  const auto compressed_semantic =
      DiscreteLayer<int>::fromRaw(semantic_layer, one_octree_level);
  const auto compressed_changed =
      DiscreteLayer<bool>::fromRaw(changed_layer, one_octree_level);

  const DiscreteCompressionConfig two_octree_levels{2};
  DiscreteLayer<int> directly_updated_semantic(two_octree_levels);
  fillBlock(directly_updated_semantic, wavemap::Index3D(0, 8, 0),
            two_octree_levels.blockSideLength(), 7);
  directly_updated_semantic.setValue(wavemap::Index3D(3, 11, 3), 9);

  std::cout << "Discrete layer compression experiment\n";
  std::cout << "Compression stores one dominant value per configured parent "
               "region and explicit child exceptions. Reconstruction is "
               "lossless for stored voxels.\n";
  printCompressionStats("semantic layer", semantic_layer,
                        compressed_semantic);
  printCompressionStats("binary changed layer", changed_layer,
                        compressed_changed);
  std::cout << "directly updated semantic layer\n";
  std::cout << "  block height: "
            << directly_updated_semantic.config().block_height << "\n";
  std::cout << "  block side length: "
            << directly_updated_semantic.config().blockSideLength() << "\n";
  std::cout << "  compressed parents: "
            << directly_updated_semantic.parentCount() << "\n";
  std::cout << "  exceptions: "
            << directly_updated_semantic.exceptionCount() << "\n";
  std::cout << "  updated exception preserved: "
            << (sameValue(directly_updated_semantic.getValue(
                              wavemap::Index3D(3, 11, 3)),
                          9)
                    ? "yes"
                    : "no")
            << "\n";
  std::cout << "  sparse unknown remains empty: "
            << (!directly_updated_semantic.getValue(wavemap::Index3D(8, 8, 8))
                    ? "yes"
                    : "no")
            << "\n";

  const bool semantic_lossless =
      countPreservedValues(semantic_layer, compressed_semantic) ==
      semantic_layer.size();
  const bool changed_lossless =
      countPreservedValues(changed_layer, compressed_changed) ==
      changed_layer.size();

  const bool direct_update_ok =
      sameValue(directly_updated_semantic.getValue(wavemap::Index3D(3, 11, 3)),
                9) &&
      !directly_updated_semantic.getValue(wavemap::Index3D(8, 8, 8));

  if (!semantic_lossless || !changed_lossless || !direct_update_ok) {
    std::cout << "Compressed discrete layer did not reconstruct all values.\n";
    return 1;
  }
  return 0;
}

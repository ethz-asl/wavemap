#include <iostream>
#include <optional>
#include <string>

#include <wavemap/layered/discrete_layer.h>

using wavemap::layered::DiscreteCompressionConfig;
using wavemap::layered::DiscreteLayer;
using wavemap::layered::RawDiscreteLayer;

namespace {
template <typename ValueT>
bool sameValue(const std::optional<ValueT>& value, const ValueT& expected) {
  return value && *value == expected;
}

void fillBlock(DiscreteLayer<int>& layer, const wavemap::Index3D& origin,
               int side_length, int value) {
  for (int dx = 0; dx < side_length; ++dx) {
    for (int dy = 0; dy < side_length; ++dy) {
      for (int dz = 0; dz < side_length; ++dz) {
        layer.setValue(origin + wavemap::Index3D(dx, dy, dz), value);
      }
    }
  }
}

bool expect(const std::string& name, bool condition) {
  std::cout << "  " << name << ": " << (condition ? "pass" : "FAIL")
            << "\n";
  return condition;
}
}  // namespace

int main() {
  bool ok = true;

  std::cout << "Discrete layer update behavior experiment\n";

  {
    DiscreteLayer<int> layer(DiscreteCompressionConfig{1});
    fillBlock(layer, wavemap::Index3D(0, 0, 0), 2, 1);
    ok &= expect("uniform block has one parent", layer.parentCount() == 1u);
    ok &= expect("uniform block has zero exceptions",
                 layer.exceptionCount() == 0u);
    ok &= expect("uniform block observed all children",
                 layer.observedValueCount() == 8u);
    ok &= expect("uniform block reconstructs dominant value",
                 sameValue(layer.getValue(wavemap::Index3D(1, 1, 1)), 1));
  }

  {
    DiscreteLayer<int> layer(DiscreteCompressionConfig{1});
    fillBlock(layer, wavemap::Index3D(0, 0, 0), 2, 1);
    layer.setValue(wavemap::Index3D(1, 1, 1), 2);
    ok &= expect("one different child becomes one exception",
                 layer.exceptionCount() == 1u);
    ok &= expect("exception value reconstructs exactly",
                 sameValue(layer.getValue(wavemap::Index3D(1, 1, 1)), 2));

    layer.setValue(wavemap::Index3D(1, 1, 1), 1);
    ok &= expect("setting exception back removes exception",
                 layer.exceptionCount() == 0u);
    ok &= expect("restored child reconstructs dominant value",
                 sameValue(layer.getValue(wavemap::Index3D(1, 1, 1)), 1));
  }

  {
    DiscreteLayer<int> layer(DiscreteCompressionConfig{1});
    fillBlock(layer, wavemap::Index3D(0, 0, 0), 2, 1);
    layer.setValue(wavemap::Index3D(0, 0, 0), 2);
    layer.setValue(wavemap::Index3D(1, 0, 0), 2);
    layer.setValue(wavemap::Index3D(0, 1, 0), 2);
    layer.setValue(wavemap::Index3D(1, 1, 0), 2);
    layer.setValue(wavemap::Index3D(0, 0, 1), 2);
    ok &= expect("majority update changes dominant value",
                 sameValue(layer.dominantValue(wavemap::Index3D(0, 0, 0)), 2));
    ok &= expect("old minority values become exceptions",
                 layer.exceptionCount() == 3u);
    ok &= expect("new majority child reconstructs exactly",
                 sameValue(layer.getValue(wavemap::Index3D(0, 0, 0)), 2));
    ok &= expect("old minority child reconstructs exactly",
                 sameValue(layer.getValue(wavemap::Index3D(1, 1, 1)), 1));
  }

  {
    DiscreteLayer<int> layer(DiscreteCompressionConfig{1});
    layer.setValue(wavemap::Index3D(0, 0, 0), 7);
    ok &= expect("unknown child in existing parent remains unknown",
                 !layer.getValue(wavemap::Index3D(1, 1, 1)));
    ok &= expect("observed child still reconstructs",
                 sameValue(layer.getValue(wavemap::Index3D(0, 0, 0)), 7));
  }

  {
    DiscreteLayer<int> layer(DiscreteCompressionConfig{2});
    fillBlock(layer, wavemap::Index3D(0, 0, 0), 4, 3);
    layer.setValue(wavemap::Index3D(3, 3, 3), 4);
    ok &= expect("block_height 2 uses one 4x4x4 parent",
                 layer.parentCount() == 1u);
    ok &= expect("block_height 2 stores one exception",
                 layer.exceptionCount() == 1u);
    ok &= expect("block_height 2 reconstructs exception",
                 sameValue(layer.getValue(wavemap::Index3D(3, 3, 3)), 4));
  }

  if (!ok) {
    std::cout << "Discrete layer update behavior failed.\n";
    return 1;
  }
  return 0;
}

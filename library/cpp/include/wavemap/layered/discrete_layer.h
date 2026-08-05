#ifndef WAVEMAP_LAYERED_DISCRETE_LAYER_H_
#define WAVEMAP_LAYERED_DISCRETE_LAYER_H_

#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <utility>

#include <wavemap/core/common.h>

namespace wavemap::layered {

struct IndexKey {
  int x = 0;
  int y = 0;
  int z = 0;

  explicit IndexKey(const wavemap::Index3D& index)
      : x(index.x()), y(index.y()), z(index.z()) {}

  bool operator<(const IndexKey& other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

inline wavemap::Index3D indexFromKey(const IndexKey& key) {
  return wavemap::Index3D(key.x, key.y, key.z);
}

struct DiscreteCompressionConfig {
  // Matches one octree level by default: one parent represents 2x2x2 children.
  int block_height = 1;

  int blockSideLength() const { return 1 << block_height; }

  int blockVoxelCount() const {
    const int side_length = blockSideLength();
    return side_length * side_length * side_length;
  }
};

inline int floorDiv(int value, int divisor) {
  int quotient = value / divisor;
  const int remainder = value % divisor;
  if (remainder != 0 && ((remainder < 0) != (divisor < 0))) {
    --quotient;
  }
  return quotient;
}

inline int positiveModulo(int value, int divisor) {
  const int remainder = value % divisor;
  return remainder < 0 ? remainder + divisor : remainder;
}

inline IndexKey parentKey(const wavemap::Index3D& index,
                          const DiscreteCompressionConfig& config) {
  const int side_length = config.blockSideLength();
  return IndexKey(wavemap::Index3D(floorDiv(index.x(), side_length),
                                   floorDiv(index.y(), side_length),
                                   floorDiv(index.z(), side_length)));
}

inline int childOffset(const wavemap::Index3D& index,
                       const DiscreteCompressionConfig& config) {
  const int side_length = config.blockSideLength();
  const int x = positiveModulo(index.x(), side_length);
  const int y = positiveModulo(index.y(), side_length);
  const int z = positiveModulo(index.z(), side_length);
  return x + side_length * (y + side_length * z);
}

template <typename ValueT>
class RawDiscreteLayer {
 public:
  using Value = ValueT;

  void setValue(const wavemap::Index3D& index, const ValueT& value) {
    values_[IndexKey(index)] = value;
  }

  std::optional<ValueT> getValue(const wavemap::Index3D& index) const {
    const auto it = values_.find(IndexKey(index));
    if (it == values_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  bool hasValue(const wavemap::Index3D& index) const {
    return values_.find(IndexKey(index)) != values_.end();
  }

  size_t size() const { return values_.size(); }

  void clear() { values_.clear(); }

  const std::map<IndexKey, ValueT>& values() const { return values_; }

 private:
  std::map<IndexKey, ValueT> values_;
};

template <typename ValueT>
struct CompressedDiscreteCell {
  ValueT dominant_value{};
  std::set<int> observed_offsets;
  std::map<int, ValueT> exceptions;
};

template <typename ValueT>
class DiscreteLayer {
 public:
  using Value = ValueT;
  using Cell = CompressedDiscreteCell<ValueT>;

  explicit DiscreteLayer(
      DiscreteCompressionConfig config = DiscreteCompressionConfig())
      : config_(config) {}

  static DiscreteLayer fromRaw(
      const RawDiscreteLayer<ValueT>& layer,
      DiscreteCompressionConfig config = DiscreteCompressionConfig()) {
    std::map<IndexKey, std::map<int, ValueT>> grouped_values;
    for (const auto& [key, value] : layer.values()) {
      const wavemap::Index3D index = indexFromKey(key);
      grouped_values[parentKey(index, config)][childOffset(index, config)] =
          value;
    }

    DiscreteLayer compressed(config);
    for (const auto& [parent_key, child_values] : grouped_values) {
      Cell cell;
      compressed.rebuildCell(child_values, cell);
      compressed.cells_[parent_key] = cell;
    }
    return compressed;
  }

  static DiscreteLayer fromRaw2x2x2(
      const RawDiscreteLayer<ValueT>& layer) {
    return fromRaw(layer, DiscreteCompressionConfig{1});
  }

  void setValue(const wavemap::Index3D& index, const ValueT& value) {
    Cell& cell = cells_[parentKey(index, config_)];
    std::map<int, ValueT> values = expandCell(cell);
    values[childOffset(index, config_)] = value;
    rebuildCell(values, cell);
  }

  std::optional<ValueT> getValue(const wavemap::Index3D& index) const {
    const auto cell_it = cells_.find(parentKey(index, config_));
    if (cell_it == cells_.end()) {
      return std::nullopt;
    }
    const Cell& cell = cell_it->second;
    const int offset = childOffset(index, config_);
    if (cell.observed_offsets.find(offset) == cell.observed_offsets.end()) {
      return std::nullopt;
    }
    const auto exception_it = cell.exceptions.find(offset);
    if (exception_it != cell.exceptions.end()) {
      return exception_it->second;
    }
    return cell.dominant_value;
  }

  size_t parentCount() const { return cells_.size(); }

  size_t exceptionCount() const {
    size_t count = 0u;
    for (const auto& [parent_key, cell] : cells_) {
      (void)parent_key;
      count += cell.exceptions.size();
    }
    return count;
  }

  size_t observedValueCount() const {
    size_t count = 0u;
    for (const auto& [parent_key, cell] : cells_) {
      (void)parent_key;
      count += cell.observed_offsets.size();
    }
    return count;
  }

  std::optional<ValueT> dominantValue(const wavemap::Index3D& index) const {
    const auto cell_it = cells_.find(parentKey(index, config_));
    if (cell_it == cells_.end()) {
      return std::nullopt;
    }
    return cell_it->second.dominant_value;
  }

  const DiscreteCompressionConfig& config() const { return config_; }

  const std::map<IndexKey, Cell>& cells() const { return cells_; }

  void replaceCellsForLoad(DiscreteCompressionConfig config,
                           std::map<IndexKey, Cell> cells) {
    config_ = config;
    cells_ = std::move(cells);
  }

 private:
  std::map<int, ValueT> expandCell(const Cell& cell) const {
    std::map<int, ValueT> values;
    for (const int offset : cell.observed_offsets) {
      const auto exception_it = cell.exceptions.find(offset);
      values[offset] = exception_it != cell.exceptions.end()
                           ? exception_it->second
                           : cell.dominant_value;
    }
    return values;
  }

  void rebuildCell(const std::map<int, ValueT>& values, Cell& cell) const {
    std::map<ValueT, size_t> counts;
    ValueT dominant_value{};
    size_t dominant_count = 0u;
    for (const auto& [offset, value] : values) {
      (void)offset;
      const size_t count = ++counts[value];
      if (dominant_count < count) {
        dominant_value = value;
        dominant_count = count;
      }
    }

    cell.dominant_value = dominant_value;
    cell.observed_offsets.clear();
    cell.exceptions.clear();
    for (const auto& [offset, value] : values) {
      cell.observed_offsets.insert(offset);
      if (value != dominant_value) {
        cell.exceptions[offset] = value;
      }
    }
  }

  DiscreteCompressionConfig config_;
  std::map<IndexKey, Cell> cells_;
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_DISCRETE_LAYER_H_

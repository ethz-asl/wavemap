#ifndef WAVEMAP_LAYERED_ENDPOINT_ATTRIBUTE_POINTCLOUD_H_
#define WAVEMAP_LAYERED_ENDPOINT_ATTRIBUTE_POINTCLOUD_H_

#include <cstddef>
#include <type_traits>
#include <vector>

#include <wavemap/core/data_structure/pointcloud.h>
#include <wavemap/layered/layer_traits.h>

namespace wavemap::layered {
namespace detail {
template <typename LayerTagT>
struct EndpointAttributeSlot {
  std::vector<LayerValueT<LayerTagT>> values;
};
}  // namespace detail

// Structure-of-arrays point cloud whose endpoint attributes remain aligned by
// point index. Positions use the existing Pointcloud format and every
// configured attribute uses one contiguous typed array.
template <typename... LayerTags>
class EndpointAttributePointcloud
    : private detail::EndpointAttributeSlot<LayerTags>... {
 public:
  EndpointAttributePointcloud() = default;
  explicit EndpointAttributePointcloud(size_t size) { resize(size); }

  bool empty() const { return points_.empty(); }
  size_t size() const { return points_.size(); }

  void resize(size_t size) {
    points_.resize(size);
    (attributeValues<LayerTags>().resize(size), ...);
  }

  void clear() {
    points_.clear();
    (attributeValues<LayerTags>().clear(), ...);
  }

  void truncate(size_t size) {
    CHECK_LE(size, this->size());
    points_.conservativeResize(size);
    (attributeValues<LayerTags>().resize(size), ...);
  }

  Pointcloud<>& points() { return points_; }
  const Pointcloud<>& points() const { return points_; }

  auto point(size_t index) { return points_[index]; }
  auto point(size_t index) const { return points_[index]; }

  template <typename LayerTagT>
  std::vector<LayerValueT<LayerTagT>>& attributeValues() {
    static_assert((std::is_same_v<LayerTagT, LayerTags> || ...),
                  "Endpoint attribute is not present in this point cloud.");
    return static_cast<detail::EndpointAttributeSlot<LayerTagT>&>(*this)
        .values;
  }

  template <typename LayerTagT>
  const std::vector<LayerValueT<LayerTagT>>& attributeValues() const {
    static_assert((std::is_same_v<LayerTagT, LayerTags> || ...),
                  "Endpoint attribute is not present in this point cloud.");
    return static_cast<const detail::EndpointAttributeSlot<LayerTagT>&>(*this)
        .values;
  }

  template <typename LayerTagT>
  LayerValueT<LayerTagT>& attribute(size_t index) {
    return attributeValues<LayerTagT>()[index];
  }

  template <typename LayerTagT>
  const LayerValueT<LayerTagT>& attribute(size_t index) const {
    return attributeValues<LayerTagT>()[index];
  }

  bool hasConsistentSizes() const {
    return ((attributeValues<LayerTags>().size() == size()) && ...);
  }

 private:
  Pointcloud<> points_;
};

template <typename... LayerTags>
using PosedEndpointAttributePointcloud =
    PosedObject<EndpointAttributePointcloud<LayerTags...>>;

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_ENDPOINT_ATTRIBUTE_POINTCLOUD_H_

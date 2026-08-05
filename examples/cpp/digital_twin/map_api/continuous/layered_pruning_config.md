# Layered pruning config

This example uses `LayeredVoxel`, which stores occupancy plus continuous layers:

```cpp
struct ContinuousLayers {
  Rgb rgb;
  float traversability;
};
```

The map still stores wavelet coefficients. Pruning decides whether a node's detail coefficients are relevant enough to keep.

The layered pruning score is:

```cpp
score = sum(weight * abs(wavelet_detail) / scale)
```

A node is kept when:

```cpp
score > combined_threshold
```

## Parameters

`scale` is the amount of detail that counts as one unit of relevance for a field. Larger scale means that field is more tolerant and will allow more pruning.

`weight` is the relative importance of the field. Use `0` to ignore a field during pruning.

`combined_threshold` is the final pruning decision threshold. The default is `1`.

## Examples

Occupancy-only pruning:

```cpp
auto pruning = makeOccupancyOnlyPruningConfig();
```

Equal weighting for occupancy, RGB, and traversability:

```cpp
auto pruning = makeEqualLayerPruningConfig();
```

Custom field scales and weights:

```cpp
auto pruning = makeLayeredPruningConfigWithScales(
    /*occupancy_weight=*/1.f,
    /*rgb_weight=*/1.f,
    /*traversability_weight=*/2.f,
    /*occupancy_scale=*/1.f,
    /*rgb_scale=*/0.1f,
    /*traversability_scale=*/0.1f);
```

The trade-off is expected: ignoring extra layers usually gives stronger compression, but can remove RGB/traversability details. Including extra layers preserves them, but may keep more nodes.

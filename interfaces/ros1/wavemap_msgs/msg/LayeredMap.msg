Header header

# True for a complete snapshot. False for an incremental patch whose
# continuous blocks and discrete parent cells replace only the included data.
bool is_full_update

# Existing Wavemap structured message for the continuous wavelet map.
Map continuous_map

# Additional non-wavelet discrete layers aligned with the same voxel index space.
DiscreteLayer[] discrete_layers

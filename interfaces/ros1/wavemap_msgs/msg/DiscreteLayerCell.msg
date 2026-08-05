# One compressed parent cell in a discrete layer.
# The parent_index identifies the compressed parent block.
Index3D parent_index

# Default value for all observed children unless an exception overrides it.
# Exactly one value array is used, according to DiscreteLayer.value_type.
int32 dominant_int32
uint8 dominant_uint8

# Child offsets observed inside this parent cell.
# This keeps sparse unknown children unknown instead of implicitly assigning
# them the dominant value.
int32[] observed_offsets

# Child offsets whose value differs from the dominant value.
int32[] exception_offsets

# Exception values aligned with exception_offsets. Exactly one array is used,
# according to DiscreteLayer.value_type.
int32[] exception_int32_values
uint8[] exception_uint8_values

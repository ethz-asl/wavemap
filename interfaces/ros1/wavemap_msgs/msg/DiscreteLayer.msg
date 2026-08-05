# A compressed discrete voxel layer stored as parent cells with dominant values
# and explicit exceptions.
string name
string value_type

# Same meaning as DiscreteCompressionConfig.block_height:
# 1 -> 2x2x2, 2 -> 4x4x4, etc.
int32 block_height

DiscreteLayerCell[] cells

# A compressed discrete voxel layer stored as parent cells with dominant values
# and explicit exceptions.
string name
string value_type

# Same meaning as DiscreteCompressionConfig.block_height:
# 1 -> 2x2x2, 2 -> 4x4x4, etc.
int32 block_height

# Optional labels for integer category values. Values without an entry remain
# valid and are displayed using their numeric value.
DiscreteLayerCategory[] categories

DiscreteLayerCell[] cells

# Parent cells removed since the preceding update. Only used by incremental
# LayeredMap messages; full snapshots implicitly remove all omitted cells.
Index3D[] deleted_parent_indices

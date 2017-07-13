# Godel supports two processes at the moment: surface blending and line scanning
# This msg encapsulates a motion plan for one of these components.

# Enumeration represeting plan type
uint32 SEGMENT_PROCESS_TYPE=0
uint32 SEGMENT_APPROACH_TYPE=1
uint32 SEGMENT_TRAVERSE_TYPE=2

uint32[] segment_types
uint32[] segment_size

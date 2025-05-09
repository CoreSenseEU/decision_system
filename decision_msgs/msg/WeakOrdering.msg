# Each alternative that was ordered.
Alternative[] alternatives

# A list of integer ranks encoding a weak preference ordering on the set of
# alteratives. Each rank corresponds to the alternative with the same index.
# Assume that lower values are always better, and identical values are tied.
# Consequently no cycles can exist.
uint8[] ranks 

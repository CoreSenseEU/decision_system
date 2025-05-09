# Each alternative that was assessed.
Alterntive[] alternatives

# An array of cues that was used to assess each alternative.
Cue[] cues

# A numpy-style N-by-M matrix of assessments where N is the number of alternatives 
# and M is the number of cues. Each assessment is represented
# as the column in this matrix corresponding with the cue with the same index.
# In each row are preference scores of each assessment for each alternative
# with the corresponding index.
float32[] scores

# Each alternative that was evaluated.
Alternative[] alternatives

# Feature axes by which each alternative is judged.
string[] axes

# A numpy-style N-by-M matrix of judgments where N is the number of alternatives
# and M is the number of features of each alternative. Each judgment is represented
# as the row in this matrix corresponding with the alternative with the same index.
# In each column are feature scores of each judgment along the corresponding 
# feature axis with the same index.
float32[] scores

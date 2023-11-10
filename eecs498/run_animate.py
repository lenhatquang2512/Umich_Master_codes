"""
Script to visualize a GMM.
"""

from gmm import gmm
from run import get_data
import numpy as np

np.random.seed(445)
X = get_data()
_, _, _, _, BIC = gmm(X[:,:2], 3, num_iter=30, plot=True)
# Here, we train on only the first two columns of our data for visualization
# purposes. In run.py, you should use all columns of X
assert BIC > 1782.09 and BIC < 1782.11, 'Expected BIC = 1782.10, got BIC = ' + str(BIC)
print('Correct BIC value! with BIC =' + str(BIC))


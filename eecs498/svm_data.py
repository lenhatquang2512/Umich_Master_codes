import numpy as np
from sklearn.datasets import fetch_openml
from sklearn import svm
from sklearn import preprocessing
from scipy.io import arff
import matplotlib.pyplot as plt


# Load data from https://www.openml.org/d/554
X, y = fetch_openml('mnist_784', version=1, return_X_y=True, as_frame=False)

# plot one digital image
j = 1
plt.title('The jth image is a {label}'.format(label=int(y[j]))) 
plt.imshow(X[j].reshape((28,28)), cmap='gray')
plt.show()

#Preprocessing: scale data with zero mean and unit variance
X = preprocessing.scale(X)

# Extract out the digits "4" and "9"
X4 = X[y=='4',:]
X9 = X[y=='9',:]
y4 = 4*np.ones((len(X4),), dtype=int)
y9 = 9*np.ones((len(X9),), dtype=int)

# split the data into test and train (which further splitted into train and validation)

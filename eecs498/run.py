"""
Script for running GMM soft clustering
"""

import matplotlib.pyplot as plt
import numpy as np
import string as s

from sklearn.datasets import fetch_openml
from gmm import gmm


def get_data():
    """Load penguins data from Github."""
    penguins = fetch_openml("penguins", as_frame=False)
    # get 'culmen_length_mm', 'culmen_depth_mm', 'flipper_length_mm',
    # 'body_mass_g' features
    X = penguins["data"][:, 1:5]
    # drop NA values
    X = X[~np.isnan(X).any(axis=1)]
    X = (X - np.mean(X, axis=0)) / np.std(X, axis=0)
    print(f"Shape of the input data: {X.shape[0]} by {X.shape[1]}")
    return X


def main():
    """Call GMM with different numbers of clusters.

    - num_K is an array containing the tested cluster sizes
    - cluster_proportions maps each cluster size to a size by 1 vector
      containing the mixture proportions
    - means is a dictionary mapping the cluster size to matrix of means
    - z_K maps each cluster size into a num_points by k matrix of pointwise
      cluster membership probabilities
    - sigma2 maps each cluster size to the corresponding sigma^2 value learnt
    - BIC_K contains the best BIC values for each of the cluster sizes
    """
    print(
            "We'll try different numbers of clusters with GMM, using multiple runs"
            " for each to identify the 'best' results"
    )
    np.random.seed(445)
    trainX = get_data()
    num_K = range(2, 9)  # List of cluster sizes
    BIC_K = np.zeros(len(num_K))
    
    # MODIFIED
    means = {}  # Dictionary mapping cluster size to corresponding matrix of means
    cluster_proportions = {}  # Dictionary mapping cluster size to corresponding mixture proportions vector
    z_K = {}
    sigma2 = {}  # Dictionary mapping cluster size to the learnt variance value
    num_iter = 10
    for idx in range(len(num_K)):
        # Running
        k = num_K[idx]
        print("%d clusters..." % k)
        # TODO: Run gmm function 10 times and get the best set of parameters
        # for this particular value of k. Use the default num_iter=10 in calling gmm()
        # for i in range(10):
        #     pass
        for i in range(10):
          mu, pk, zk, si2, BIC = gmm(trainX, k,num_iter=num_iter)  
          if BIC_K[idx] == 0 or BIC_K[idx] > BIC:
              means[k] = mu
              cluster_proportions[k] = pk
              z_K[k] = zk
              sigma2[k] = si2
              BIC_K[idx] = BIC

    # TODO: Part g: Make a plot to show BIC as function of clusters K
    min_bic = BIC_K[0]
    for idx in range(0,7):
        if BIC_K[idx] < min_bic:
            min_bic = BIC_K[idx]
            min_bic_k_val = idx
    min_k_val = num_K[min_bic_k_val]
    print("Optimal number of clusters: " + str(min_k_val))
    print("Means: " + str(means[min_k_val]))
    plt.plot(num_K, BIC_K, 'bo', label='BIC vs num_K')
    plt.xlabel('num_K')
    plt.ylabel('BIC')
    plt.legend(loc='upper right')
    plt.savefig("bic_plot.png")


if __name__ == "__main__":
    main()
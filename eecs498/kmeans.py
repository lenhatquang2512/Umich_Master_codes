"""
K-Means Clustering
"""

import sys 
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import KMeans


def visualize_kmeans(data, n_clusters, init):
    """
    Visualize the result of k-means clustering
    """

    # TODO: Define a scikit-learn KMeans object
    # - Set argument n_clusters (number of clusters) to n_clusters
    # - Set argument init ('random' or 'k-means++') to init
    # - Set random_state to 20
    kmeans = KMeans(n_clusters=n_clusters,init = init,random_state=20)

    # Fit data to obtain clusters
    kmeans.fit(data)

    # TODO: print final value of objective function ("inertia_") 
    print(f"The final value of objective function using {init} is :")
    print(kmeans.inertia_)


    # Plot each cluster on the same axes
    plt.figure()
    for cluster in np.arange(n_clusters):
        plt.plot(data[kmeans.labels_==cluster, 0], data[kmeans.labels_==cluster, 1], 'x')
    plt.title(f"K-Means Clustering Visualization - {init}")
    plt.savefig(f'kmeans_visualization_{init}.png', dpi=200, bbox_inches='tight')


if __name__ == '__main__':
    data = np.loadtxt(sys.argv[1])
    n_clusters = int(sys.argv[2])
    init = sys.argv[3]

    print("Data file:", sys.argv[1])
    print("Number of clusters:", n_clusters)
    print("Initialization method:", init)
    visualize_kmeans(data, n_clusters, init)

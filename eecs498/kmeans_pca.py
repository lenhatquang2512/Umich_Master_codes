"""
K-Means Clustering
"""

import sys 
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import KMeans
from sklearn.datasets import load_digits
from sklearn.decomposition import PCA


def visualize_kmeans_pca(data, n_clusters, init):
    """
    Visualize the result of k-means clustering
    """
    # transform data using PCA
    reduced_data = PCA(n_components=2).fit_transform(data)


    # TODO: Define a scikit-learn KMeans object
    # - Set argument n_clusters (number of clusters) to n_clusters
    # - Set argument init ('random' or 'k-means++') to init
    # - Set random_state to 20
    kmeans = KMeans(init=init, n_clusters=n_clusters, n_init=10,random_state=20)

    # Fit data to obtain clusters
    kmeans.fit(reduced_data)

    # TODO: print final value of objective function ("inertia_") 
    print(f"The final value of objective function using {init} is :")
    print(kmeans.inertia_)


    # Plot each cluster on the same axes
    plt.figure()
    for cluster in np.arange(n_clusters):
        plt.plot(reduced_data[kmeans.labels_==cluster, 0], reduced_data[kmeans.labels_==cluster, 1], 'x')
    plt.title(f"K-Means Clustering Visualization - {init}")
    # plt.legend()
    plt.show()
    plt.savefig(f'kmeans_visualization_pca_hand_digit_using_{init}.png', dpi=200, bbox_inches='tight')


if __name__ == '__main__':
    data, labels = load_digits(return_X_y=True)
    (n_samples, n_features), n_digits = data.shape, np.unique(labels).size
    print(f"# digits: {n_digits}; # samples: {n_samples}; # features {n_features}")
    # data = np.loadtxt(sys.argv[1])
    # n_clusters = int(sys.argv[2])
    # init = sys.argv[3]

    # print("Data file:", sys.argv[1])
    print("Number of clusters:", n_digits)
    print("Initialization method:", "k-means++")
    visualize_kmeans_pca(data, n_digits, "k-means++")
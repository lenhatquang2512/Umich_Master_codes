"""
The gmm function takes in as input a data matrix X and a number of gaussians in
the mixture model

The implementation assumes that the covariance matrix is shared and is a
spherical diagonal covariance matrix

If you get this ImportError
    ImportError: cannot import name 'logsumexp' from 'scipy.special',
you may need to update your scipy install to >= 0.19.0
"""

from scipy.stats import norm, multivariate_normal
from scipy.special import logsumexp
import numpy as np
import math
import matplotlib.pyplot as plt


def calc_logpdf(x, mean, cov):
    """Return log probability density."""
    x = multivariate_normal.logpdf(x, mean=mean, cov=cov)
    return x


def gmm(trainX, num_K, num_iter=10, plot=False):
    """Fit a gaussian mixture model on trainX data with num_K clusters.

    trainX is a NxD matrix containing N datapoints, each with D features
    num_K is the number of clusters or mixture components
    num_iter is the maximum number of EM iterations run over the dataset

    Description of other variables:
        - mu, which is KxD, the coordinates of the means
        - pk, which is Kx1 and represents the cluster proportions
        - zk, which is NxK, has at each z(n,k) the probability that the nth
          data point belongs to cluster k, specifying the cluster associated
          with each data point
        - si2 is the estimated (shared) variance of the data
        - BIC is the Bayesian Information Criterion (smaller BIC is better)
    """
    N = trainX.shape[0]
    D = trainX.shape[1]

    if num_K >= N:
        print("You are trying too many clusters")
        raise ValueError
    if plot and D != 2:
        print("Can only visualize if D = 2")
        raise ValueError

    si2 = 5  # Initialization of variance
    pk = np.ones((num_K, 1)) / num_K  # Uniformly initialize cluster proportions
    mu = np.random.randn(num_K, D)  # Random initialization of clusters
    zk = np.zeros(
        [N, num_K]
    )  # Matrix containing cluster membership probability for each point

    if plot:
        plt.ion()
        fig = plt.figure()
    for iter in range(0, num_iter):
        """Iterate through one loop of the EM algorithm."""
        if plot:
            plt.clf()
            xVals = trainX[:, 0]
            yVals = trainX[:, 1]
            x = np.linspace(np.min(xVals), np.max(xVals), 500)
            y = np.linspace(np.min(yVals), np.max(yVals), 500)
            X, Y = np.meshgrid(x, y)
            pos = np.array([X.flatten(), Y.flatten()]).T
            plt.scatter(xVals, yVals, color="black")
            pdfs = []
            for k in range(num_K):
                rv = multivariate_normal(mu[k], si2)
                pdfs.append(rv.pdf(pos).reshape(500, 500))
            pdfs = np.array(pdfs)
            plt.contourf(X, Y, np.max(pdfs, axis=0), alpha=0.8)
            plt.pause(0.01)

        """
        E-Step
        In the first step, we find the expected log-likelihood of the data
        which is equivalent to:
        Finding cluster assignments for each point probabilistically
        In this section, you will calculate the values of zk(n,k) for all n and
        k according to current values of si2, pk and mu
        """
        # TODO: Implement the E-step
        for i in range(N):
            for j in range(num_K):
                exp_arr = np.zeros([num_K, 1])
                for k in range(num_K):
                    exp_arr[k] += np.log(pk[k])
                    exp_arr[k] += multivariate_normal.logpdf(trainX[i], mu[k], si2*np.eye(D))
                b = np.exp(logsumexp(exp_arr))
                a = pk[j]*multivariate_normal.pdf(trainX[i], mu[j], si2*np.eye(D))
                zk[i][j] = a/b


        """
        M-step
        Compute the GMM parameters from the expressions which you have in the spec
        """

        # Estimate new value of pk
        # TODO
        for k in range(num_K):
            sum = 0
            for i in range(N):
                sum += zk[i][k]
            pk[k] = sum / N

        # Estimate new value for means
        # TODO
        for k in range(num_K):
            n_hat = 0
            sum_zik_x = 0
            for i in range(N):
                n_hat += zk[i][k]
                sum_zik_x += zk[i][k]*trainX[i]
            mu[k] = sum_zik_x / n_hat

        # Estimate new value for sigma^2
        # TODO
        sum = 0
        for i in range(N):
            for k in range(num_K):
                sum += zk[i][k]*np.dot( np.transpose(trainX[i] - mu[k]) , trainX[i] - mu[k])
        si2 = sum / (N*D)

    if plot:
        plt.ioff()
        plt.savefig('visualize_clusters.png')
    # Computing the expected log-likelihood of data for the optimal parameters computed
    # TODO
    maximum_log_likelihood = 0
    for i in range(N):
        max_log_exp_arr = np.zeros([num_K,1])
        for k in range(num_K):
            max_log_exp_arr[k] += np.log(pk[k])
            max_log_exp_arr[k] += multivariate_normal.logpdf(trainX[i], mu[k], si2*np.eye(D))
        maximum_log_likelihood += logsumexp(max_log_exp_arr)
    # print(maximum_log_likelihood)
    # Compute the BIC for the current clustering
    # BIC = None  # TODO: calculate BIC
    BIC = (num_K*(D + 1))*np.log(N) - 2*maximum_log_likelihood

    return mu, pk, zk, si2, BIC

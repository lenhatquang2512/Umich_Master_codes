import numpy as np
import matplotlib.pyplot as plt
import time
from sklearn.decomposition import PCA

def validate_PCA(states, train_data):
  from sklearn.decomposition import PCA
  pca = PCA()
  pca.fit(train_data)
  true_matrix = pca.components_.T
  true_ev = pca.explained_variance_
  
  output_matrix = states['transform_matrix']
  error = np.mean(np.abs(np.abs(true_matrix) - np.abs(output_matrix)) / np.abs(true_matrix))
  if error > 0.01:
    print('Matrix is wrong! Error=',error)
  else:
    print('Matrix is correct! Error=', error)

  output_ev = states['eigen_vals']
  error = np.mean(np.abs(true_ev - output_ev) / true_ev)
  if error > 0.01:
    print('Variance is wrong! Error=', error)
  else:
    print('Variance is correct! Error=', error)

def train_PCA(train_data):
  ##### TODO: Implement here!! #####
  # Note: do NOT use sklearn here!
  # Hint: np.linalg.eig() might be useful
  X = train_data
  X_meaned = X - np.mean(X , axis = 0)
  cov_mat = np.cov(X_meaned , rowvar = False)
  M,D = np.linalg.eig(cov_mat)
  sorted_index = np.argsort(M)[::-1]
  M_sort = M[sorted_index]
  D_sort = D[:,sorted_index]
  
  states = {
      'transform_matrix': np.identity(train_data.shape[-1]),
      'eigen_vals': np.ones(train_data.shape[-1])
  }
  ##### TODO: Implement here!! #####
  states = {
    'transform_matrix': D_sort,
    'eigen_vals': M_sort
  }
  return states


def plot_eigenvalues(n_components,states):
  index_pc = np.arange(n_components) + 1
  fig = plt.figure(1)
  plt.plot(index_pc,states['eigen_vals'][0:10],'go-')
  plt.xticks(np.arange(10)+1)
  plt.xlabel('Index of corresponding principal components')
  plt.ylabel('Eigenvalues')
  plt.grid()
  plt.title('Plot of Eigenvalues')
  # plt.legend(loc='upper right')
  # plt.savefig("hw5_eecs498_1b.png")
  plt.show()


def plot_faces(title, faces):
    fig = plt.figure(2)
    # plt.suptitle(title, y=1.05, fontsize=24)
    for i in range(10):
        sub = plt.subplot(2,5, i + 1)
        sub.imshow(faces[i].reshape(48,42), cmap='Greys_r')
        # sub.imshow(faces[i],cmap='rainbow')
        plt.xticks(())
        plt.yticks(())

    plt.tight_layout()
    # plt.savefig("hw5_eecs498_1c.png")
    plt.show()


def plot_eigenfaces(n_components,train_data,states):
  P_reduce = states['transform_matrix'][:,0:n_components]
  # transform
  X = train_data
  X_meaned = X - np.mean(X , axis = 0)

  Z = np.dot(P_reduce.transpose() , X_meaned.transpose())
  X_approx = np.matmul(P_reduce,Z)
  plot_faces('10 components', X_approx.transpose())

def plot_eigenvectors(n_components,train_data,states):
    P_reduce = states['transform_matrix'][:,0:n_components]
    train_data_mean = np.mean(train_data,axis=0)
    # print(train_data_mean.shape)
    fig = plt.figure()
    # plt.suptitle(title, y=1.05, fontsize=24)
    for i in range(10):
        sub = plt.subplot(2,5, i + 1)
        if i == 0:
            sub.imshow(train_data_mean.reshape(48,42), cmap='Greys_r')
        else:
            sub.imshow(P_reduce[:,i-1].reshape(48,42), cmap='Greys_r')

        plt.xticks(())
        plt.yticks(())

    plt.tight_layout()
    # plt.savefig("hw5_eecs498_1c.png")
    plt.show()

def check_preserved_variance(states):
    evals = states['eigen_vals']
    vL = np.sum(evals)
    L = evals.shape[0]
    for k in range(1,L+1):
      vk = np.sum(evals[0:k])
      if np.isclose(vk/vL, 0.99,rtol=5e-5):
      # if k == 166:
          #print((vk/vL)-0.99)
          print('Percentage of total variance: ' + str(100 *(vk/vL)))
          print('K = ' + str(k))
          print('Percentage of reduction: ' + str(100 *(((L-k)/L))))
          break
    

def main():
  # Load data
  start = time.time()
  images = np.load('pca_data.npy')
  num_data = images.shape[0]
  train_data = images.reshape(num_data, -1)

  states = train_PCA(train_data)
  print('training time = %.1f sec'%(time.time() - start))

  validate_PCA(states, train_data)

  plot_eigenvalues(10,states)
  # plot_eigenfaces(10,train_data,states)
  plot_eigenvectors(10,train_data,states)
  check_preserved_variance(states)


if __name__ == '__main__':
  main()

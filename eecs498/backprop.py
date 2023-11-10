import numpy as np

# Problem 1 (i)
def f_1(x, y, z):
    """
    Computes the forward and backward pass through the computational graph 
    of (i)

    Inputs:
    - x, y, z: Python floats

    Returns a tuple of:
    - L: The output of the graph
    - grads: A tuple (grad_x, grad_y, grad_z)
    giving the derivative of the output L with respect to each input.
    """
    L = None
    grad_x = None
    grad_y = None
    grad_z = None
    ###########################################################################
    # TODO: Implement the forward pass for the computational graph for (i) and#
    # store the output of this graph as L                                     #
    ###########################################################################
    y1 = y
    y2 = y
    p = x + y1
    q = y2 + z
    L = p * q
    ###########################################################################
    #                              END OF YOUR CODE                           #
    ###########################################################################


    ###########################################################################
    # TODO: Implement the backward pass for the computational graph for (i)   #
    # Store the gradients for each input                                      #
    ###########################################################################
    grad_L = 1
    grad_p = grad_L * q
    grad_q = grad_L * p
    grad_y1 = grad_p * 1
    grad_x = grad_p * 1
    grad_y2 = grad_q * 1
    grad_z = grad_q * 1
    grad_y = grad_y1 + grad_y2
    ###########################################################################
    #                              END OF YOUR CODE                           #
    ###########################################################################

    grads = (grad_x, grad_y, grad_z)
    return L, grads



# Problem 1 (iii)
def f_3(x0, x1, w0, w1, w2):
    """
    Computes the forward and backward pass through the computational graph 
    of (iii)

    Inputs:
    - x0, x1, w0, w1, w2: Python floats

    Returns a tuple of:
    - L: The output of the graph
    - grads: A tuple (grad_x0, grad_x1, grad_w0, grad_w1, grad_w2)
    giving the derivative of the output L with respect to each input.
    """
    L = None
    grad_x0 = None
    grad_x1 = None
    grad_w0 = None
    grad_w1 = None
    grad_w2 = None
    ###########################################################################
    # TODO: Implement the forward pass for the computational graph for (iii)  #
    # and store the output of this graph as L                                 #
    ###########################################################################
    p0 = w0 * x0 + w1 * x1
    p1 = p0 + w2
    p2 = -p1
    p3 = np.exp(p2)
    p4 = p3 + 1
    L = 1.0/p4
    ###########################################################################
    #                              END OF YOUR CODE                           #
    ###########################################################################


    ###########################################################################
    # TODO: Implement the backward pass for the computational graph for (iii) #
    # Store the gradients for each input                                      #
    ###########################################################################
    grad_L = 1
    grad_p4 = -1.0/(p4**2) * grad_L
    grad_p3 = 1 * grad_p4
    grad_p2 = p3 * grad_p3 #np.exp(p2) * grad_p3
    grad_p1 = -1 * grad_p2
    grad_w2 = 1 * grad_p1
    grad_p0 = 1 * grad_p1
    grad_w0 = x0 * grad_p0
    grad_x0 = w0 * grad_p0
    grad_w1 = x1 * grad_p0
    grad_x1 = w1 * grad_p0
    ###########################################################################
    #                              END OF YOUR CODE                           #
    ###########################################################################

    grads = (grad_x0, grad_x1, grad_w0, grad_w1, grad_w2)
    return L, grads


# Problem 1 (iv)
def f_4(x, y, z):
    """
    Computes the forward and backward pass through the computational graph 
    of (iv)

    Inputs:
    - x, y, z: Python floats

    Returns a tuple of:
    - L: The output of the graph
    - grads: A tuple (grad_x, grad_y, grad_z)
    giving the derivative of the output L with respect to each input.
    """
    L = None
    grad_x = None
    grad_y = None
    grad_z = None
    ###########################################################################
    # TODO: Implement the forward pass for the computational graph for (iv)   #
    # and store the output of this graph as L                                 #
    ###########################################################################
    a = -x 
    b = np.exp(y)
    c = np.exp(z)
    p = b * c
    d = b + p
    e = c / p
    m = a - d
    n = m / e
    L = n ** 2
    ###########################################################################
    #                              END OF YOUR CODE                           #
    ###########################################################################


    ###########################################################################
    # TODO: Implement the backward pass for the computational graph for (iv)  #
    # Store the gradients for each input                                      #
    ###########################################################################
    grad_L = 1
    grad_n = 2*n * grad_L

    grad_m = 1.0/e * grad_n
    grad_e = -m/e**2 * grad_n

    grad_d = -1 * grad_m
    grad_a = 1 * grad_m

    grad_c_1 = 1.0/p * grad_e
    grad_p_1 = -c/p**2 * grad_e 
    
    grad_b_1 = 1 * grad_d
    grad_p_2 = 1 * grad_d 

    grad_p = grad_p_1 + grad_p_2

    grad_b_2 = c* grad_p
    grad_c_2 = b* grad_p
    grad_b = grad_b_1 + grad_b_2
    grad_c = grad_c_1 + grad_c_2
    grad_x = -1 * grad_a
    grad_y = b * grad_b 
    grad_z = c * grad_c 
    ###########################################################################
    #                              END OF YOUR CODE                           #
    ###########################################################################

    grads = (grad_x, grad_y, grad_z)
    return L, grads


def main():
    # verified again using result from MATLAB
    L,grads = f_1(1,2,3)
    print(L)
    print(grads)

    L,grads = f_3(0.01,0.02,0.03,0.04,0.05)
    print(L)
    print(grads)

    L,grads = f_4(0.01,0.02,0.03)
    print(L)
    print(grads)


if __name__== "__main__":
    main()



# MATLAB code
# close all;
# clear;
# clc;
# format default;

# syms x y z

# a = -x ;
# b = exp(y);
# c = exp(z);
# p = b * c;
# d = b + p;
# e = c / p;
# m = a - d;
# n = m / e;
# L = n ^2;

# dx = diff(L,x);
# dy = diff(L,y);
# dz = diff(L,z);

# d = [dx dy dz];

# x = 0.01;
# y = 0.02;
# z = 0.03;

# for i = 1:3
#     disp(subs(d(i)));
# end
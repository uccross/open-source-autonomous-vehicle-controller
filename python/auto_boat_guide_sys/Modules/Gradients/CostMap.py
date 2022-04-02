"""
.. module:: CostMap.py
    :platform: MacOS, Unix, Windows,
    :synopsis: 
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D


def cost_map(m, n, i_start_point, i_stop_point, k=1, umin=0, umax=10):
    """
    :param m: number of rows of matrix
    :param n: number of columns of matrix
    :param umin: minimum cost 
    :param umax: maximum cost
    :param k: weight
    """

    # Kinetic energy
    # k_e = 0.5*mass*(v**2.0)
    
    a = np.array([i_start_point])
    b = np.array([i_stop_point])

    U = np.ones((m, n))

    # Sanity check
    ds = np.linalg.norm(np.array([[0, 0]])-np.array([[0, 1]]))

    print("ds = {}".format(ds))

    for iy in range(0, m):
        for ix in range(0, n):
            p = np.array([[ix, iy]])
            a_mag = np.linalg.norm(a-p)
            b_mag = np.linalg.norm(b-p)
            if a_mag == 0.0:
                a_mag = ds
            if b_mag == 0.0:
                b_mag = ds
            U[iy][ix] -= (1/((a_mag**(1/k))) + (1/((b_mag**(1/k)))))

    # Normalize
    U -= np.min(U)
    U /= np.abs(np.max(U) - np.min(U))
    U *= np.abs(umax - umin)

    return U

if __name__ == "__main__":
    m = 100
    n = 100
    mass = 20.0
    v = 2.0
    k_e = 0.5*mass*(v**2.0)

    i_start = [10, 10]
    i_stop = [m-20, n-30]

    J = cost_map(m, n, i_start, i_stop, 1000000, 0, k_e)

    Jmin = 0.0
    Jmax = np.max(J)  # 0.5*mass*(v**2.0)

    ###########################################################################
    # Plot

    Y = np.linspace(0, m-1, m)
    X = np.linspace(0, n-1, n)
    X, Y = np.meshgrid(X, Y)

    print("X.shape: {}".format(X.shape))
    print("Y.shape: {}".format(Y.shape))
    print("J.shape: {}".format(J.shape))

    fig = plt.figure()
    ax = fig.gca(projection='3d')   # Create the axes
    # f = ax.matshow(J, cmap=cm.plasma, vmin=Jmin, vmax=Jmax,
    #                extent=[0, n,
    #                        m, 0],
    #                origin='upper')
    # 

    # Plot the 3d surface
    surface = ax.plot_surface(X, Y, J,
                            cmap=cm.plasma,
                            rstride=2,
                            cstride=2)
                            
    ax.set_xlabel('x (meters)')
    ax.set_ylabel('y (meters)')
    ax.set_zlabel('Kinetic Energy (Joules)')
    plt.title('Energy Cost Gradient')

    ax.xaxis.set_ticks_position('top')
    plt.show()

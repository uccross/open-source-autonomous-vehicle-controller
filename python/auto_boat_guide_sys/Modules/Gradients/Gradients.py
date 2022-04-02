"""
.. module:: Gradients.py
	:platform: MacOS, Unix, Windows,
	:synopsis: A class for Gaussian Random Fields (GRF)\
    :note: Credit to:
            https://andrewwalker.github.io/statefultransitions/post/gaussian-fields/
            and 
            https://garrettgoon.com/gaussian-fields/
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu> with some modified code taken from above links
"""

import numpy as np
from ..Constants import Constants as CNST


class GaussianRandomField():
    def __init__(self, kSize=CNST.kSize, kSigma=CNST.kSigma, kN=CNST.kN,
                 mMax=CNST.mMax, mMin=CNST.mMin, xLen=CNST.xLen,
                 yLen=CNST.yLen, alpha=CNST.alpha, x=[], y=[], x0=0, xf=10,
                 y0=0, yf=10):
        """
        :param kSize: The The width and hieght dimension such that K is an 
        [m x m] matrix
        :param kN: The number of times to convolve the kernel
        :param mMax: The maximum possible measurement of the surface
        :param mMin: The minimum possible measurement of the surface
        :param xLen: Number of elements in a field's x coordinate vector
        :param yLen: Number of elements in a field's y coordinate vector
        :return: none
        """
        self.kSize = kSize
        self.kSigma = kSigma
        self.kN = kN
        self.mMax = mMax
        self.mMin = mMin

        self.kMu = 0.0
        self.K = self.GaussianKernel()

        Y = np.linspace(y0, yf, yLen)
        X = np.linspace(x0, xf, xLen)
        self.X, self.Y = np.meshgrid(X, Y)

        self.Z = np.random.normal(0.0, self.kSigma,
                                  size=(yLen, xLen))

        N = int(self.Z.size)

        # print("        Gradients.py: n: {}".format(n))

        self.alpha = alpha

        # self.xs = x
        # self.ys = y

        self.Z = self.apply_fft_method(self.Z, alpha, yLen, xLen,
                                       #    ell=1,
                                       #    sigma_f=1.0
                                       ).real

        return

    def Pk2(self, kx, ky, alpha):
        """
        """
        if kx == 0 and ky == 0:
            return 0.0
        return np.sqrt((np.sqrt(kx**2 + ky**2))**alpha)
        # return self.k2D_alt(kx, ky, ell=5.1, sigma_f=1.0)

    # def K_rbf(self, x, y, ell, sigma_f):
    #     """
    #     """
    #     return sigma_f**2 * np.exp(-((x - y)/ell)**2/2)

    # def k2D(yxi, yxj, ell_1, ell_2, sigma_f):
    #     yi = yxi[0]
    #     xi = yxi[1]

    #     yj = yxj[0]
    #     xj = yxj[1]

    #     return sigma_f**2 * np.exp((-((xi - xj)/ell_1)**2/2) +
    #                             (-((yi - yj)/ell_2)**2/2))

    def calc_kern_2D(self, y_i, x_j, ell, sigma_f):
        K = np.zeros((y_i.shape[0], x_j.shape[0]))
        for i in range(y_i.shape[0]):
            for j in range(x_j.shape[0]):
                # K[i][j] = k2D(y_i[i], x_j[j], ell, ell, sigma_f)
                K[i][j] = self.K_rbf(y_i[i], x_j[j], ell, sigma_f)
        return K

    def apply_fft_method(self, Z, alpha, ylen, xlen):  # , ell, sigma_f):
        """
        """
        noise = np.fft.fft2(Z)
        amplitude = np.zeros((Z.shape))
        # amplitude = self.calc_kern_2D(self.ys, self.xs, ell, sigma_f)
        # print("amplitude.shape: {}".format(amplitude.shape))

        for i, ky in enumerate(self.fftIndgen(ylen)):
            for j, kx in enumerate(self.fftIndgen(xlen)):
                k = self.Pk2(kx, ky, alpha)
                amplitude[i, j] = k

        return np.fft.ifft2(noise * amplitude)

    def fftIndgen(self, n):
        """
        """
        a = list(range(0, int(np.floor(n/2+1))))
        b = list(range(1, int(np.ceil(n/2))))
        b.reverse()
        b = [-i for i in b]
        return a + b

    def applyConvolutions(self, kN=CNST.kN):
        """
        Finish making the field. We don't want to apply the expensive 
        convolution at start, rather only when we need it.
        :param kN: Number of times to convovle kernel
        """
        for i in range(0, kN):
            self.Z = self.convolve(self.Z, self.K)

        self.Z = self.scale(self.Z)

    def GaussianKernel(self):
        """
        def GuassianKernel(self)
        :return: K, The Gaussian kernel for convolution
        """
        # Esnure an odd and equal width and height dimension for the kernel
        if (np.mod(self.kSize, 2) == 0.0):
            self.kSize = self.kSize + 1

        self.kMu = np.floor(self.kSize/2)

        K = np.zeros((self.kSize, self.kSize))
        sum = 0

        for row in range(0, self.kSize):
            for col in range(0, self.kSize):
                K[row, col] = self.Gaussian2D(col, row)
                sum += K[row, col]

        K /= sum

        return K

    def Gaussian2D(self, x, y):
        """
        def Gaussian2D(self, x, y):
        :param x: The x-dimension input 
        :param y: The x-dimension input 
        :return: g, The 2D-Gaussian evaluated at x and y
        """
        g = (1/(2*np.pi*(self.kSigma**2)))*np.exp(-(((x-self.kMu)**2)
                                                    + ((y-self.kMu)**2))
                                                  / (2*(self.kSigma**2)))
        return g

    def convolve(self, Z, K):
        """
        def convolve(self):
        :param Z: A GRF to convolve with the kernal
        :param K: A kernal to convolve with the GRF
        :return: C, The resulting convolution
        """
        fm, fn = Z.shape
        C = np.zeros((fm, fn))
        km, kn = K.shape

        if km != kn:
            ValueError('Kernel is not square')

        c = int(np.floor(km/2))

        for p in range(0, fm):
            for q in range(0, fn):
                for u in range(0, km):
                    for v in range(0, kn):
                        if ((p+u-c) < fm) and ((q+v-c) < fn) \
                                and (((p+u-c) >= 0) and ((q+v-c) >= 0)):
                            C[p, q] += K[u, v]*Z[p+u-c, q+v-c]
        return C

    def scale(self, Z):
        """
        def scale(self):
        Scale the GRF to be within the mMax and mMin range
        :param Z: A GRF to scale
        :return: Z, the scaled GRF
        """
        Z -= np.min(Z)
        Z /= np.abs(np.max(Z))-np.abs(np.min(Z))
        Z *= (self.mMax-self.mMin)
        Z += self.mMin

        return Z

    def getZ(self):
        return self.Z

    def getX(self):
        return self.X

    def getY(self):
        return self.Y

    def get_max(self):
        return self.mMax

    def get_min(self):
        return self.mMin

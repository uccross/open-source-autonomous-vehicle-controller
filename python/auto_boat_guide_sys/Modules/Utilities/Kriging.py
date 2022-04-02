"""
.. module:: Kriging.py
    :platform: MacOS, Unix, Windows,
    :synopsis: A class for spatial estimation of a field, using techniques such
    as Ordinary Kriging
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import numpy as np
from scipy.sparse.data import _minmax_mixin
from ..Constants import Constants as CONST
from ..Gradients import Gradients as GD
from ..Field import Field as Field
import copy
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


class OrdinaryKriging():
    """
    Ordinary Kriging for spatial estimation
    """

    def __init__(self, d_lag=CONST.d_lag, tol=CONST.tol, lag_max=CONST.lag_max,
                 lag_vec=CONST.lag_vec, nLags=CONST.nLags, sill=CONST.sill,
                 ise=CONST.ise, sill_eff=CONST.sill_eff, nugget=CONST.nugget,
                 range2a=CONST.range2a, C=CONST.C,
                 field=Field.Field()):
        """
        :param d_lag: Distance increments for calculating the semivariogram
        :param tol: The tolerance for the lags AKA bins
        :param lag_max: The maximum distance for creating the semivariogram
        :param lag_vec: All lags
        :param nLags: the nuber of lags AKA bins
        :param sill: The sill
        :param ise: The index of the efCONSTtive sill
        :param sill_eff: The efCONSTtive sill
        :param nugget: The nugget
        :param range2a: The range squared, multiplied by a seeminlyg arbitrary
        scalar
        :param C: The covariance matrix
        :return: None
        """
        self.d_lag = d_lag
        self.tol = tol
        self.lag_max = lag_max
        self.nLags = nLags
        self.E = np.zeros((self.nLags, 1))
        self.lag_vec = lag_vec
        self.ise = ise
        self.sill = sill
        self.sill_eff = sill_eff
        self.nugget = nugget
        self.range2a = range2a
        self.C = C
        self.Cinv = np.zeros(C.shape)
        self.initInvert = False
        self.minNumObsv = 2
        self.lastNumO = 0
        self.field = field
        self.Zhat = np.zeros((self.field.yLen, self.field.xLen))
        self.Vzh = np.zeros((self.field.yLen, self.field.xLen))
        self.eta_vec = np.zeros((self.field.yLen*self.field.xLen, 1))
        self.lambda_vec_list = []
        # self.dist_vec = np.zeros((self.field.yLen*self.field.xLen, 1))
        self.scaling_tolerance = 0.001

        return

    def predict(self, C, sensorLog=np.array([[0.0, 0.0, 0.0]])):
        """
        IMPORTANT: if an Empirical semivariogram has been constructed AND
        a variogram model has been fit to the empriical semivariogram, then
        and ONLY then should this function be called! This version of predict
        purposefully does NOT use an iterative matrix inverse method, for 
        performance comparisons.
        :param C: The covariance matrix
        :param sensorLog: A matrix with coloumns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :return: The field estimate, Zhat, and the variance matrix, Vzh
        """
        numO = sensorLog.shape[0]
        d = np.zeros((numO, 1))
        ones = np.ones((numO, 1))
        onePz = np.concatenate((np.ones((1, numO)), np.array([[0.0]])), axis=1)

        # Clear the field estimate
        self.Zhat = np.zeros((self.field.yLen, self.field.xLen))

        if numO > self.minNumObsv:
            self.calcCovMatFromVario(sensorLog)

            self.Cinv = np.linalg.inv(C)

            i_eta = 0
            for u in range(0, self.field.yLen):
                for v in range(0, self.field.xLen):
                    for i in range(0, numO):
                        si = sensorLog[i, 1:3]
                        pi = np.array([[self.field.x[v], self.field.y[u]]])
                        h = np.linalg.norm(si - pi)
                        d[i] = self.variogramGauss(h)

                    lePack = np.concatenate((self.Cinv, ones), axis=1)
                    lePack = np.concatenate((lePack, onePz), axis=0)

                    dp1 = np.concatenate((d, np.array([[1.0]])), axis=0)

                    lePack = np.matmul(lePack, dp1)

                    lmbda = lePack[0:numO]

                    # adding this step fixes the strange normalization below, but isn't listed in the literature, may be a bug somwhere?
                    # lmbda /= np.sum(lmbda)

                    eta = np.array([[lePack[-1][0]]])

                    # print(eta)
                    # self.lambda_vec_list.append(copy.deepcopy(lmbda))
                    self.eta_vec[i_eta][0] = copy.deepcopy(eta[0])

                    i_eta += 1

                    self.Zhat[u, v] = np.matmul(np.transpose(
                        sensorLog[:, [0]]), lmbda)

                    lambdaEta = np.concatenate((lmbda, eta), axis=0)

                    self.Vzh[u, v] = np.matmul(np.transpose(dp1), lambdaEta)
            self.Vzh = np.abs(self.Vzh)

        # Normalize matrix, TODO: Figure out how to avoid this step if possible
        self.Zhat -= np.min(self.Zhat)

        delta = (np.max(self.Zhat)-np.min(self.Zhat))

        if np.abs(delta) <= self.scaling_tolerance:
            delta = (self.field.get_max() - self.field.get_min())

        self.Zhat /= delta

        self.Zhat *= (self.field.get_max() - self.field.get_min())
        self.Zhat += self.field.get_min()

        # self.MZ = np.concatenate((self.MZ, np.array([self.Zhat])), axis=0)

        # self.MV = np.concatenate((self.MV, np.array([self.Vzh])), axis=0)

        return self.Zhat, self.Vzh

    def predict_one_point(self, u, v, C, sensorLog=np.array([[0.0, 0.0, 0.0]])):
        """
        :param u: field y-vector coordinate index
        :param v: field x-vector coordinate index
        :param C: The covariance matrix
        :param sensorLog: A matrix with coloumns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :return: The field estimate at a single point self.Zhat[u, v] and the 
        variance associated with that estimated point self.Vzh[u, v]
        """
        numO = sensorLog.shape[0]
        d = np.zeros((numO, 1))
        distance_vec = np.zeros((numO, 1))
        ones = np.ones((numO, 1))
        onePz = np.concatenate((np.ones((1, numO)), np.array([[0.0]])), axis=1)

        self.Cinv = np.linalg.inv(C)

        for i in range(0, numO):
            si = sensorLog[i, 1:3]
            pi = np.array([[self.field.x[v], self.field.y[u]]])
            h = np.linalg.norm(si - pi)
            distance_vec[i] = copy.deepcopy(h)
            d[i] = self.variogramGauss(h)

        d_out = copy.deepcopy(d)

        lePack = np.concatenate((self.Cinv, ones), axis=1)
        lePack = np.concatenate((lePack, onePz), axis=0)

        dp1 = np.concatenate((d, np.array([[1.0]])), axis=0)

        lePack = np.matmul(lePack, dp1)

        lmbda = lePack[0:numO]
        lmbda /= np.sum(lmbda)
        eta = np.array([[lePack[-1][0]]])

        # print(eta)
        lmbda_out = copy.deepcopy(lmbda)

        self.Zhat[u, v] = np.matmul(np.transpose(
            sensorLog[:, [0]]), lmbda)

        lambdaEta = np.concatenate((lmbda, eta), axis=0)

        self.Vzh[u, v] = np.matmul(np.transpose(dp1), lambdaEta)

        return self.Zhat[u, v], self.Vzh[u, v], lmbda_out, d_out, distance_vec, eta

    def predict1Step(self, C, Cinv, field=Field.Field(),
                     sensorLog=np.array([[0.0, 0.0, 0.0]])):
        """
        IMPORTANT: if an Empirical semivariogram has been constructed AND
        a variogram model has been fit to the empriical semivariogram, then
        and ONLY then should this function be called!
        :param C: The covariance matrix
        :param Cinv: The covariance matrix inverse
        :param field: a field object
        :param sensorLog: A matrix with coloumns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :return: The field estimate, Zhat, and the variance matrix, Vzh
        """
        numO = sensorLog.shape[0]

        # self.lastNumO = numO
        d = np.zeros((numO, 1))
        ones = np.ones((numO, 1))
        onePz = np.concatenate((np.ones((1, numO)), np.array([[0.0]])), axis=1)

        # Clear the field estimate
        self.Zhat = np.zeros((field.yLen, field.xLen))

        if numO > self.minNumObsv:

            if (self.initInvert is False) or (Cinv.shape[0] == 0):
                Cinv = np.linalg.inv(C)
                self.initInvert = True
                # print('----- first inv')
            else:
                # print('----- iterative inv')

                # Find how many more observations were made
                m = numO - Cinv.shape[0]
                # print('Cinv.shape[0]={}'.format(Cinv.shape[0]))
                # print('numO={}'.format(numO))
                # print('m={}'.format(m))
                alpha = np.array([C[0:-m, -m:]])[0]
                alphaT = np.transpose(alpha)
                # print('alpha.shape={}'.format(alpha.shape))
                # print('alphaT.shape={}'.format(alphaT.shape))
                # print('Cinv.shape={}'.format(Cinv.shape))

                beta = C[-1, -1]

                aTCinv = alphaT @ Cinv
                Cinva = Cinv @ alpha
                # print('aTCinv.shape={}'.format(aTCinv.shape))
                # print('Cinva.shape={}'.format(Cinva.shape))

                s = beta - (aTCinv @ alpha)
                # print('s.shape={}'.format(s.shape))

                if s.shape[0] != 0:
                    if np.linalg.det(s) == 0:
                        s = np.eye(s.shape[0])

                    sInv = np.linalg.inv(s)
                # if s == 0:
                #     s = np.array([[1.0]])

                topleft = Cinv + (Cinva @ sInv @ aTCinv)

                topright = -Cinva @ sInv

                botleft = -sInv @ aTCinv

                botright = sInv

                # print('topleft.shape={}'.format(topleft.shape))
                # print('topright.shape={}'.format(topright.shape))
                # print('botleft.shape={}'.format(botleft.shape))
                # print('botright.shape={}'.format(botright.shape))

                top = np.concatenate((topleft, topright), axis=1)
                bot = np.concatenate((botleft, botright), axis=1)

                Cinv = np.concatenate((top, bot), axis=0)
                # print('Cinv.shape={}'.format(Cinv.shape))

                # Catch nans
                Cinv[np.isnan(Cinv)] = 0.0

            self.Cinv = Cinv
            # print('self.Cinv.shape={}'.format(self.Cinv.shape))
            # print('ones.shape={}'.format(ones.shape))

            for u in range(0, field.yLen):
                for v in range(0, field.xLen):
                    for i in range(0, numO):
                        si = sensorLog[i, 1:3]
                        pi = np.array([[field.x[v], field.y[u]]])
                        h = np.linalg.norm(si - pi)
                        d[i] = self.variogramGauss(h)

                    lePack = np.concatenate((self.Cinv, ones), axis=1)
                    lePack = np.concatenate((lePack, onePz), axis=0)

                    dp1 = np.concatenate((d, np.array([[1.0]])), axis=0)

                    lePack = np.matmul(lePack, dp1)

                    lmbda = lePack[0:numO]
                    eta = np.array([[lePack[-1][0]]])

                    self.Zhat[u, v] = np.matmul(np.transpose(
                        sensorLog[:, [0]]), lmbda)

                    lambdaEta = np.concatenate((lmbda, eta), axis=0)

                    self.Vzh[u, v] = np.matmul(np.transpose(dp1), lambdaEta)
            self.Vzh = np.abs(self.Vzh)

        # Normalize field, TODO: Figure out how to avoid this step if possible
        self.Zhat -= np.min(self.Zhat)

        delta = (np.max(self.Zhat)-np.min(self.Zhat))

        if np.abs(delta) <= self.scaling_tolerance:
            delta = (field.get_max() - field.get_min())

        self.Zhat /= delta

        self.Zhat *= (field.get_max() - field.get_min())
        self.Zhat += field.get_min()

        # self.MZ = np.concatenate((self.MZ, np.array([self.Zhat])), axis=0)

        # self.MV = np.concatenate((self.MV, np.array([self.Vzh])), axis=0)

        return self.Zhat, self.Vzh

    def predict1Stepii(self, C, Cinv, field=Field.Field(),
                       sensorLog=np.array([[0.0, 0.0, 0.0]])):
        """
        IMPORTANT: if an Empirical semivariogram has been constructed AND
        a variogram model has been fit to the empriical semivariogram, then
        and ONLY then should this function be called!
        :param C: The covariance matrix
        :param C: The previous covariance matrix inverse
        :param field: a field object
        :param sensorLog: A matrix with coloumns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :return: The field estimate, Zhat, and the variance matrix, Vzh
        """
        numO = sensorLog.shape[0]

        # self.lastNumO = numO
        d = np.zeros((numO, 1))
        ones = np.ones((numO, 1))
        onePz = np.concatenate((np.ones((1, numO)), np.array([[0.0]])), axis=1)

        # Clear the field estimate
        self.Zhat = np.zeros((field.yLen, field.xLen))

        # print('sensorLog.shape={}'.format(sensorLog.shape))

        if numO > self.minNumObsv:

            if self.initInvert is False:
                Cinv = np.linalg.inv(C)
                # print('Cinv={}'.format(Cinv))
                # Cinv = np.ones((self.C.shape[0]-1, self.C.shape[1]-1))
                self.initInvert = True
                # print("-------------------2")

            else:
                # print('numO={}'.format(numO))
                # print('self.initInvert = {}'.format(self.initInvert))
                # alpha = np.reshape(self.C[0:-1, -1], (numO-1, 1))
                alphaT = np.array([C[:, -1]])
                alpha = np.transpose(alphaT)

                # print('C.shape={}'.format(C.shape))
                # print('Cinv.shape={}'.format(Cinv.shape))
                # print('alpha.shape={}'.format(alpha.shape))

                # Cinv = Math.iiFast(Cinv, alpha, alpha)

                u = alpha
                v = alpha

                Cinv = np.concatenate(
                    (Cinv, np.zeros((Cinv.shape[1], 1))), axis=1)
                Cinv = np.concatenate(
                    (Cinv, np.zeros((1, Cinv.shape[0]+1))), axis=0)

                Cinv = Cinv - (Cinv @ u) @ (v.T @ Cinv) / (1 + v.T @ Cinv @ u)

            self.Cinv = Cinv

            for u in range(0, field.yLen):
                for v in range(0, field.xLen):
                    for i in range(0, numO):
                        si = sensorLog[i, 1:3]
                        pi = np.array([[field.x[v], field.y[u]]])
                        h = np.linalg.norm(si - pi)
                        d[i] = self.variogramGauss(h)

                    lePack = np.concatenate((Cinv, ones), axis=1)
                    lePack = np.concatenate((lePack, onePz), axis=0)

                    dp1 = np.concatenate((d, np.array([[1.0]])), axis=0)

                    lePack = np.matmul(lePack, dp1)

                    lmbda = lePack[0:numO]
                    eta = np.array([[lePack[-1][0]]])

                    self.Zhat[u, v] = np.matmul(np.transpose(
                        sensorLog[:, [0]]), lmbda)

                    lambdaEta = np.concatenate((lmbda, eta), axis=0)

                    self.Vzh[u, v] = np.matmul(np.transpose(dp1), lambdaEta)
            self.Vzh = np.abs(self.Vzh)

        # Normalize field, TODO: Figure out how to avoid this step if possible
        self.Zhat -= np.min(self.Zhat)
        self.Zhat /= (np.max(self.Zhat)-np.min(self.Zhat))
        self.Zhat *= (field.mMax - field.mMin)
        self.Zhat += field.mMin

        # self.MZ = np.concatenate((self.MZ, np.array([self.Zhat])), axis=0)

        # self.MV = np.concatenate((self.MV, np.array([self.Vzh])), axis=0)

        return self.Zhat, self.Vzh

    def calcCovMatFromVario(self, sensorLog):
        """
        Calculate the covariance matrix for estimation from the variogram
        :param sensorLog: A matrix with coloumns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :return: none
        """
        numO = sensorLog.shape[0]
        self.C = np.zeros((numO, numO))

        # TODO: *SEE 2!* Calculate the covariance matrix once, maybe break this
        # up

        i = 0
        for row in sensorLog:
            s1 = row[1:3]

            j = 0
            for row in sensorLog:
                s2 = row[1:3]
                dist = np.linalg.norm(s1-s2)
                self.C[i, j] = self.variogramGauss(dist)
                j += 1

            i += 1

        return self.C

    def makeEmpericalSemiVariogram(self, sensorLog, lag_vec=CONST.lag_vec):
        """
        :param sensorLog: A matrix with coloumns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :param vmin: Minimum value able to be measured in field
        :param vmax: Maximum value able to be measured in field
        :param lag_vec: All lags
        :return: The Emperical/Experimental Variogram as a vector, and update
        ise, the index of the efCONSTtive sill
        """
        self.ise = int(np.ceil(0.8*self.nLags))  # TODO: find a better way
        # self.ise = np.argmax(np.abs(self.E))
        firstNegativeFound = False

        # TODO: *SEE 2!* Calculate the covariance matrix once, maybe break this
        # up

        i = 0
        for h in lag_vec:

            numInBin = 0
            sum = 0
            for row in sensorLog:
                s1 = row[1:3]

                z1 = row[0]

                for row in sensorLog:
                    s2 = row[1:3]
                    dist = np.linalg.norm(s1-s2)

                    if ((h-self.tol) <= dist) and (dist <= (h+self.tol)):

                        # print("h-tol: {}, dist: {}, h+tol: {}".format(h-self.tol,
                        #                                               dist,
                        #                                               h+self.tol))

                        z2 = row[0]
                        sum += (abs(z1-z2)**2)
                        numInBin += 1

            # Hold if division by zero
            if (numInBin == 0):
                if i > 0:
                    self.E[i][0] = self.E[i-1][0]
                else:
                    self.E[i][0] = 0.0
            else:
                self.E[i][0] = sum/(2.0*numInBin)

            if i > 0:
                gp = self.E[i][0]
                gm = self.E[i-1][0]

                # Using slope of Empirical Variogram to find the index of the
                # efCONSTtive sill
                if (((gp-gm) < 0) and (firstNegativeFound == False) and
                        (i >= self.ise)):
                    self.ise = i
                    firstNegativeFound = True

            i += 1

        return self.E

    def variogramGauss(self, h):
        """
        :param h: The distance or lag (or really the difference between field
        values at two locations)
        :return: The variance associated with the difference between field
        values at two locations
        """
        return ((self.sill-self.nugget)*(1.0-np.exp(-(h**2)/(self.range2a)))
                + self.nugget)
        # return ((self.sill)*(1.0-np.exp(-(h**2)/(self.range2a)))
        #         + self.nugget)

    def calculateSemiVariogram(self, E, buflen=10,
                               e_min=CONST.min_empirical_index):
        """
        Calculate the semi-variogram from the empirical variogram
        :param E: The Empirical variogram
        :param buflen: The length of the buffer for low-passing the variogram
        :param e_min: The minimum empirical variogram index
        :return: S The empirical semi-variogram and Low-Pass
        """
        LPS = np.zeros(E.shape)
        imax = 0
        # firstPeakFound = False

        shift = int(buflen/2)

        for i in range(0, len(E)):
            if i < buflen:
                LPS[i] = np.mean(E[0:i])
            else:
                LPS[i] = np.mean(E[i-buflen:i])

            # imax = i-shift

            if (i > 0):  # and (firstPeakFound is False):
                if LPS[i] < LPS[i-1]:
                    imax = i-shift
                    # imax = i
                    # firstPeakFound = True

        # print('imax={}'.format(imax))
        # print('LPS[imax]={}'.format(LPS[imax]))

        # Shift the low-pass back to account for phase-shift
        for i in range(0, len(LPS)-shift):
            LPS[i] = LPS[i+shift]

        for i in range(0, len(LPS)):
            if (0 < (i - shift)) and ((i - shift) < len(LPS)):
                if (LPS[i - shift] == LPS[-1]):
                    imax = i
                    break

        if imax < e_min:
            print("    Using minimum empirical distance")
            imax = e_min

        S = E[0:imax]

        return S, LPS

    def GaussianModel(self, h, s, n, r2a):
        """
        :param h: a distance value in meters
        :param s: The sill 
        :param n: The nugget
        :param r2a: The range^2 * a
        """
        return ((s-n)*(1.0-np.exp(-(h**2)/(r2a))) + n)

    def fit2EmpiricalScipy(self, lagvec, E, lag_max=25, sill_min=1, lag_min=10,
                           sill_max=10):
        """
        Wrapper function to fit the empirical variogrm usign scipy
        :param lagvec: mx1 Vector of distances or lags
        :param E: the empirical semi-variogram
        :param lag_max: Maximum lag distance
        :param sill_min: The minimum expected sill
        """
        # The initial guess for the Gaussian model coefficients, s, n, r2a
        p0 = [4.0, 0, 20]
        # print('lagvec.shape = {}'.format(lagvec.shape))
        # print('E.shape = {}'.format(E.shape))
        lagvec = np.reshape(lagvec, (len(lagvec)))
        E = np.reshape(E, (len(E)))
        # print('lagvec.shape = {}'.format(lagvec.shape))
        # print('E.shape = {}'.format(E.shape))
        c, _ = curve_fit(self.GaussianModel, lagvec, E, p0=p0)
        # print('coefficients, c = {}'.format(c))

        self.sill = c[0]
        self.nugget = c[1]
        self.range2a = c[2]

        if self.sill > np.max(E):
            # print("    scipy: self.sill > np.max(E): {}".format(self.sill))
            self.sill = np.max(E)

        # if self.sill > np.max(E):
        #     print("    scipy: self.sill < np.max(E): {}".format(self.sill))
        #     self.sill = sill_max

        # if self.sill < sill_min:
        #     print("    scipy: self.sill < np.max(E): {}".format(self.sill))
        #     # self.sill = sill_min
        #     self.sill = sill_max

        # if self.sill < sill_min:
        #     # print("    scipy: self.sill < np.median(E):{} < {}".format(sill_min,
        #     #                                                        self.sill))
        #     print("    scipy: self.sill: {}".format(self.sill))
        #     self.sill = sill_min

        if self.nugget != 0.0:
            # print("    scipy: nugget != 0.0: {}".format(self.nugget))
            # print("    scipy: self.nugget: {}".format(self.nugget))
            self.nugget = 0.0

        if self.range2a > lag_max:
            # print("    scipy: range2a > lag_max: {}".format(self.range2a))
            # print("    scipy: self.range2a: {}".format(self.range2a))
            self.range2a = lag_max

        # if self.range2a < lag_min:
        #     # print("    scipy: range2a > lag_max: {}".format(self.range2a))
        #     print("    scipy: self.range2a: {}".format(self.range2a))
        #     self.range2a = lag_min

        print("- scipy: self.sill: {}".format(self.sill))
        print("- scipy: self.nugget: {}".format(self.nugget))
        print("- scipy: self.range2a: {}".format(self.range2a))

        return self.sill, self.range2a, self.nugget

    def fit2Empirical(self, lagvec, E, e_min=CONST.min_empirical_lag):
        """
        Fit a semi-variogram model to the Empirical/Expiremntal semi-variogram
        using least-squares. The coefficients for the model are found for the
        fit.  Updates the following coefficients: sill, nugget, and range2a
        :param lagvec: mx1 Vector of distances or lags
        :param E: the empirical semi-variogram
        :param e_min: The minimum empricial lag value
        :return: self.sill, self.range2a, self.nugget
        """
        # print('lagvec.shape={}'.format(lagvec.shape))
        # print('E.shape={}'.format(E.shape))
        h = lagvec[0:len(E)]

        m = h.shape[0]

        H = np.concatenate(
            (h**2, -h**4, h**6, np.min(E)*np.ones((m, 1))), axis=1)

        # Least-squares fit
        # xh, res, rank, s
        xh, _, _, _ = np.linalg.lstsq(H, E, rcond=None)

        self.nugget = xh[-1][0]

        self.range2a = xh[0][0]/(2.0*xh[1][0])

        # self.range2a *= 0.7  # Scaling factor for better fit TODO: get rid of

        self.sill = (self.range2a*xh[0][0]) + self.nugget

        # print('nugget={}'.format(self.nugget))
        # print('range2a={}'.format(self.range2a))
        # print('sill={}'.format(self.sill))

        if self.nugget < 0:
            print("    nugget < 0")
            self.nugget = 0

        if self.nugget != 0:
            print("    nugget != 0")
            self.nugget = 0

        if self.sill < 0:
            print("    sill < 0")
            self.sill = np.abs(self.sill)

        if self.range2a < 0:
            print("    range2a < 0")
            print("    lagvec[-1]*10.0: {}".format(lagvec[-1]*10.0))
            self.range2a = lagvec[-1]

        if self.sill > np.max(E):
            print("    sill > np.max(E) = {}".format(np.max(E)))
            print("    2*np.median(E) = {}".format(2*np.mean(E)))
            self.sill = np.max(E)

            # self.sill = np.max(E)
            print("    corrected sill = {}".format(self.sill))

        # if self.range2a > np.max(lagvec):
        #     print("    range2a > np.max(lagvec) = {}".format(np.max(lagvec)))
        #     self.range2a = np.max(lagvec)*1.2
        #     print("    2*np.median(E) = {}".format(2*np.mean(E)))
        #     self.sill = (2*np.median(E))
        #     print("    corrected sill = {}".format(self.sill))

        if np.isnan(self.nugget):
            print("    Nan case: nugget")
            self.nugget = 0

        if np.isnan(self.range2a):
            print("    Nan case: range2a")
            self.range2a = self.lag_max

        if np.isnan(self.sill):
            print("    Nan case: sill")
            self.sill = 0.8*np.max(E)

        if self.sill < e_min:
            self.sill = e_min

        return self.sill, self.range2a, self.nugget

    def fit2EmpiricalSimple(self, lagvec, E):
        """
        NOTE: USING A VERY SIMPLE FIT CRITERIA
        :return: self.sill, self.range2a, self.nugget
        """

        self.sill = 0.95*np.max(E)
        self.nugget = np.min(E)

        rng = 0
        for lag, e in zip(lagvec, E):
            if e >= self.sill:
                rng = lag[0]
                break

        self.range2a = (rng**2)*0.3

        return self.sill, self.range2a, self.nugget

    def getCovarianceMatrix(self):
        """
        :return: The Covariance Matirx
        """
        return self.C

    def getInvCovarianceMatrix(self):
        """
        :return: The Covariance Matirx
        """
        return self.Cinv

    def getEmpiricalSemiVariogram(self):
        """
        :return: The Empirical/Experimental semi-variogram
        """
        return self.E[0:self.ise, 0]

    def getlag_vec(self):
        """
        :return: All lags, the vector of lags
        """
        return self.lag_vec

    def getSemiLags(self):
        """
        :return: All lags, the vector of lags
        """
        return self.lag_vec[0:self.ise, 0]

    def getFieldEstimate(self):
        """
        :return: The estimated field
        """
        return self.Zhat

    def getVarianceMatrix(self):
        """
        :return: The matrix of variances at each point in the field
        """
        return self.Vzh

    def getSill(self):
        """
        :return: The matrix of variances at each point in the field
        """
        return self.sill

    def getFieldEstimateTensor(self):
        """
        :return: The tensor of field estimate matrices
        """
        return self.MZ

    def get_eta_vec(self):
        return self.eta_vec

    def get_lambda_vec_list(self):
        return self.lambda_vec_list

    def get_range(self):
        return self.range2a

    def set_range(self, rng):
        self.range2a = rng

    # def get_variance_circle_matrix(self, range_radius):
    #     """
    #     :param range_radius: The range from the variogram, being used as a
    #     radius
    #     :return: The
    #     """

    #     return


class PartitionedKriging:
    """
    Partitioned Ordinary Kriging for spatial estimation. The field is divided
    into 4 sub-fields and then those sub-fields may be further divided into
    sub-fields. The idea is to reduce the  size of the covariance matrices
    required for spatial estimation.
    """

    def __init__(self, rlvl=4, C=CONST.C, Zhat=CONST.Zhat, Vhat=CONST.Vzh,
                 d_lag=CONST.d_lag, tol=CONST.tol, lag_max=CONST.lag_max,
                 nLags=CONST.nLags, xMag=CONST.xMagnitude, yMag=CONST.yMagnitude,
                 x0=CONST.x0, xf=CONST.xf, y0=CONST.y0, yf=CONST.yf, ds=CONST.ds,
                 mMax=CONST.mMax, mMin=CONST.mMin):
        """
        :param rlvl: Number of levels of recursion
        :param C: The covariance matrix
        :param Zhat: The field estimate
        :param Vhat: The variance at each point on the field
        """
        self.sill = 0
        self.range2a = 50
        self.nugget = 0

        self.pastSensorLog = np.array([[0.0, 0.0, 0.0]])

        self.field = Field.Field(ds=ds, x0=x0, y0=y0, xf=xf, yf=yf, kSize=1,
                                 mMax=mMax, mMin=mMin)

        self.Zhat = np.zeros(self.field.getZ().shape)
        self.Vhat = np.ones(self.field.getZ().shape)*10000
        self.C = C
        self.Cinv = C

        self.d_lag = d_lag
        self.tol = tol
        self.lag_max = lag_max
        self.nLags = nLags
        self.E = np.zeros((self.nLags, 1))

        self.rlvl = rlvl
        self.divnum = 2**self.rlvl  # Number of partitions
        self.estThresh = self.rlvl - 1  # Sub-field estimate threshold

        # The smallest x and y-dimension of a sub-field
        self.mindivx = xMag/(self.divnum)
        self.mindivy = yMag/(self.divnum)

        self.xmid = np.mean(np.arange(x0, xf, ds))
        self.ymid = np.mean(np.arange(y0, yf, ds))

        # The x and y-dimension sub-field partition coordinates
        self.xdivs = np.arange(x0, xf + self.mindivx, self.mindivx)
        self.ydivs = np.arange(y0, yf + self.mindivy, self.mindivy)

        self.mMax = mMax
        self.mMin = mMin
        self.scaling_tolerance = 0.001

        self.minNumObsv = 2

        self.x0 = x0
        self.y0 = y0
        self.xf = xf
        self.yf = yf
        self.ds = ds

    def predict(self, C, sensorLog, endlevel=1, x_field=CONST.x,
                y_field=CONST.y):
        """
        :param C: Covariance matrix
        :param sensorLog: A matrix with columns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :param endlevel:
        :return: Total field estimate and field variance
        """
        sensorLogCurrQuad, x, y = self.isolatePointsInQuadrant(
            sensorLog=sensorLog, endlevel=endlevel)

        # print("    x.shape: {}".format(x.shape))
        # print("    y.shape: {}".format(y.shape))

        n = len(sensorLogCurrQuad)

        # if (n > self.minNumObsv):
        ZhatQ, VhatQ = self.predictSubField(C, x, y,
                                            sensorLog=sensorLogCurrQuad,
                                            sill=self.sill,
                                            range2a=self.range2a,
                                            nugget=self.nugget)

        # Add sub-field estiamte to the main field estimate
        m, n = ZhatQ.shape
        yoffset = np.where(y_field == y[0])[0][0]
        xoffset = np.where(x_field == x[0])[0][0]
        for row in range(0, m):
            for col in range(0, n):
                self.Zhat[row+yoffset, col+xoffset] = ZhatQ[row, col]
                self.Vhat[row+yoffset, col+xoffset] = VhatQ[row, col]
        # else:
        #     for row in sensorLog:
        #         if list(row) not in self.pastSensorLog.tolist():
        #             my, mx = self.field.quantizePosition(row[2], row[1])
        #             u = np.where(CONST.y == my)[0][0]
        #             v = np.where(CONST.x == mx)[0][0]
        #             self.Zhat[u, v] = row[0]

        self.pastSensorLog = sensorLog

        return self.Zhat, self.Vhat

    def predictSubField(self, C, x, y, sensorLog, a0=0.0, a1=0.0, a2=0.0, a3=0.0,
                        a4=0.0, sill=0.0, range2a=0.0, nugget=0.0,
                        mode='gaussian'):
        """
        IMPORTANT: if an Empirical semivariogram has been constructed AND
        a variogram model has been fit to the empriical semivariogram, then
        and ONLY then should this function be called!
        :TODO: Add one-step update that is more efficient
        :param C: Covariance matrix
        :param x: vector of x-coordinates of field, *or* sub-field
        :param y: vector of y-coordinates of field, *or* sub-field
        :param sensorLog: A matrix with columns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :param a0: Polynomial model constant
        :param a1: Polynomial model constant
        :param a2: Polynomial model constant
        :param a3: Polynomial model constant
        :param a4: Polynomial model constant
        :param sill:
        :param range2a:
        :param nugget:
        :param mode: The variogram model, either 'gaussian' or 'polynomial'
        :return: Zhat The field estimate matrix, Vhat the variance matrix
        """
        numO = sensorLog.shape[0]
        # print("numO={}".format(numO))
        d = np.zeros((numO, 1))
        ones = np.ones((numO, 1))
        onePz = np.concatenate((np.ones((1, numO)), np.array([[0.0]])), axis=1)

        xlen = len(x)
        ylen = len(y)

        # Clear the field estimate
        Zhat = np.zeros((ylen, xlen))
        Vhat = np.ones((ylen, xlen))

        if numO > self.minNumObsv:

            # TODO: Add one-step update that is more efficient
            Cinv = np.linalg.inv(C)

            for u in range(0, ylen):
                for v in range(0, xlen):
                    for i in range(0, numO):
                        si = sensorLog[i, 1:3]
                        pi = np.array([[x[v][0], y[u][0]]])
                        h = np.linalg.norm(si - pi)

                        if mode == 'gaussian':
                            d[i] = self.variogramGauss(h)
                        elif mode == 'polynomial':
                            d[i] = self.varioPolynomial(h, a0, a1, a2, a3, a4)

                        else:
                            d[i] = self.variogramGauss(h)

                    lePack = np.concatenate((Cinv, ones), axis=1)
                    lePack = np.concatenate((lePack, onePz), axis=0)

                    dp1 = np.concatenate((d, np.array([[1.0]])), axis=0)

                    lePack = np.matmul(lePack, dp1)

                    lmbda = lePack[0:numO]
                    eta = np.array([[lePack[-1][0]]])

                    Zhat[u, v] = np.matmul(np.transpose(
                        sensorLog[:, [0]]), lmbda)

                    lambdaEta = np.concatenate((lmbda, eta), axis=0)

                    Vhat[u, v] = np.matmul(np.transpose(dp1), lambdaEta)
            Vhat = np.abs(Vhat)

        # Normalize field, TODO: Figure out how to avoid this step if possible
        # mMax = np.min([np.max(sensorLog[:, 0]), self.mMax])
        # mMin = np.max([np.min(sensorLog[:, 0]), self.mMin])

        Zhat -= np.min(Zhat)

        delta = np.abs(np.max(Zhat) - np.min(Zhat))

        if np.abs(delta) <= self.scaling_tolerance:
            delta = (self.mMax - self.mMin)

        Zhat /= delta

        Zhat *= (self.mMax - self.mMin)
        Zhat += self.mMin

        return Zhat, Vhat

    def isolatePointsInQuadrant(self, sensorLog, endlevel=1):
        """
        :param sensorLog: A matrix with coloumns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :param endlevel: End level for recursion and thus field divisions
        """
        latestPoint = self.field.quantizePosition(sensorLog[-1, 2],
                                                  sensorLog[-1, 1])

        [x0, xf, y0, yf] = self.getQuadrant(latestPoint, endlevel)

        x = np.arange(x0, xf+self.ds, self.ds)
        x = np.reshape(x, (len(x), 1))
        y = np.arange(y0, yf+self.ds, self.ds)
        y = np.reshape(y, (len(y), 1))

        # Boundary Solution
        bnds = (2*self.ds)
        if (x0 - bnds) > self.x0:
            x0 -= bnds
        if (xf + bnds) < self.xf:
            xf += bnds

        if (y0 - bnds) > self.y0:
            y0 -= bnds
        if (yf + bnds) < self.yf:
            yf += bnds

        sensorLogCurrQuad = np.zeros((1, 3))

        i = 0
        for row in sensorLog:
            # print("    row: {}".format(row))
            if self.isPositionInQuadrant(row[-2:], x0, xf, y0, yf):
                if i == 0:
                    sensorLogCurrQuad[0][0] = row[0]
                    sensorLogCurrQuad[0][1] = row[1]
                    sensorLogCurrQuad[0][2] = row[2]
                else:
                    sensorLogCurrQuad = np.concatenate(
                        (sensorLogCurrQuad, np.array([row])), axis=0)
                i += 1
        return sensorLogCurrQuad, x, y

    def calcCovMatFromVario(self, sensorLog, a0=0.0, a1=0.0, a2=0.0, a3=0.0,
                            a4=0.0,
                            mode='gaussian', endlevel=1):
        """
        Calculate the covariance matrix for estimation from the variogram
        :param sensorLog: A matrix with coloumns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :param a0: Polynomial model constant
        :param a1: Polynomial model constant
        :param a2: Polynomial model constant
        :param a3: Polynomial model constant
        :param a4: Polynomial model constant
        :param sill:
        :param range2a:
        :param nugget:
        :param mode: The variogram model, either 'gaussian' or 'polynomial'
        :param endlevel: End level for recursion and thus field divisions
        :return: none
        """

        sensorLog, _, _ = self.isolatePointsInQuadrant(sensorLog,
                                                       endlevel=endlevel)

        numO = sensorLog.shape[0]
        C = np.zeros((numO, numO))

        # TODO: *SEE 2!* Calculate the covariance matrix once, maybe break this
        # up

        i = 0
        for row in sensorLog:
            s1 = row[1:3]

            j = 0
            for row in sensorLog:
                s2 = row[1:3]
                dist = np.linalg.norm(s1-s2)

                if mode == 'gaussian':
                    C[i, j] = self.variogramGauss(dist)

                elif mode == 'polynomial':
                    C[i, j] = self.varioPolynomial(dist, a0, a1, a2, a3, a4)

                else:
                    C[i, j] = self.variogramGauss(dist)
                j += 1

            i += 1

        return C

    def makeEmpericalSemiVariogram(self, sensorLog, endlevel=1, vmin=CONST.mMin,
                                   vmax=CONST.mMax, lag_vec=CONST.lag_vec):
        """
        :param sensorLog: A matrix with coloumns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :param endlevel: End level for recursion and thus field divisions
        :param vmin: Minimum value able to be measured in field
        :param vmax: Maximum value able to be measured in field
        :param lag_vec: All lags
        :return: The Emperical/Experimental Variogram as a vector, and update
        ise, the index of the efCONSTtive sill
        """

        sensorLog, _, _ = self.isolatePointsInQuadrant(sensorLog,
                                                       endlevel=endlevel)

        i = 0
        for h in lag_vec:
            numInBin = 0
            sum = 0
            for row in sensorLog:
                s1 = row[1:3]

                # # Limit the data acquired
                # if (row[0] < CONST.mMin):
                #     row[0] = CONST.mMin
                # if (row[0] > CONST.mMax):
                #     row[0] = CONST.mMax

                z1 = row[0]

                for row in sensorLog:
                    s2 = row[1:3]
                    dist = np.linalg.norm(s1-s2)

                    if ((h-self.tol) <= dist) and (dist <= (h+self.tol)):
                        # # Limit the data acquired
                        # if (row[0] < CONST.mMin):
                        #     row[0] = CONST.mMin
                        # if (row[0] > CONST.mMax):
                        #     row[0] = CONST.mMax

                        z2 = row[0]
                        sum += (abs(z1-z2)**2)
                        numInBin += 1

            # Hold if division by zero
            if (numInBin == 0):
                self.E[i][0] = self.E[i-1][0]
            else:
                self.E[i][0] = sum/(2.0*numInBin)

            i += 1

        return self.E

    def calculateSemiVariogram(self, E, buflen=10,
                               e_min=CONST.min_empirical_index):
        """
        Calculate the semi-variogram from the empirical variogram
        :param E: The Empirical variogram
        :param buflen: The length of the buffer for low-passing the variogram
        :param e_min: The minimum empirical variogram index
        :return: S The empirical semi-variogram and Low-Pass
        """
        LPS = np.zeros(E.shape)
        imax = 0
        firstPeakFound = False

        shift = int(buflen/2)

        for i in range(0, len(E)):
            if i < buflen:
                LPS[i] = np.mean(E[0:i])
            else:
                LPS[i] = np.mean(E[i-buflen:i])

            # imax = i-shift

            if (i > 0) and (firstPeakFound is False):
                if LPS[i] < LPS[i-1]:
                    imax = i-shift
                    # imax = i
                    firstPeakFound = True

        # print('imax={}'.format(imax))
        # print('LPS[imax]={}'.format(LPS[imax]))

        # Shift the low-pass back to account for phase-shift
        for i in range(0, len(LPS)-shift):
            LPS[i] = LPS[i+shift]

        for i in range(0, len(LPS)):
            if (0 < (i - shift)) and ((i - shift) < len(LPS)):
                if (LPS[i - shift] == LPS[-1]):
                    imax = i
                    break

        if imax < e_min:
            print("    POK Using minimum empirical distance")
            imax = e_min

        S = E[0:imax]

        return S, LPS

    def variogramGauss(self, h):
        """
        :param h: The distance or lag (or really the difference between field
        values at two locations)
        :return: The variance associated with the difference between field
        values at two locations
        """
        return ((self.sill-self.nugget)*(1.0-np.exp(-(h**2)/(self.range2a)))
                + self.nugget)

    def varioPolynomial(self, h, a0, a1, a2, a3, a4):
        """
        A polynomial variogram model
        :param h: The distance or lag (or really the difference between field
        values at two locations)
        """
        return (a0*(h**4) + a1*(h**3) + a2*(h**2) + a3*(h) + a4)

    def fitPolynomial2EmpiricalLS(self, E, lagvec, thresh=8):
        """
        :param E: Empirical variogram
        :param lagvec: The vector of increasing lags Mx1
        """
        Ecopy = copy.deepcopy(E)

        for i in range(0, len(Ecopy)):
            if Ecopy[i] > thresh:
                Ecopy[i] = thresh
        h = lagvec
        n = lagvec.shape[0]
        H = np.concatenate((h**4, h**3, h**2, h, np.ones((n, 1))), axis=1)
        xh, _, _, _ = np.linalg.lstsq(H, Ecopy, rcond=None)
        a0 = xh[0][0]
        a1 = xh[1][0]
        a2 = xh[2][0]
        a3 = xh[3][0]
        a4 = xh[4][0]
        return [a0, a1, a2, a3, a4]

    def GaussianModel(self, h, s, n, r2a):
        """
        :param h: a distance value in meters
        :param s: The sill 
        :param n: The nugget
        :param r2a: The range^2 * a
        """
        return ((s-n)*(1.0-np.exp(-(h**2)/(r2a))) + n)

    def fit2EmpiricalScipy(self, lagvec, E, lag_max=50, lag_min=10):
        """
        Wrapper function to fit the empirical variogrm usign scipy
        :param lagvec: mx1 Vector of distances or lags
        :param lag_max: Maximum lag distance
        :param E: the empirical semi-variogram
        """
        # The initial guess for the Gaussian model coefficients, s, n, r2a
        p0 = [4.0, 0, 20]
        # print('lagvec.shape = {}'.format(lagvec.shape))
        # print('E.shape = {}'.format(E.shape))
        lagvec = np.reshape(lagvec, (len(lagvec)))
        E = np.reshape(E, (len(E)))
        # print('lagvec.shape = {}'.format(lagvec.shape))
        # print('E.shape = {}'.format(E.shape))
        c, _ = curve_fit(self.GaussianModel, lagvec, E, p0=p0)
        # print('coefficients, c = {}'.format(c))

        self.sill = c[0]
        self.nugget = c[1]
        self.range2a = c[2]

        if self.sill < np.max(E):
            self.sill = np.max(E)

        if self.nugget != 0.0:
            # print("    POK scipy: nugget != 0.0: {}".format(self.nugget))
            self.nugget = 0.0

        if self.range2a > lag_max:
            print("    POK scipy: range2a > lag_max: {}".format(self.range2a))
            # self.range2a = lag_max

        if self.range2a < lag_min:
            # print("    scipy: range2a > lag_max: {}".format(self.range2a))
            print("    scipy: self.range2a: {}".format(self.range2a))
            self.range2a = lag_min

        return self.sill, self.range2a, self.nugget

    def fit2Empirical(self, lagvec, E):
        """
        Fit a semi-variogram model to the Empirical/Expiremntal semi-variogram
        using least-squares. The coefficients for the model are found for the
        fit.  Updates the following coefficients: sill, nugget, and range2a
        :param lagvec: mx1 Vector of distances or lags
        :param E: the empirical semi-variogram
        :return: self.sill, self.range2a, self.nugget
        """
        h = lagvec[0:len(E)]

        m = h.shape[0]

        H = np.concatenate(
            (h**2, -h**4, h**6, np.min(E)*np.ones((m, 1))), axis=1)

        # Least-squares fit
        # xh, res, rank, s
        xh, _, _, _ = np.linalg.lstsq(H, E, rcond=None)

        self.nugget = xh[-1][0]

        self.range2a = xh[0][0]/(2.0*xh[1][0])

        self.sill = (self.range2a*xh[0][0]) + self.nugget

        if self.nugget < 0:
            print("    nugget < 0")
            self.nugget = 0

        if self.nugget != 0:
            print("    nugget != 0")
            self.nugget = 0

        if self.sill < 0:
            print("    sill < 0")
            self.sill = np.abs(self.sill)

        if self.range2a < 0:
            print("    range2a < 0")
            self.range2a = np.abs(self.range2a)

        if self.sill > np.max(E):
            print("    sill > np.max(E) = {}".format(np.max(E)))
            print("    2*np.median(E) = {}".format(2*np.mean(E)))
            self.sill = (2*np.median(E))
            print("    corrected sill = {}".format(self.sill))

        # if self.range2a > np.max(lagvec):
        #     print("    range2a > np.max(lagvec) = {}".format(np.max(lagvec)))
        #     self.range2a = np.max(lagvec)*1.2
        #     print("    2*np.median(E) = {}".format(2*np.mean(E)))
        #     self.sill = (2*np.median(E))
        #     print("    corrected sill = {}".format(self.sill))

        if np.isnan(self.nugget):
            print("    Nan case: nugget")
            self.nugget = 0

        if np.isnan(self.range2a):
            print("    Nan case: range2a")
            self.range2a = self.lag_max

        if np.isnan(self.sill):
            print("    Nan case: sill")
            self.sill = 0.8*np.max(E)

        if self.sill < CONST.min_empirical_lag:
            self.sill = CONST.min_empirical_lag

        return self.sill, self.range2a, self.nugget

    # def fit2Empirical(self, E, maxlag):
    #     """
    #     NOTE: USING A VERY SIMPLE FIT CRITERIA FOR NOW - NOT USING LS
    #     Fit a variogram model to the Empirical/Expiremntal variogram using
    #     least-squares. The coefficients for the model are found for the fit.
    #     Updates the following coefficients: sill, nugget, and range2a
    #     # :param x: Mx1 vector of x-coordinates of field, *or* sub-field
    #     # :param y: Nx1 vector of y-coordinates of field, *or* sub-field
    #     :param E: Empirical variogram
    #     :param
    #     :return: sill, range2a, nugget
    #     """

    #     self.sill = 0.8*np.max(E)

    #     # Maximum diagonal distance
    #     # range2a = np.sqrt((x[-1][0]-x[0][0])**2 + (y[-1][0]-y[0][0])**2)

    #     self.range2a = maxlag*np.sqrt(len(E))

    #     self.nugget = np.min(E)

    #     return self.sill, self.range2a, self.nugget

    def fit2EmpiricalLS(self, E, lagvec):
        """
        :param E: Empirical variogram
        :param lagvec: The vector of increasing lags Mx1
        """
        h = lagvec
        n = lagvec.shape[0]

        self.nugget = np.min(E)

        # H = np.concatenate((h**2, -(h**4), np.ones((n, 1))), axis=1)
        H = np.concatenate((h**2, -(h**4), h**6, np.ones((n, 1))), axis=1)
        # Least-squares fit
        # xh, res, rank, s
        xh, _, _, _ = np.linalg.lstsq(H, E, rcond=None)

        self.nugget = xh[-1][0]

        # print("xh={}".format(xh))

        range2a = xh[0][0]/(2.0*xh[1][0])

        self.sill = (range2a*xh[0][0])-self.nugget

        return self.sill, self.range2a, self.nugget

    def isEstimatable(self, positions, endlevel=CONST.lowestLevel):
        """
        :param positions: np.array([east:, north:]) a Mx2
        matrix
        :param endlevel: the recursion-end level to stop recurring at (0 is
        the top-most level, and 1, 2, 3, ... m, represent  the subsequent lower
        levels)
        :return: True or False
        """
        count = 0
        firstOb = positions[0, :]

        # print("self.getQuadrant({}, {}) = {}".format(firstOb, endlevel,
        #                                              self.getQuadrant(firstOb, endlevel)))

        Q0 = self.getQuadrant(firstOb, endlevel)

        positions = positions[1:]

        for obs in positions:

            # print("self.getQuadrant({}, {}) = {}".format(obs, endlevel,
            #                                              self.getQuadrant(obs, endlevel)))

            Qi = self.getQuadrant(obs, endlevel)

            subcount = 0

            for a, b in zip(Q0, Qi):
                if a == b:
                    subcount += 1

            # TODO: 4 bc quadrants, but still a magic number, so rename
            if subcount == 4:
                count += 1

        if count == len(positions):
            return True

        return False

    def isPositionInQuadrant(self, position, x0, xf, y0, yf):
        """
        :param position: np.array([[east , north]]) a position vector 1x2
        :param x0:
        :param xf:
        :param y0:
        :param yf:
        """

        if ((x0 <= position[0]) and (position[0] <= xf) and (y0 <= position[1])
                and (position[1] <= yf)):
            return True

        return False

    def partition24(self, x, y):
        """
        Partition a field or sub-field into four sub-fields
        :param x: Mx1 vector of x-coordinates of a field or sub-field
        :param y: Mx1 vector of y-coordinates of a field or sub-field
        :return: 4x4 numpy matrix (array):
                Q = np.array([[q0x0, q0xf, q0y0, q0yf]
                              [q1x0, q1xf, q1y0, q1yf]
                              [q2x0, q2xf, q2y0, q2yf]
                              [q3x0, q3xf, q3y0, q3yf]])

                quadrants in physical space:
                q3 | q2
                --- ---
                q0 | q1
        """
        xn = len(x)-1
        ym = len(y)-1

        ixhalf = int(np.floor(xn/2))
        iyhalf = int(np.floor(ym/2))

        Q = np.zeros((4, 4))

        # Quadrant 0
        Q[0][0] = x[0][0]  # q0x0
        Q[0][1] = x[ixhalf]  # q0xf
        Q[0][2] = y[0][0]  # q0y0
        Q[0][3] = y[iyhalf]  # q0yf

        # Quadrant 1
        Q[1][0] = x[ixhalf+1][0]  # q1x0
        Q[1][1] = x[-1][0]  # q1xf
        Q[1][2] = y[0][0]  # q1y0
        Q[1][3] = y[iyhalf]  # q1yf

        # Quadrant 2
        Q[2][0] = x[ixhalf+1][0]  # q2x0
        Q[2][1] = x[-1]  # q2xf
        Q[2][2] = y[iyhalf+1][0]  # q2y0
        Q[2][3] = y[-1]  # q2yf

        # Quadrant 3
        Q[3][0] = x[0][0]  # q3x0
        Q[3][1] = x[ixhalf][0]  # q3xf
        Q[3][2] = y[iyhalf+1][0]  # q3y0
        Q[3][3] = y[-1][0]  # q3yf

        return Q

    def getQuadrant(self, position, endlevel=CONST.lowestLevel):
        """
        :param position: np.array([[east , north]]) a position vector 1x2
        :param endlevel: the recursion-end level to stop recurring at (0 is
        the top-most level, and 1, 2, 3, ... m, represent  the subsequent lower
        levels)
        :return: The x and y coordinates of the quadrant
        """

        # binary search in x for minimum x partition coordinate
        x0, xf = self.binarySearch(position[0], 0,
                                   len(self.xdivs)-1,
                                   self.xdivs, 0, endlevel)

        # binary search in y for minimum y partition coordinate
        y0, yf = self.binarySearch(position[1], 0,
                                   len(self.ydivs)-1,
                                   self.ydivs, 0, endlevel)

        y0, x0 = self.field.quantizePosition(y0, x0)
        yf, xf = self.field.quantizePosition(yf, xf)

        return [x0, xf, y0, yf]

    def binarySearch(self, v, il, ir, divs, n, endlevel=CONST.lowestLevel):
        """
        binary search
        :param v: value to be searched with
        :param il: left index
        :param ir: right index
        :param divs: An np.array([x0, x1, x2, ..., xN-1]) representing the
        spatial divisions of a field in either the x or y-dimension.
        :param n: the current level of recursion
        :param endlevel: the recursion-end level to stop recurring at (0 is
        the top-most level, and 1, 2, 3, ... m, represent  the subsequent lower
        levels)
        :return: divs[il], divs[ir]
        """

        # Termination, or "Bottom-out" case
        if n >= endlevel:
            return [divs[il], divs[ir]]

        i = round((il+ir)/2)

        # bias towards left
        if v <= divs[i]:
            ir = i

        else:
            il = i

        return self.binarySearch(v, il, ir, divs, n+1, endlevel)

    def makeEmpericalVariogram(self, tol, lagvec, sensorLog, thresh=2.0):
        """
        :param tol: The tolerance for the lag/bins
        :param lagvec: A vector of lags/bins
        :param sensorLog: A matrix with coloumns having the following
        designation:
                    measurement,    East position,  North Position
        np.array([[ :,              :,              :               ]])
        :param thresh: A threshold to detect outliers
        :return: [sillIndex The index of the sill in the emperical variogram,
        E The emperical variogram]
        """
        m = len(lagvec)
        sillIndex = 0
        firstNegativeFound = False

        E = np.zeros((len(lagvec), 1))

        # TODO: *SEE 2!* Calculate the covariance matrix once, maybe break this
        # up

        i = 0
        for h in lagvec:  # TODO: change this to be changeable
            numInBin = 0
            sum = 0
            # print("h={}".format(h))
            for row in sensorLog:
                s1 = row[1:3]
                z1 = row[0]

                for row in sensorLog:
                    s2 = row[1:3]
                    dist = np.linalg.norm(s1-s2)

                    if ((h-tol) <= dist) and (dist <= (h+tol)):

                        z2 = row[0]
                        sum += (abs(z1-z2)**2)
                        numInBin += 1

            # Hold if division by zero
            if (numInBin == 0):
                E[i][0] = E[i-1][0]
            else:
                E[i][0] = sum/(2.0*numInBin)

            if i > 0:
                gp = E[i][0]
                gm = E[i-1][0]

                # # Detect ourliers
                # if abs(gp-gm) > thresh:
                #     E[i][0] = E[i-1][0]

                # Using slope of Empirical Variogram to find the index of the
                # efCONSTtive sill
                if ((firstNegativeFound == False) and (i >= sillIndex)):
                    sillIndex = i

                if (gp-gm) < 0:
                    firstNegativeFound = True

            i += 1

        return [sillIndex, E]

    def getFieldEstimate(self):
        """
        :return: The estimated field
        """
        return self.Zhat

    def getVarianceMatrix(self):
        """
        :return: The matrix of variances at each point in the field
        """
        return self.Vhat

    def getCovarianceMatrix(self):
        """
        :return: The Covariance Matirx
        """
        return self.C

    def getInvCovarianceMatrix(self):
        """
        :return: The Covariance Matirx
        """
        return self.Cinv

    def get_range(self):
        return self.range2a

    def get_sill(self):
        return self.sill

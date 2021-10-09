"""
.. module:: Quaterions.py
    :platform: MacOS, Unix, Windows,
    :synopsis: Quaternions for rotating vectors, and doing attitude estimation. 
    This is based off the excellent book: "Quaternions and Rotation Sequnces: 
    A Primer with Applications to Orbits Aerospace, and Virtual Reality," by 
    Jack B. Kuipers
    Quaternions are treated as 4x1 np.array([[]])'s
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time
from matplotlib.animation import FFMpegWriter


def setQuaternionEulerAngles(psi, theta, phi):
    """
    :param psi: Yaw angle in radians from -pi to pi
    :param theta: Pitch angle in radians
    :param phi: Roll angle in radians
    :return: A quaternion
    """
    cPsi2 = np.cos(psi/2.0)
    cTheta2 = np.cos(theta/2.0)
    cPhi2 = np.cos(phi/2.0)

    sPsi2 = np.sin(psi/2.0)
    sTheta2 = np.sin(theta/2.0)
    sPhi2 = np.sin(phi/2.0)

    q = np.zeros((4, 1))

    q0 = (cPsi2*cTheta2*cPhi2 + sPsi2*sTheta2*sPhi2)
    q1 = (cPsi2*cTheta2*sPhi2 - sPsi2*sTheta2*cPhi2)
    q2 = (cPsi2*sTheta2*cPhi2 + sPsi2*cTheta2*sPhi2)
    q3 = (sPsi2*cTheta2*cPhi2 - cPsi2*sTheta2*sPhi2)

    q[0, 0] = q0
    q[1, 0] = q1
    q[2, 0] = q2
    q[3, 0] = q3

    return q


def getQuaternionComplexConjugate(q):
    """
    :param q: A quaternion
    :return: The complex congjugate of the input quaternion
    """
    qConj = np.zeros((4, 1))

    qConj[0, 0] = q[0, 0]
    qConj[1, 0] = -q[1, 0]
    qConj[2, 0] = -q[2, 0]
    qConj[3, 0] = -q[3, 0]

    return qConj


def multiplyQuaternions(q, p):
    """
    :param q: A quaternion
    :param p: A quaternion
    :return: A quaternion
    """

    p0 = p[0, 0]
    p1 = p[1, 0]
    p2 = p[2, 0]
    p3 = p[3, 0]

    pMat = np.array([[p0, -p1, -p2, -p3],
                     [p1, p0, p3, -p2],
                     [p2, -p3, p0, p1],
                     [p3, p2, -p1, p0]])

    r = np.matmul(pMat, q)

    return r


def rotateVectorWithQuaternion(v, psi=0.0, theta=0.0, phi=0.0):
    """
    :param v: A vector in R^3 as a 3x1 np.array([[]])
    :param psi: Yaw angle in radians from -pi to pi
    :param theta: Pitch angle in radians
    :param phi: Roll angle in radians
    :return: The rotated vector in R^3 as a 3x1 np.array([[]])
    """

    q = setQuaternionEulerAngles(psi, theta, phi)

    vPure = np.zeros((4, 1))

    vPure[1:4, 0] = v[:, 0]

    qConj = getQuaternionComplexConjugate(q)

    p = multiplyQuaternions(qConj, vPure)

    vNew = multiplyQuaternions(p, q)

    return vNew[1:4]
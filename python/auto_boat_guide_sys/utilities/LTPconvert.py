"""
.. module:: LTPconvert.py
    :platform: MacOS, Unix, Windows,
    :synopsis: For converting LLA to NED
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import numpy as np


def lla2ned2(lla, ref):
    """
    converts latitude, longtitude and altitude coordinates (given in
    degrees and meters) into ECEF coordinates (given in meters).

    Note: Alitude should be negative!!

    Modified for spherical earth - COLB 8/8/01
    :param ref: A reference point for the conversion
    :return: ned, A matrix of North East Down LTP coordinate columns [meters]
    """
    EarthRad = 6378137.0
    d2r = np.pi/180.0

    lla[0, 0] = lla[0, 0]*d2r     # convert to radians
    lla[0, 1] = lla[0, 1]*d2r     # convert to radians
    lla[0, 2] = lla[0, 2]         # convert to meters

    lat = d2r*ref[0, 0]
    lon = d2r*ref[0, 1]
    alt = ref[0, 2]

    # Form rotation matrix

    Te2n = np.array(
        [[-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)],
         [-np.sin(lon), np.cos(lon), 0],
         [-np.cos(lat)*np.cos(lon), -np.cos(lat)*np.sin(lon), -np.sin(lat)]])

    N = len(lla[:, [0]])

    ecef = lla2ecef2(lla)

    ned = np.transpose(np.matmul(Te2n, np.transpose(ecef)))

    ned = ned + (EarthRad + alt)*np.concatenate(
        (np.zeros((N, 2)), np.ones((N, 1))), axis=1)

    return ned


def lla2ecef2(lla):
    """
    converts latitude, longtitude and altitude coordinates (given in
    degrees and meters) into ECEF coordinates (given in meters).

    Note: Alitude should be negative!!   

    Modified for spherical earth - COLB 8/8/01
    :param lla: A matrix of Latitude, Longitude, and Altiude coordinates
    :return: ecef The Earth-Centered-Earth-Fixed coordinates
    """

    N = 1

    EarthRad = 6378137.0

    #ecc2 = ecc*ecc

    sinlat = np.sin(lla[0, 0])
    coslat = np.cos(lla[0, 0])
    #Rn = EarthRad / sqrt(abs(1.0 - (ecc2 * sinlat * sinlat)))
    Rn = EarthRad

    ecef = np.zeros((N,3))

    ecef[:, [0]] = np.multiply(
        np.multiply((Rn - lla[:, [2]]), coslat), np.cos(lla[:,[1]]))
    ecef[:, [1]] = np.multiply(
        np.multiply((Rn - lla[:, [2]]), coslat), np.sin(lla[:,[1]]))
    ecef[:, [2]] = np.multiply((Rn - lla[:, [2]]), sinlat)

    return ecef

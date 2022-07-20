# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 11:14:58 2022

@author: Aaron
"""

from math import radians, cos, sin, asin, sqrt, pi

def calc_earth_radius(lat):
    lat = lat * pi/180
    # radius at equator in km
    r1 = 6378.137
    # radius at the poles in km
    r2 = 6356.752
    R = sqrt( ( (r1**2 *cos(lat))**2 + (r2**2 * sin(lat))**2 )/ ( (r1 * cos(lat))**2 + (r2 * sin(lat))**2) )
    return R

def haversine(lon1, lat1, lon2, lat2, alt = 20):
    """
    Calculate the great circle distance in kilometers between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    # Radius of earth in kilometers from https://rechneronline.de/earth-radius/
    # converted to meters and including optional altitude
    r = 6370.433*1000 + alt
    
    return c * r

# lat = 37
# R = calc_earth_radius(lat)
# print('Accepted value = 6370.433')
# print('Value from calculator: ', R)

# lat1 = 37.001
# lat2 = 37.002
# lon1 = 122.001
# lon2 = 122.001

# d = haversine(lon1, lat1, lon2, lat2, alt=0)
# print('haversine calculation: ', d)

# d_phi = (lat2-lat1) *pi/180
# d_test = R*d_phi
# print('approximation is: ', d_test*1000)

# lat1 = 37.001
# lat2 = 37.001
# lon1 = 122.001
# lon2 = 122.002

# d = haversine(lon1, lat1, lon2, lat2, alt=0)
# print('haversine calculation: ', d)

# d_theta = (lon2-lon1) *pi/180
# d_test = R*d_theta * cos(lat1*pi/180)
# print('approximation is: ', d_test*1000)
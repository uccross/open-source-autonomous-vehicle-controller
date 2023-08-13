# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 11:14:58 2022

@author: Aaron
"""

from math import radians, cos, sin, asin, sqrt, pi

def calc_earth_radius(lat):
    lat = lat * pi/180
    # radius at equator in m
    r1 = 6378137.0
    # radius at the poles in m
    r2 = 6356752.3
    R = sqrt( ( (r1**2 *cos(lat))**2 + (r2**2 * sin(lat))**2 )/ ( (r1 * cos(lat))**2 + (r2 * sin(lat))**2) )
    return R

def haversine(lon1, lat1, lon2, lat2,r = 6370433,  alt = 20):
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
    r = r + alt
    
    return c * r

# london_coord = 51.5073219,  -0.1276474
# cities = {
#     'berlin': (52.5170365,  13.3888599),
#     'vienna': (48.2083537,  16.3725042),
#     'sydney': (-33.8548157, 151.2164539),
#     'madrid': (40.4167047,  -3.7035825) 
# }

# for city, coord in cities.items():
#     distance = haversine(london_coord[1],london_coord[0], coord[1], coord[0])
#     print(city, distance)


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
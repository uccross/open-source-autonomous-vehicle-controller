# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 16:25:36 2022

@author: Aaron
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from haversine import calc_earth_radius, haversine

filename = '.\logfiles\log_20230101-142316.csv'

df_raw = pd.read_csv(filename)
df_raw['time_msec'] = df_raw['time_usec']/1000

df_raw['time_merged'] = np.where(df_raw['time_msec']>df_raw['time_boot_ms'],df_raw['time_msec']/1000,df_raw['time_boot_ms']/1000)
df_raw['time_elapsed'] = df_raw['time_merged'] - df_raw['time_merged'][0]
df_raw['lon2'] = df_raw['lon']*np.pi/(180*1e7)
df_raw['lat2'] = df_raw['lat']*np.pi/(180*1e7)
df_raw['lon1'] = df_raw['lon2'].shift(1)
df_raw['lat1'] = df_raw['lat2'].shift(1)
    # # haversine formula 
    # dlon = lon2 - lon1 
    # dlat = lat2 - lat1 
df_raw['dlat'] = df_raw['lat2']-df_raw['lat1']
df_raw['dlon'] = df_raw['lon2']-df_raw['lon1']
# calc earth radius at LTP
R = calc_earth_radius(df_raw.lat[0]/1e7)
# cal scaling factor for longitude
cos_lat = np.cos(df_raw.lat2[0])
# calculate distances at each step
df_raw['dy_ltp'] = df_raw.dlat * R
df_raw['dx_ltp'] = df_raw.dlon * R *cos_lat
# compite x,y
df_raw['y_ltp'] = df_raw.dy_ltp.cumsum()
df_raw['x_ltp'] = df_raw.dx_ltp.cumsum()

df_raw.plot.scatter(x = 'x_ltp', y='y_ltp')

# plot GPS track
# def plot_GPS(lat, lon):
#     fig, ax = plt.subplots(2)
#     ax[0].scatter()


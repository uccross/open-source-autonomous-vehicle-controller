# -*- coding: utf-8 -*-
"""
Created on Mon Jan  2 11:46:46 2023

@author: Aaron
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from haversine import calc_earth_radius, haversine

filename = '.\logfiles\log_20230101-141730.csv'

df_raw = pd.read_csv(filename)
df_raw['time_msec'] = df_raw['time_usec']/1000

df_raw['time_merged'] = np.where(df_raw['time_msec']>df_raw['time_boot_ms'],df_raw['time_msec']/1000,df_raw['time_boot_ms']/1000)
df_raw['time_elapsed'] = df_raw['time_merged'] - df_raw['time_merged'][0]
df_raw['lon_r'] = df_raw['lon']/1e7
df_raw['lat_r'] = df_raw['lat']/1e7
home = (df_raw['lon_r'][0], df_raw['lat_r'][0])
# calc earth radius at LTP
R = calc_earth_radius(df_raw.lat[0]/1e7)

rows = df_raw.shape[0]
x_ltp = np.zeros(rows)
y_ltp = np.zeros(rows)

for j in range(rows):
    x_ltp[j] = haversine(home[0],home[1],df_raw.lon_r[j],df_raw.lat_r[0],R,20)
    y_ltp[j] = haversine(home[0],home[1],df_raw.lon_r[0],df_raw.lat_r[j],R,20)
    # print(df_raw.x_ltp[j], df_raw.y_ltp[j])




fig, ax = plt.subplots()
ax.scatter(x_ltp, y_ltp)
ax.set_xlabel("Leprechauns")
ax.set_ylabel("Gold")

plt.show()
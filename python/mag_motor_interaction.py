# -*- coding: utf-8 -*-
"""
Created on Wed Mar  2 11:53:18 2022

@author: Aaron
"""

import pandas as pd
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

filename = 'test_mag_motor2.csv'
df_raw = pd.read_csv(filename)
data_df = df_raw[['time_usec','xmag','ymag','zmag']]
df_raw['time_msec'] = df_raw['time_usec']/1000

df_raw['time_merged'] = np.where(df_raw['time_msec']>df_raw['time_boot_ms'],df_raw['time_msec']/1000,df_raw['time_boot_ms']/1000)
df_raw['time_elapsed'] = df_raw['time_merged'] - df_raw['time_merged'][0]


fig,ax = plt.subplots(4)
fig.suptitle('Encoder Interference on Magnetometer')

ax[0].plot(df_raw['time_elapsed'],df_raw['xmag'],label='X')
ax[0].set_xlim(11,16)
ax[0].legend(loc="upper left")
ax[1].plot(df_raw['time_elapsed'],df_raw['ymag'],label='Y')
ax[1].set_xlim(11,16)
ax[1].legend(loc="upper left")
ax[2].plot(df_raw['time_elapsed'],df_raw['zmag'],label='Z')
ax[2].set_xlim(11,16)
ax[2].legend(loc="upper left")
ax[3].plot(df_raw['time_elapsed'],df_raw['chan3_raw'],color='black', label='velocity cmd')
ax[3].set_xlim(11,16)
ax[3].set_xlabel('time [sec]')
ax[3].set_ylim(800,1200)
ax[3].legend(loc="upper left")


fig.show()


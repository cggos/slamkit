#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

# adb pull /storage/emulated/0/cglog_larvio/larvio_log_traj.tum ./

f = open("/home/cg/SynologyDrive/hjimi/larvio_benchmark/logs/larvio_log_traj.tum")

x  = []
bax = []
bay = []
baz = []
bgx = []
bgy = []
bgz = []
ts = 0
n = 0
for line in f:
    data = line.split()
    if n == 0:
        ts = float(data[0])
    x.append((float(data[0])) - ts)
    bax.append(float(data[8]))
    bay.append(float(data[9]))
    baz.append(float(data[10]))
    bgx.append(float(data[11]))
    bgy.append(float(data[12]))
    bgz.append(float(data[13]))
    n = n + 1
        
fig, ax = plt.subplots(2, 1)
fig.suptitle("larvio ba bg")

plt.subplot(211)    
plt.plot(x, bax, 'r', label="bax")
plt.plot(x, bay, 'g', label="bay")
plt.plot(x, baz, 'b', label="baz") 
plt.xlim(xmin=0)
plt.ylabel("ba")  
plt.legend(loc = 'upper right')

plt.subplot(212)
plt.plot(x, bgx, 'r--', label="bgx")
plt.plot(x, bgy, 'g--', label="bgy")
plt.plot(x, bgz, 'b--', label="bgz")       
plt.xlim(xmin=0)
plt.ylabel("bg")
plt.legend(loc = 'upper right')

# plt.xticks(np.arange(min(x), max(x)+1, 10.0))

plt.xlabel("timestamp (s)")

plt.show()

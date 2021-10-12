#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

# adb pull /storage/emulated/0/cglog_larvio/larvio_log_dlta.txt ./

f = open("/home/cg/SynologyDrive/hjimi/larvio_benchmark/logs/larvio_log_dlta.txt")

x  = []
dpx = []
dpy = []
dpz = []
dvx = []
dvy = []
dvz = []
dbax = []
dbay = []
dbaz = []
dbgx = []
dbgy = []
dbgz = []
ts = 0
n = 0
for line in f:
    data = line.split()
    x.append(n)
    dpx.append(float(data[1]))
    dpy.append(float(data[2]))
    dpz.append(float(data[3]))
    dvx.append(float(data[4]))
    dvy.append(float(data[5]))
    dvz.append(float(data[6]))
    dbax.append(float(data[7]))
    dbay.append(float(data[8]))
    dbaz.append(float(data[9]))
    dbgx.append(float(data[10]))
    dbgy.append(float(data[11]))
    dbgz.append(float(data[12]))
    n = n + 1
        
# plt.figure()

plt.plot(x, dbax)
plt.title("delta ba_x")

fig, ax = plt.subplots(2, 2)
fig.suptitle("larvio delta X")

plt.subplot(221)    
plt.plot(x, dpx, 'r', label="bax")
plt.plot(x, dpy, 'g', label="bay")
plt.plot(x, dpz, 'b', label="baz") 
plt.xlim(xmin=0)
plt.ylabel("dp")  
plt.legend(loc = 'upper right')
plt.title("delta P")

plt.subplot(222)
plt.plot(x, dvx, 'r--', label="bgx")
plt.plot(x, dvy, 'g--', label="bgy")
plt.plot(x, dvz, 'b--', label="bgz")       
plt.xlim(xmin=0)
plt.ylabel("dv")
plt.legend(loc = 'upper right')
plt.title("delta V")

plt.subplot(223)    
plt.plot(x, dbax, 'r', label="bax")
plt.plot(x, dbay, 'g', label="bay")
plt.plot(x, dbaz, 'b', label="baz") 
plt.xlim(xmin=0)
plt.ylabel("dp")  
plt.legend(loc = 'upper right')
plt.title("delta ba")

plt.subplot(224)
plt.plot(x, dbgx, 'r--', label="bgx")
plt.plot(x, dbgy, 'g--', label="bgy")
plt.plot(x, dbgz, 'b--', label="bgz")       
plt.xlim(xmin=0)
plt.ylabel("dv")
plt.legend(loc = 'upper right')
plt.title("delta bg")

# plt.xticks(np.arange(min(x), max(x)+1, 10.0))

plt.show()

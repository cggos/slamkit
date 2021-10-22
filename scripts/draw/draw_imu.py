#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import numpy as np
import matplotlib.pyplot as plt

f = open("../data/imu0.csv")

n = 0

tm = []
x1 = []
y1 = []
z1 = []
x2 = []
y2 = []
z2 = []

# for line in f:
#     data = line.split(',')
#     if len(data) != 7:
#         print("wrong data!!! %d" % len(data))
#         sys.exit(1)
#     tm.append( float(data[0] ) * 10e-9 )
#     x1.append( float(data[1] ) )
#     y1.append( float(data[2] ) )
#     z1.append( float(data[3] ) )
#     x2.append( float(data[4] ) )
#     y2.append( float(data[5] ) )
#     z2.append( float(data[6] ) )
#     n += 1
#     if n>100:
#         break
    
while True:
    line0 = f.readline()
    line1 = f.readline()
    if not line0 or not line1:
        break
    data0 = line0.split(',')
    data1 = line1.split(',')
    t0 = float(data0[0] ) * 10e-9;
    t1 = float(data1[0] ) * 10e-9;
    t1 = (t0+t1) * 0.5
    tm.append( t0 )
    tm.append( t1 )
    z2.append( float(data0[6] ) )
    z2.append( 0 )
    n += 1
    if n>50:
        break
    
print(n)

plt.plot(tm, z2)
plt.title("imu acc Z")

plt.show()

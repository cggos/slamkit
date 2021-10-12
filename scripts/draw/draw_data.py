#!/usr/bin/env python
# coding=utf-8

import sys
import numpy as np
import matplotlib.pyplot as plt

f = open("/home/cg/.ros/td_data_larvio.txt")

n = 0

x1 = []
y1 = []
z1 = []
x2 = []
y2 = []
z2 = []

for line in f:
    data = line.split(',')
    if len(data) != 6:
        print("wrong data!!! %d" % len(data))
        sys.exit(1)
    x1.append( float(data[0] ) )
    y1.append( float(data[1] ) )
    z1.append( float(data[2] ) )
    x2.append( float(data[3] ) )
    y2.append( float(data[4] ) )
    z2.append( float(data[5] ) )
    n += 1

x = np.arange(0, n, 1)
interval = 20

plt.subplot(3, 1, 1)
plt.plot(x, x1, 'b--', x, x2, 'r--')
plt.ylabel("x")
plt.xticks(np.arange(min(x), max(x)+1, interval))
plt.title('position plot')

plt.subplot(3, 1, 2)
plt.plot(x, y1, 'b--', x, y2, 'r--')
plt.ylabel("y")
plt.xticks(np.arange(min(x), max(x)+1, interval))

plt.subplot(3, 1, 3)
plt.plot(x, z1, 'b--', x, z2, 'r--')
plt.ylabel("z")
plt.xticks(np.arange(min(x), max(x)+1, interval))
plt.xlabel("time")

plt.show()

#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d

f = open("./groundtruth.txt")
x = []
y = []
z = []
for line in f:
    if line[0] == '#':
        continue
    data = line.split()
    x.append( float(data[1] ) )
    y.append( float(data[2] ) )
    z.append( float(data[3] ) )
ax = plt.subplot( 111, projection='3d')
ax.plot(x,y,z)
plt.show()

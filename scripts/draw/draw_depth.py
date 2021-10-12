#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

# adb pull /storage/emulated/0/larvio_log_time.txt ./

f = open("/home/cg/projects/slam/larvio_android/larvio_log_depth.txt")

x  = []
y0 = []
y1 = []
y2 = []
ye1 = []
ye2 = []
lcn = []
dnorm = []
n = 0
m = 0
ne1 = 0
ne2 = 0
for line in f:
    data = line.split(',')
    init = float(data[0])
    depth = float(data[1])
    twomeas = float(data[2])
    err1 = (depth - init)
    err2 = (twomeas - depth)
    if init == 0 or depth > 2 or depth < 0 or abs(err2) > 10 or twomeas < 0 or twomeas >3.5:
        continue
    if abs(err2) > 0.05:
        ne2 = ne2 + 1
    n = n + 1
    if n % 2 != 0:
        continue
    x.append(m)
    y0.append(init)
    y1.append(depth)
    y2.append(twomeas)
    ye1.append(err1)
    ye2.append(err2)
    lcn.append(float(data[3]))
    dnorm.append(float(data[4]))
    m = m + 1

rate = ne2 / n * 100
print("error rate: %f%%" % rate)

# plt.plot(x, y1, marker = 'x', color = 'b', label='old')
# plt.plot(x, y2, marker = 'o', color = 'r', label='new')

plt.scatter(x, y0, marker = 'o', color = 'g', label='depth init', s = 30)
plt.scatter(x, y1, marker = 'x', color = 'b', label='depth + 2d', s = 20)
plt.scatter(x, y2, marker = 'v', color = 'r', label='only 2d', s = 10)

plt.plot(x, ye1, 'b-', label="depth-init")
plt.plot(x, ye2, 'r--', label="2d-depth")

plt.axhline(y=0)

# plt.xlim(xmin=0)
# plt.ylim(ymin=0)
plt.xticks(np.arange(min(x), max(x)+1, 10.0))

# for xy in zip(x, y):
#     plt.annotate("(%s,%s)" % xy, xy=xy, xytext=(-20, 10), textcoords='offset points')
plt.xlabel("point index")
plt.ylabel("depth")
plt.title("larvio depth (twomeas-depth > 0.05m %f%%)" % rate)

plt.legend(loc = 'upper right')

plt.show()


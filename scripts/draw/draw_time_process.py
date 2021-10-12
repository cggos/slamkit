#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

# adb pull /sdcard/larvio/larvio_log_time.txt ./

f_time = open("/home/cg/projects/slam/larvio_android/larvio_log_time.txt")

x  = []
y1 = []
y2 = []
n = 0
m = 0
for line in f_time:
    n = n + 1
    if n < 5:
        continue
    if n % 2 == 0:
        data = line.split()
        x.append(m)
        y1.append(float(data[0]))
        # y2.append(float(data[1]))
        m = m + 1

# plt.plot(x, y1, 'b--', x, y2, 'r--')

plt.scatter(x, y1, marker = 'x', color = 'b', label='process image', s = 30)
# plt.scatter(x, y2, marker = 'o', color = 'r', label='process features', s = 30)

plt.axhline(y=20)
plt.axhline(y=30)

plt.xlim(xmin=0)
plt.ylim(ymin=0)
plt.xticks(np.arange(min(x), max(x)+1, 10.0))

# for xy in zip(x, y):
#     plt.annotate("(%s,%s)" % xy, xy=xy, xytext=(-20, 10), textcoords='offset points')
plt.xlabel("frame num")
plt.ylabel("time (ms)")
plt.title('larvio process time (ms)')

plt.legend(loc = 'upper right')

plt.show()

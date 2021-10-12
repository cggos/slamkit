#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

f1 = open("/home/cg/.ros/td_data_larvio.txt")
# f2 = open("/home/cg/.ros/td_data_hkvins.txt")

x1 = []
y1 = []
n = 0
m = 0
# for line in f1:
#     n = n + 1
#     # data = line.split()
#     if n < 50:
#         continue
#     if n % 20 == 0:
#         x1.append(m)
#         y1.append(float(line))
#         m = m + 1

for line in f1:
    x1.append(m)
    y1.append(float(line))
    m = m + 1

# x2 = []
# y2 = []
# n = 0
# m = 0
# for line in f2:
#     n = n + 1
#     if n < 50:
#         continue
#     if n % 20 == 0:
#         x2.append(m)
#         y2.append(float(line))
#         m = m + 1

plt.plot(x1, y1, marker='o', label='larvio td_data')
# plt.plot(x2, y2, marker='x', label='hkvins td_data')

plt.xlim(xmin=-5)
plt.ylim(ymin=-2.5)
plt.xticks(np.arange(min(x1), max(x1)+1, 100.0))

# for xy in zip(x, y):
#     plt.annotate("(%s,%s)" % xy, xy=xy, xytext=(-20, 10), textcoords='offset points')
plt.xlabel("frame num")
plt.ylabel("time offset (ms)")
plt.title('td(ms) evaluation on sensetime A1 dataset')

plt.legend()

plt.show()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 28 19:52:05 2020

@author: cg
"""


import numpy as np
import matplotlib.pyplot as plt

T = 10
dt = 0.1
N = round(T/dt)
t = np.linspace(0, T, N)

z1 = np.random.standard_normal(size=N)

z2 = np.cumsum(z1)

z = z2 * np.sqrt(dt)

plt.xlabel('t')
plt.ylabel('z')
plt.title(u'BM')
plt.plot(t, z)
plt.show()
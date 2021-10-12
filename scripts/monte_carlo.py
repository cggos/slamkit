#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 28 19:21:37 2020

Monte Carlo method

@author: cg
"""


import random


def calpai():
    n = 1000000
    r = 1.0
    a, b = (0.0, 0.0)
    x_neg, x_pos = a - r, a + r
    y_neg, y_pos = b - r, b + r

    count = 0
    for i in range(0, n):
        x = random.uniform(x_neg, x_pos)
        y = random.uniform(y_neg, y_pos)
        if x*x + y*y <= 1.0:
            count += 1

    print( (count / float(n)) * 4)

calpai()


def integral():
    n = 1000000
    x_min, x_max = 0.0, 1.0
    y_min, y_max = 0.0, 1.0

    count = 0
    for i in range(0, n):
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        # x*x > y，表示该点位于曲线的下面。所求的积分值即为曲线下方的面积与正方形面积的比。
        if x*x > y:
            count += 1

    integral_value = count / float(n)
    print(integral_value)
    
integral()
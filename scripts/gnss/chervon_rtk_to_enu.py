#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import matplotlib.pyplot as plt

# pip install pymap3d
import pymap3d as pm

fi = open("/Users/gavin.gao/Downloads/RTK/data_bag.txt", 'r')
fo = open("/Users/gavin.gao/Downloads/RTK/rtk_enu.tum", 'w')


def get_wgs84(line_data):
    lon = float(line_data[5])
    lat = float(line_data[6])
    alt = float(line_data[7])

    lon = int(lon / 100) + (lon / 100 - int(lon / 100)) / 100 * 60
    lat = int(lat / 100) + (lat / 100 - int(lat / 100)) / 100 * 60

    return lon, lat, alt


n = 0
xx = []
yy = []
for line in fi:
    global lon0, lat0, alt0
    if line[:4] == "#rtk":
        n += 1
        data = line.split(' ')
        if n == 1:
            lon0, lat0, alt0 = get_wgs84(data)
            continue
        lon, lat, alt = get_wgs84(data)
        res = pm.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
        xx.append(res[0])
        yy.append(res[1])
        fo.write(("{} {} {} {} {} {} {} {}\n").format(n, res[0], res[1], res[2], 0, 0, 0, 1))
        print(res)

fi.close()
fo.close()

plt.plot(xx, yy)

plt.show()

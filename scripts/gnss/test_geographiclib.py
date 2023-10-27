#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from geographiclib.geodesic import Geodesic

geod = Geodesic.WGS84

print(geod.a)

# import geographiclib as gc
#
# # Create a Geodesic object
# geodesic = gc.Geodesic("WGS84")
#
# # Create a LocalCartesian object
# local_cartesian = gc.LocalCartesian(geodesic, 40.0, -70.0, 100.0)
#
# # Convert the point from geodetic coordinates to local cartesian coordinates
# point = local_cartesian.convert(geodesic, gc.Point(40.0, -70.0, 100.0))
#
# # Print the point in local cartesian coordinates
# print(point.x, point.y, point.z)

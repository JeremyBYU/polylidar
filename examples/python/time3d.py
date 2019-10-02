import time

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from polylidar import extractPlanesAndPolygons


config = dict(alpha=0.0, xyThresh=0.0, lmax=100.0)
points = np.random.randn(100_000, 3) * 100
points[:,2] = points[:,2] * 0.0001

t0 = time.time()
delaunay, planes, polygons = extractPlanesAndPolygons(points, **config)
t1 = time.time()
print("Time: {:.2f}".format((t1-t0) * 1000))
print(polygons)


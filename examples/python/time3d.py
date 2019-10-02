import time

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from polylidar import extractPolygons, extractPlanesAndPolygons
from polylidarutil import (generate_test_points, plot_points, plot_triangles, generate_3d_plane, set_axes_equal, plot_planes_3d,
                            plot_triangle_meshes, get_triangles_from_he, get_plane_triangles, plot_polygons, scale_points)
from tests.helpers.utils import load_csv


config = dict(alpha=0.0, xyThresh=0.0, lmax=100.0)
points = load_csv('100K_array_3d.csv')

t0 = time.time()
delaunay, planes, polygons = extractPlanesAndPolygons(points, **config)
t1 = time.time()
print("Time: {:.2f}".format((t1-t0) * 1000))
print("Point indices of concave shell:")
print(polygons[0].shell)

# triangles = np.asarray(delaunay.triangles).reshape(int(len(delaunay.triangles)/ 3), 3)
# fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1, subplot_kw=dict(projection='3d'))
# # plot all triangles
# plot_planes_3d(points, triangles, planes, ax)
# # plot points
# ax.scatter(*scale_points(points), c='k', s=0.1)
# set_axes_equal(ax)
# plt.show()

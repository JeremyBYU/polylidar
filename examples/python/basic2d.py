import time
import math

import numpy as np
import matplotlib.pyplot as plt

from polylidar import extractPlanesAndPolygons
from polylidarutil import (generate_test_points, plot_points, plot_triangles, get_estimated_lmax,
                            plot_triangle_meshes, get_triangles_from_he, get_plane_triangles, plot_polygons)

kwargs = dict(num_groups=2, group_size=1000, dist=100.0, seed=1)
# generate random normally distributed clusters of points, 200 X 2 numpy array.
points = generate_test_points(**kwargs)
lmax = get_estimated_lmax(**kwargs)

# Extracts planes and polygons, time
t1 = time.time()
delaunay, planes, polygons = extractPlanesAndPolygons(points, alpha=0.0, lmax=lmax, minTriangles=5)
t2 = time.time()
print("Took {:.2f} milliseconds".format((t2 - t1) * 1000))

# Plot Data
if points.shape[0] < 100000:
    fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
    # plot points
    plot_points(points, ax)
    # plot all triangles
    plot_triangles(get_triangles_from_he(delaunay.triangles, points), ax)
    # plot mesh triangles
    triangle_meshes = get_plane_triangles(planes, delaunay.triangles, points)
    plot_triangle_meshes(triangle_meshes, ax)
    # plot polygons
    plot_polygons(polygons, delaunay, points, ax)

    plt.axis('equal')

    plt.show()

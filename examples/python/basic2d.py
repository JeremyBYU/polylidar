import time
import math

import numpy as np
import matplotlib.pyplot as plt

from polylidar import extractPlanesAndPolygons
from polylidarutil import (generate_test_points, plot_points, plot_triangles,
                            plot_triangle_meshes, get_triangles_from_he, get_plane_triangles, plot_polygons)

# generate random points, 200 X 2 numpy array.
kwargs = dict(num_groups=2, group_size=100, dist=100.0)
points = generate_test_points(**kwargs)
clust_radius = math.sqrt(kwargs['dist'] / kwargs['num_groups'])
print(clust_radius)
clust_point_density = (kwargs['group_size'] * .50) / (math.pi * clust_radius * clust_radius)
print(clust_point_density)
lmax = 3 / math.sqrt(clust_point_density)
print(lmax) 

# Extracts planes and polygons, time
t1 = time.time()
delaunay, planes, polygons = extractPlanesAndPolygons(points, xyThresh=0.0, alpha=0.0, lmax=lmax, minTriangles=5)
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

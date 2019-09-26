import time

import numpy as np
import matplotlib.pyplot as plt

from polylidar import extractPlanesAndPolygons
from polylidarutil import (generate_test_points, plot_points, plot_triangles,
                            plot_triangle_meshes, get_triangles_from_he, get_plane_triangles, plot_polygons)

# generate random points, 200 X 2 numpy array.
points = generate_test_points(group_size=100)

# Extracts planes and polygons, time
t1 = time.time()
delaunay, planes, polygons = extractPlanesAndPolygons(points, xyThresh=0.0, alpha=0.0, lmax=5.0, minTriangles=20)
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

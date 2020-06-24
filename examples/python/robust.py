import time
import numpy as np
import matplotlib.pyplot as plt
from python_tests.helpers.utils import load_csv
from polylidar import Polylidar3D, MatrixDouble
from polylidar.polylidarutil import (plot_points, plot_triangles,
                           plot_triangle_meshes, get_triangles_from_he, get_plane_triangles, plot_polygons)

# Load point set that for which delaunator generates invalid convex hulls when using non-robust predicates
# Convex hull should be malformed if polylidar not built with robust predicates
robust_tests = ['robust_1.csv']
polylidar = Polylidar3D(alpha=0.0, lmax=1000.0, min_triangles=1)
for robust_test in robust_tests:
    points = load_csv(robust_test)
    points_mat = MatrixDouble(points)

    # Extracts planes and polygons, time
    t1 = time.time()
    # pick large lmax to get the convex hull, no triangles filtered
    mesh, planes, polygons = polylidar.extract_planes_and_polygons(points_mat)
    t2 = time.time()
    print("Took {:.2f} milliseconds".format((t2 - t1) * 1000))
    print("If Robust Geoemetric Predicates is NOT activated, you should see a malformed concave hull")
    print("See README.md to activate if desired. ~20% performance penalty when active.")
    print("")

    triangles = np.asarray(mesh.triangles)
    # print(triangles, triangles.shape)
    triangles = triangles.flatten()
    # print(triangles, triangles.shape)
    planes_np = planes
    print("Took {:.2f} milliseconds".format((t2 - t1) * 1000))

    # Plot Data
    if points.shape[0] < 100000:
        fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
        # plot points
        plot_points(points, ax)
        # plot all triangles
        plot_triangles(get_triangles_from_he(triangles, points), ax)
        # plot mesh triangles
        triangle_meshes = get_plane_triangles(planes_np, triangles, points)
        plot_triangle_meshes(triangle_meshes, ax)
        # plot polygons
        plot_polygons(polygons, points, ax)

        plt.axis('equal')

        plt.show()



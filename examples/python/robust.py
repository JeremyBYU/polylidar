"""Demonstrates Polylidar3D using Robust Geometric Predicates with Delaunator
You need to have built Polylidar3D with this enabled to see correct polygon generation
"""
import time
import numpy as np
import matplotlib.pyplot as plt
from tests.python.helpers.utils import load_csv
from polylidar import Polylidar3D, MatrixDouble
from polylidar.polylidarutil import (plot_points, plot_triangles,
                                     plot_triangle_meshes, get_triangles_from_list, get_colored_planar_segments, plot_polygons)

# Load point set that for which delaunator generates invalid convex hulls when using non-robust predicates
# Convex hull should be malformed if polylidar not built with robust predicates
def main():
    points = load_csv('robust_1.csv')
    polylidar_kwargs = dict(alpha=0.0, lmax=1000.0, min_triangles=1)

    points_mat = MatrixDouble(points)
    polylidar = Polylidar3D(**polylidar_kwargs)

    # Extracts planes and polygons, time
    t1 = time.time()
    # pick large lmax to get the convex hull, no triangles filtered
    mesh, planes, polygons = polylidar.extract_planes_and_polygons(points_mat)
    t2 = time.time()
    print("Took {:.2f} milliseconds".format((t2 - t1) * 1000))
    print("If Robust Geometric Predicates is NOT activated, you should see a malformed convex hull")
    print("See README.md to activate if desired. ~20% performance penalty when active.")
    print("")
    # Convert to numpy format, no copy with np.asarray()
    triangles = np.asarray(mesh.triangles)

    fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
    # plot points
    ax.scatter(points[:, 0], points[:, 1], c='k')
    # plot all triangles
    plot_triangles(get_triangles_from_list(triangles, points), ax)
    # plot seperated planar triangular segments
    triangle_meshes = get_colored_planar_segments(planes, triangles, points)
    plot_triangle_meshes(triangle_meshes, ax)
    # plot polygons
    plot_polygons(polygons, points, ax)
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    main()

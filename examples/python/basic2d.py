"""Example of extracting MultiPolygons with holes of a 2D point set
"""
import time

import numpy as np
from polylidar import MatrixDouble, Polylidar3D
from polylidar.polylidarutil import (generate_test_points, plot_triangles, get_estimated_lmax,
                                     plot_triangle_meshes, get_triangles_from_list, get_colored_planar_segments, plot_polygons)
import matplotlib.pyplot as plt

np.random.seed(1)


def main():
    kwargs = dict(num_groups=2, group_size=1000, dist=100.0, seed=1)
    # generate 2 random normally distributed clusters of points, 200 X 2 numpy array.
    points = generate_test_points(**kwargs)
    lmax = get_estimated_lmax(**kwargs)
    polylidar_kwargs = dict(alpha=0.0, lmax=lmax, min_triangles=5)

    # Convert points to matrix format (no copy) and make Polylidar3D Object
    points_mat = MatrixDouble(points, copy=False)
    polylidar = Polylidar3D(**polylidar_kwargs)

    # Extract the mesh, planes, polygons, and time
    t1 = time.perf_counter()
    mesh, planes, polygons = polylidar.extract_planes_and_polygons(points_mat)
    t2 = time.perf_counter()

    print("Took {:.2f} milliseconds".format((t2 - t1) * 1000))

    # Convert to numpy format, no copy with np.asarray()
    triangles = np.asarray(mesh.triangles)

    fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
    # plot points
    ax.scatter(points[:, 0], points[:, 1], c='k')
    # plot all triangles
    # plt.triplot(points[:,0], points[:,1], triangles) # better alternative
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

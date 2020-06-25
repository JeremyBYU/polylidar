"""Example of extracting MultiPolygons with holes of a 2D point set. Step through algorithm.
"""
import time
import numpy as np
from polylidar import Delaunator, MatrixDouble, Polylidar3D
from polylidar.polylidarutil import (generate_test_points, plot_points, plot_triangles, get_estimated_lmax,
                            plot_triangle_meshes, get_triangles_from_list, get_colored_planar_segments, plot_polygons)
import matplotlib.pyplot as plt

np.random.seed(1)

def get_np_buffer_ptr(a):
    pointer, read_only_flag = a.__array_interface__['data']
    return pointer

def main():

    kwargs = dict(num_groups=2, group_size=1000, dist=100.0, seed=1)
    # generate random normally distributed clusters of points, 200 X 2 numpy array.
    points = generate_test_points(**kwargs)
    lmax = get_estimated_lmax(**kwargs)
    polylidar_kwargs = dict(alpha=0.0, lmax=lmax, min_triangles=5)
    # print(polylidar_kwargs)
    # Convert Points and make Polylidar
    points_mat = MatrixDouble(points)
    polylidar = Polylidar3D(**polylidar_kwargs)
    # Extracts planes and polygons, time
    t1 = time.perf_counter()
    mesh, planes, polygons = polylidar.extract_planes_and_polygons(points_mat)
    t2 = time.perf_counter()
    triangles = np.asarray(mesh.triangles)
    # print(triangles, triangles.shape)
    triangles = triangles.flatten()
    # print(triangles, triangles.shape)
    planes_np = np.asarray(planes)
    # print(planes_np)
    print("Took {:.2f} milliseconds".format((t2 - t1) * 1000))

    # Plot Data
    if points.shape[0] < 100000:
        fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
        # plot points
        print("Plot Points")
        ax.scatter(points[:, 0], points[:, 1], c='k', s=20.0)
        fig.savefig("assets/scratch/Basic2DAlgorithm_pointcloud.png", bbox_inches='tight', pad_inches=-0.6)
        plt.show()

        print("Plot Mesh")
        fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
        # plot points
        ax.scatter(points[:, 0], points[:, 1], c='k', s=0.1)
        # plot all triangles
        plot_triangles(get_triangles_from_list(triangles, points), ax)
        fig.savefig("assets/scratch/Basic2DAlgorithm_mesh.png", bbox_inches='tight', pad_inches=-0.6)
        plt.show()

        print("Planes and Polygons")
        fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
        # plot points
        ax.scatter(points[:, 0], points[:, 1], c='k', s=0.1)
        # plot all triangles
        plot_triangles(get_triangles_from_list(triangles, points), ax)
        # plot mesh triangles
        triangle_meshes = get_colored_planar_segments(planes_np, triangles, points)
        plot_triangle_meshes(triangle_meshes, ax)
        # plot polygons
        plot_polygons(polygons, points, ax)

        plt.axis('equal')

        plt.show()

if __name__ == "__main__":
    main()
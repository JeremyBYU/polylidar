import time
import numpy as np
from polylidar import Delaunator, MatrixDouble, Polylidar3D
from polylidar.polylidarutil import (generate_test_points, plot_points, plot_triangles, get_estimated_lmax,
                            plot_triangle_meshes, get_triangles_from_he, get_plane_triangles, plot_polygons)
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
    polylidar_kwargs = dict(alpha=0.0, lmax=lmax)
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

# def main():
#     points = np.random.randn(1000, 2)
#     polylidar_kwargs = dict(alpha=0.0, lmax=0.1)
#     # Use Delaunay Triangulation
#     points_mat = MatrixDouble(points)
#     polylidar = Polylidar3D(**polylidar_kwargs)
#     # delaunay = Delaunator(points_mat)
#     # delaunay.triangulate()

#     # triangles_ = np.asarray(delaunay.triangles)
#     # print(triangles_)
#     # print("")
#     mesh, planes, polygons = polylidar.extract_planes_and_polygons(points_mat)
#     triangles1 = np.asarray(mesh.triangles)
#     planes = np.asarray(planes)
#     print(planes)
    
#     # print(triangles1)
#     # print(np.max(triangles1))
#     # print(triangles1.flags)
#     # print(triangles1.dtype)
#     # print(get_np_buffer_ptr(triangles1))
#     # print("")

#     # triangles2 = np.copy(triangles1)
#     # print(triangles2)
#     # print(np.max(triangles2))
#     # print(triangles2.flags)
#     # print(triangles2.dtype)
#     # print(get_np_buffer_ptr(triangles2))
#     # print("")

#     # print(triangles1)
#     # print(np.max(triangles1))
#     # print(triangles1.flags)
#     # print(triangles1.dtype)
#     # print(get_np_buffer_ptr(triangles1))
#     # print("")
    
#     plt.triplot(points[:,0], points[:,1], triangles1)

#     plt.scatter(points[:, 0], points[: ,1])
#     plt.show()

if __name__ == "__main__":
    main()
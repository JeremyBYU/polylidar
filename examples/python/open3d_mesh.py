"""
This work has little to do with Polylidar3D. Just some work demonstrating mesh creation using different techniques
"""
import time
import math

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from polylidar import Polylidar3D, MatrixDouble
from polylidar.polylidarutil import (plot_polygons_3d, generate_3d_plane, set_axes_equal, plot_planes_3d,
                           scale_points, rotation_matrix, apply_rotation, COLOR_PALETTE)

from scipy.spatial import Delaunay
from scipy.stats import describe
import open3d as o3d

def create_open_3d_mesh(triangles, points, color=[1, 0, 0]):
    mesh_2d = o3d.geometry.TriangleMesh()
    mesh_2d.triangles = o3d.utility.Vector3iVector(triangles)
    mesh_2d.vertices = o3d.utility.Vector3dVector(points)
    mesh_2d.compute_vertex_normals()
    mesh_2d.compute_triangle_normals()
    mesh_2d.paint_uniform_color(color)
    return mesh_2d

def extract_mesh_planes(points, mesh, planes, color=None):
    triangles = np.asarray(mesh.triangles)
    meshes = []
    for i, plane in enumerate(planes):
        if color is None:
            color = COLOR_PALETTE[i]
        tris = np.ascontiguousarray(np.flip(triangles[plane, :], 1))
        mesh = create_open_3d_mesh(tris, points, color)
        meshes.append(mesh)
    return meshes


def generate_point_cloud(max_size=10):
    np.random.seed(1)
    # generate random plane with hole
    plane = generate_3d_plane(bounds_x=[0, max_size, 0.5], bounds_y=[0, max_size, 0.5], holes=[], #holes=[[[3, 5], [3, 5]]], 
                            height_noise=0.05, planar_noise=0.02)
    # Add double plane to simulate extra noise
    # plane2 = plane + [0.1, 0.1, 0.05]
    # plane = np.vstack((plane, plane2))
    # Generate top of box (causing the hole that we see)
    box_top = generate_3d_plane(bounds_x=[3, 5, 0.2], bounds_y=[3, 5, 0.2], holes=[
    ], height_noise=0.02, height=2, planar_noise=0.02)
    # Generate side of box (causing the hole that we see)
    box_side = generate_3d_plane(bounds_x=[0, 2, 0.2], bounds_y=[
                                0, 2, 0.2], holes=[], height_noise=0.02, planar_noise=0.02)
    rm = rotation_matrix([0,1,0], -math.pi/2.0)
    box_side = apply_rotation(rm, box_side) + [5, 3, 0]
    # box_side = r.apply(box_side) + [5, 3, 0]
    # All points joined together
    points = np.ascontiguousarray(np.concatenate((plane, box_side, box_top)))
    return points

def create_open3d_pc(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def main():
    points = generate_point_cloud()
    points_mat = MatrixDouble(points)
    polylidar_kwargs = dict(alpha=0.0, lmax=1.0, min_triangles=20, z_thresh=0.2, norm_thresh=0.98)
    polylidar = Polylidar3D(**polylidar_kwargs)
    t1 = time.perf_counter()
    # Create Pseudo 3D Surface Mesh using Delaunay Triangulation and Polylidar
    mesh, planes, polygons = polylidar.extract_planes_and_polygons(points_mat)
    t2 = time.perf_counter()
    print("Point Size: {}".format(points.shape[0]))
    print("2.5D Delaunay Triangulation and Plane Extraction; Mesh Creation {:.2f} milliseconds".format((t2 - t1) * 1000))

    # Visualize
    # Create Open3D Mesh
    mesh_planes = extract_mesh_planes(points, mesh, planes, COLOR_PALETTE[0])
    # Create Open 3D Point Cloud
    pcd = create_open3d_pc(points)
    o3d.visualization.draw_geometries([pcd, *mesh_planes])

    # Estimate Point Cloud Normals
    t0 = time.perf_counter()
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=8))
    t1 = time.perf_counter()
    # Create True 3D Surface Mesh using Ball Pivot Algorithm
    radii = o3d.utility.DoubleVector([0.4, 0.5])
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, radii)
    mesh.paint_uniform_color(COLOR_PALETTE[0])
    t2 = time.perf_counter()
    print("3D Triangulation using Ball Pivot: Normal Estimation took: {:.2f}, Mesh Creation: {:.2f}".format((t1-t0) * 1000,(t2-t1) * 1000))
    # Visualize
    o3d.visualization.draw_geometries([pcd, mesh])

    # Create True 3D Surface Mesh using Poisson Reconstruction Algorithm
    t3 = time.perf_counter()
    mesh, densities = o3d.geometry.TriangleMesh. create_from_point_cloud_poisson(pcd, depth=6)
    t4 = time.perf_counter()
    print("3D Triangulation using Poisson Reconstruction: Mesh Creation: {:.2f}".format((t4-t3) * 1000))
    # Visualize
    # o3d.visualization.draw_geometries([mesh])
    # print(mesh)
    # print('visualize densities')
    densities = np.asarray(densities)
    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    # mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    # o3d.visualization.draw_geometries([density_mesh])

    # print('remove low density vertices')
    vertices_to_remove = densities < np.quantile(densities, 0.1)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    mesh.compute_triangle_normals()
    mesh.paint_uniform_color(COLOR_PALETTE[0])
    # print(mesh)
    o3d.visualization.draw_geometries([pcd, mesh])


if __name__ == "__main__":
    main()


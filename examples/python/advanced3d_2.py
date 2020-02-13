import time
import math

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from polylidar import extractPlanesAndPolygons, extract_planes_and_polygons_from_mesh
from polylidarutil.plane_filtering import filter_planes_and_holes
from polylidarutil import (plot_polygons_3d, generate_3d_plane, set_axes_equal, plot_planes_3d,
                           scale_points, rotation_matrix, apply_rotation, COLOR_PALETTE)
from polylidarutil.open3d_util import construct_grid, create_lines, flatten

from scipy.spatial import Delaunay
from scipy.spatial.transform import Rotation as R
from scipy.stats import describe
import open3d as o3d

def create_open_3d_mesh(triangles, points, color=COLOR_PALETTE[0]):
    """Create an Open3D Mesh given triangles vertices

    Arguments:
        triangles {ndarray} -- Triangles array
        points {ndarray} -- Points array

    Keyword Arguments:
        color {list} -- RGB COlor (default: {[1, 0, 0]})

    Returns:
        mesh -- Open3D Mesh
    """
    mesh_2d = o3d.geometry.TriangleMesh()
    if triangles.ndim == 1:
        triangles = triangles.reshape((int(triangles.shape[0] / 3), 3))
        # Open 3D expects triangles to be counter clockwise
        triangles = np.ascontiguousarray(np.flip(triangles, 1))
    mesh_2d.triangles = o3d.utility.Vector3iVector(triangles)
    mesh_2d.vertices = o3d.utility.Vector3dVector(points)
    mesh_2d.compute_vertex_normals()
    mesh_2d.compute_triangle_normals()
    mesh_2d.paint_uniform_color(color)
    return mesh_2d


def extract_mesh_planes(points, triangles, planes, color=None):
    " Converts Polylidar Mesh Planes into Open3D format "
    meshes = []
    color_ = color
    triangles_ = triangles
    if triangles.ndim == 1:
        triangles_ = triangles.reshape((int(triangles.shape[0] / 3), 3))
        # Open 3D expects triangles to be counter clockwise
        triangles_ = np.ascontiguousarray(np.flip(triangles_, 1))
    for i, plane in enumerate(planes):
        if color is None:
            color_ = COLOR_PALETTE[i % (len(COLOR_PALETTE) - 1)]
        else:
            color_ = COLOR_PALETTE[0]
        tris = np.ascontiguousarray(triangles_[plane, :])
        mesh = create_open_3d_mesh(tris, points, color_)
        meshes.append(mesh)
    return meshes

def filter_and_create_open3d_polygons(points, polygons):
    " Apply polygon filtering algorithm, return Open3D Mesh Lines "
    config_pp = dict(filter=dict(hole_area=dict(min=0, max=10.0), hole_vertices=dict(min=3), plane_area=dict(min=0.0)),
                     positive_buffer=0.00, negative_buffer=0.00, simplify=0.00)
    planes, obstacles = filter_planes_and_holes(polygons, points, config_pp)
    all_poly_lines = create_lines(planes, obstacles, line_radius=0.015)
    return all_poly_lines


def generate_point_cloud(max_size=100):
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
                                0, 2, 0.2], holes=[], height_noise=0.05, planar_noise=0.02)
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

points = generate_point_cloud()


polylidar_kwargs = dict(alpha=0.0, lmax=1.0, minTriangles=10, zThresh=0.2, normThresh=0.98, normThreshMin=0.95)
t1 = time.perf_counter()
# Create Pseudo 3D Surface Mesh using Delaunay Triangulation and Polylidar
delaunay, planes, polygons = extractPlanesAndPolygons(points, **polylidar_kwargs)
t2 = time.perf_counter()
print("Point Size: {}".format(points.shape[0]))
print("2D Delaunay Triangulation and Plane Extraction; Mesh Creation {:.2f} milliseconds".format((t2 - t1) * 1000))

# Visualize
# Create Open3D Mesh
mesh_planes = extract_mesh_planes(points, np.asarray(delaunay.triangles), planes)
# Create Open 3D Point Cloud
pcd = create_open3d_pc(points)
# o3d.visualization.draw_geometries([pcd, *mesh_planes])

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
# o3d.visualization.draw_geometries([pcd, mesh])

print("Sending Mesh to Polylidar to Extract Planes")
num_triangles = np.asarray(mesh.triangles).shape[0]
vertices = np.asarray(mesh.vertices)
# triangles produced by open3d are in counter clockwise order, polylidar expects clockwise
triangles = np.ascontiguousarray(np.flip(np.asarray(mesh.triangles), 1)).flatten()
mesh.triangles = o3d.utility.Vector3iVector(np.reshape(triangles, (num_triangles, 3)))
halfedges = np.asarray(o3d.geometry.HalfEdgeTriangleMesh.extract_halfedges(mesh))


polylidar_kwargs = dict(alpha=0.0, lmax=1.0, minTriangles=10, zThresh=0.15, normThresh=0.98, desiredVector=[1, 0, 0], normThreshMin=0.95)
planes, polygons = extract_planes_and_polygons_from_mesh(vertices, triangles, halfedges, **polylidar_kwargs)
for poly in polygons:
    print(poly.shell)
    print(poly.holes)
# Convert to Open3D Geometries
mesh_planes = extract_mesh_planes(points, triangles, planes)
all_poly_lines = filter_and_create_open3d_polygons(points, polygons)
print(all_poly_lines)

mesh_planes.extend(flatten([line_mesh.cylinder_segments for line_mesh in all_poly_lines]))
axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)


# polylidar_marker = o3d.geometry.TriangleMesh.create_icosahedron(0.05)
# polylidar_marker.translate(points[418, :])

# polylidar_marker2 = o3d.geometry.TriangleMesh.create_icosahedron(0.05)
# polylidar_marker2.translate(points[427, :])
# polylidar_marker2.paint_uniform_color([0, 1.0, 0])

# polylidar_junction = o3d.geometry.TriangleMesh.create_icosahedron(0.02)
# polylidar_junction.translate(points[442, :])
# polylidar_junction.paint_uniform_color([1, 0, 0])


o3d.visualization.draw_geometries([pcd, axis_frame, *mesh_planes])
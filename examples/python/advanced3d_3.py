import time
import logging

import numpy as np

from examples.python.util.realsense_util import (get_realsense_data, get_frame_data, R_Standard_d400, prep_mesh,
                                            create_open3d_pc, extract_mesh_planes, COLOR_PALETTE, create_open_3d_mesh)
from examples.python.util.mesh_util import get_mesh_data_iterator

from polylidar import (extractPlanesAndPolygons, extract_planes_and_polygons_from_mesh, extract_tri_mesh_from_float_depth,
                      extract_point_cloud_from_float_depth, create_tri_mesh_copy)

from polylidarutil.open3d_util import construct_grid, create_lines, flatten
from polylidarutil.plane_filtering import filter_planes_and_holes

import open3d as o3d


def filter_and_create_open3d_polygons(points, polygons):
    " Apply polygon filtering algorithm, return Open3D Mesh Lines "
    # config_pp = dict(filter=dict(hole_area=dict(min=0.025, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.5)),
    #                  positive_buffer=0.00, negative_buffer=0.02, simplify=0.02)
    config_pp = dict(filter=dict(hole_area=dict(min=0.00, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.5)),
                     positive_buffer=0.00, negative_buffer=0.0, simplify=0.01)
    planes, obstacles = filter_planes_and_holes(polygons, points, config_pp)
    all_poly_lines = create_lines(planes, obstacles, line_radius=0.01)
    return all_poly_lines

def open_3d_mesh_to_trimesh(mesh: o3d.geometry.TriangleMesh):
    triangles = np.asarray(mesh.triangles)
    vertices = np.asarray(mesh.vertices)
    triangle_normals = np.asarray(mesh.triangle_normals)
    
    triangles = np.ascontiguousarray(np.flip(triangles, 1))
    tri_mesh = create_tri_mesh_copy(vertices, triangles)
    return tri_mesh
    # print(triangles)
    # print(np.asarray(tri_mesh.triangles))
    # print(vertices)
    # print(np.asarray(tri_mesh.vertices))
    # print(np.asarray(tri_mesh.halfedges))


def run_test(mesh, callback=None, stride=2):
    # Create Pseudo 3D Surface Mesh using Delaunay Triangulation and Polylidar
    polylidar_kwargs = dict(alpha=0.0, lmax=0.10, minTriangles=100,
                            zThresh=0.03, normThresh=0.98, normThreshMin=0.95, minHoleVertices=6)
    # Create Polylidar TriMesh
    # TODO convert this to a polylidar TriMesh
    tri_mesh = open_3d_mesh_to_trimesh(mesh)
    triangles = np.asarray(tri_mesh.triangles)
    vertices = np.asarray(tri_mesh.vertices)
    vertices = np.ascontiguousarray(vertices.reshape(int(vertices.shape[0] / 3), 3))
    triangles = triangles.reshape(int(triangles.shape[0] / 3), 3)
    t1 = time.perf_counter()
    planes, polygons = extract_planes_and_polygons_from_mesh(tri_mesh, **polylidar_kwargs)
    t2 = time.perf_counter()
    print("Extracted Planes: ",t2-t1)
    all_poly_lines = filter_and_create_open3d_polygons(vertices, polygons)
    print("Filtered Planes")
    # all_poly_lines = all_poly_lines[0:10]
    print(len(all_poly_lines))
    # print(len(all_poly_lines[0].cylinder_segments))
    # mesh_3d_polylidar = extract_mesh_planes(vertices, triangles, planes)
    mesh_3d_polylidar = []
    print(mesh_3d_polylidar)
    mesh_3d_polylidar.extend(flatten([line_mesh.cylinder_segments for line_mesh in all_poly_lines]))
    mesh_3d_polylidar.append(mesh)
    time_polylidar3D = (t2 - t1) * 1000
    polylidar_3d_alg_name = 'Polylidar with Uniform Grid Mesh'
    callback(polylidar_3d_alg_name, time_polylidar3D, mesh_3d_polylidar)


def make_uniform_grid_mesh(im, intrinsics, extrinsics, stride=2, **kwargs):
    """Create a Unifrom Grid Mesh from an RGBD Image

    Arguments:
        img {ndarray} -- MXN Float Depth Image
        intrinsics {ndarray} -- 3X3 intrinsics matrix
        extrinsics {ndarray} -- 4X4 matrix

    Keyword Arguments:
        stride {int} -- Stride for creating point cloud (default: {2})

    Returns:
        tuple(dict, dict) - Mesh and timings
    """
    t0 = time.perf_counter()
    tri_mesh = extract_tri_mesh_from_float_depth(im, intrinsics, extrinsics, stride=stride)
    t1 = time.perf_counter()
    points = np.asarray(tri_mesh.vertices)
    triangles = np.asarray(tri_mesh.triangles)
    halfedges = np.asarray(tri_mesh.halfedges)
    points = points.reshape((int(points.shape[0] / 3), 3))

    t2 = time.perf_counter()
    polylidar_inputs = dict(
        vertices=points, triangles=triangles, halfedges=halfedges, tri_mesh=tri_mesh)
    timings = dict(mesh_creation=(t1 - t0) * 1000, pc_rotation=(t2 - t1) * 1000)
    return polylidar_inputs, timings


def callback(alg_name, execution_time,mesh=None):
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -0.7])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    logging.info("%s took %.2f milliseconds", alg_name, execution_time)
    if mesh:
        if isinstance(mesh, list):
            o3d.visualization.draw_geometries(
                [*mesh, grid_ls, axis_frame])
        else:
            o3d.visualization.draw_geometries([mesh, grid_ls, axis_frame])

def main():
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -1.0])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-1.0, translate=[0, 0.0, 0.0])
    for i, mesh in enumerate(get_mesh_data_iterator()):
        if i < 0:
            continue
        # mesh = mesh.filter_smooth_simple(5)
        mesh = mesh.filter_smooth_laplacian(20)
        mesh.compute_triangle_normals()
        # logging.info("File %r - Point Cloud; Size: %r", idx, np.asarray(pcd.points).shape[0])
        o3d.visualization.draw_geometries([mesh, grid_ls, axis_frame])
        run_test(mesh, callback=callback, stride=2)


if __name__ == "__main__":
    main()

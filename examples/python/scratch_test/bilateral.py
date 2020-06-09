import time
import logging
import warnings
import numpy as np
from scipy.spatial.transform import Rotation as R

warnings.filterwarnings("ignore", message="Optimal rotation is not uniquely or poorly defined ")
np.set_printoptions(precision=4, suppress=True)

from examples.python.util.realsense_util import (get_realsense_data, get_frame_data, R_Standard_d400, prep_mesh,
                                            create_open3d_pc, extract_mesh_planes, COLOR_PALETTE, create_open_3d_mesh)
from examples.python.util.mesh_util import get_mesh_data_iterator

from polylidar import (Polylidar3D, MatrixDouble, MatrixFloat, MatrixInt, create_tri_mesh_copy, bilateral_filter_normals)

from polylidar.polylidarutil.open3d_util import construct_grid, create_lines, flatten
from polylidar.polylidarutil.plane_filtering import filter_planes_and_holes

from fastga import GaussianAccumulatorS2, MatX3d
from fastga.peak_and_cluster import find_peaks_from_accumulator


import open3d as o3d


def filter_and_create_open3d_polygons(points, polygons, rm=None, line_radius=0.005):
    " Apply polygon filtering algorithm, return Open3D Mesh Lines "
    config_pp = dict(filter=dict(hole_area=dict(min=0.025, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.25)),
                     positive_buffer=0.02, negative_buffer=0.05, simplify=0.02)
    # config_pp = dict(filter=dict(hole_area=dict(min=0.00, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.5)),
    #                  positive_buffer=0.00, negative_buffer=0.0, simplify=0.01)
    t1 = time.perf_counter()
    planes, obstacles = filter_planes_and_holes(polygons, points, config_pp, rm=rm)
    t2 = time.perf_counter()
    logging.info("Plane Filtering Took (ms): %.2f", (t2-t1) * 1000)
    all_poly_lines = create_lines(planes, obstacles, line_radius=line_radius)
    return all_poly_lines, (t2-t1) * 1000

def open_3d_mesh_to_trimesh(mesh: o3d.geometry.TriangleMesh):
    triangles = np.asarray(mesh.triangles)
    vertices = np.asarray(mesh.vertices)
    # print(triangles)
    # print(triangles.dtype)
    # print("")
    triangles = np.ascontiguousarray(triangles)
    # print(triangles)
    # print(triangles.dtype)
    # print("")
    vertices_mat = MatrixDouble(vertices)
    triangles_mat = MatrixInt(triangles)
    triangles_mat_np = np.asarray(triangles_mat)
    # print(triangles_mat_np)

    tri_mesh = create_tri_mesh_copy(vertices_mat, triangles_mat)
    return tri_mesh

def main():
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -1.0])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-1.0, translate=[0, 0.0, 0.0])
    for i, mesh in enumerate(get_mesh_data_iterator()):
        if i < 1:
            continue
            # o3d.io.write_triangle_mesh('test.ply', mesh)
        mesh.compute_vertex_normals()
        mesh.compute_triangle_normals()
        print("Before")
        o3d.visualization.draw_geometries([mesh, axis_frame])

        tri_mesh = open_3d_mesh_to_trimesh(mesh)
        t1 = time.perf_counter()
        bilateral_filter_normals(tri_mesh, 3, 0.1, 0.1)
        t2 = time.perf_counter()
        print(t2-t1)
        normals_smooth = np.asarray(tri_mesh.triangle_normals)
        mesh.triangle_normals = o3d.utility.Vector3dVector(normals_smooth)

        print("After")
        o3d.visualization.draw_geometries([mesh, axis_frame])



if __name__ == "__main__":
    main()

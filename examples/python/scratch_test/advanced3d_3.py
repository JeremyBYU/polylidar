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

from polylidar import (Polylidar3D, MatrixDouble, MatrixFloat, MatrixInt, create_tri_mesh_copy)

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

def extract_all_dominant_planes(tri_mesh, vertices, polylidar_kwargs, ds=50, min_samples=10000):
    ga = GaussianAccumulatorS2(level=4, max_phi=180)
    triangle_normals = np.asarray(tri_mesh.triangle_normals)
    num_normals = triangle_normals.shape[0]
    # print(np.asarray(tri_mesh.triangles)[0:3])
    # print(triangle_normals[0,:])
    # print(triangle_normals.shape)
    # Downsample, TODO improve this
    ds_normals = int(num_normals / ds)
    to_sample = max(min([num_normals, min_samples]), ds_normals)
    ds_step = int(num_normals / to_sample)
    # print(ds_step)
    triangle_normals_ds = np.ascontiguousarray(triangle_normals[:num_normals:ds_step, :])
    # A copy occurs here for triangle_normals......if done in c++ there would be no copy
    ga.integrate(MatX3d(triangle_normals_ds))
    gaussian_normals = np.asarray(ga.get_bucket_normals())
    # print(gaussian_normals)
    accumulator_counts = np.asarray(ga.get_normalized_bucket_counts())
    _, _, avg_peaks, _ = find_peaks_from_accumulator(gaussian_normals, accumulator_counts)
    logging.info("Processing mesh with %d triangles", num_normals)
    logging.info("Dominant Plane Normals")
    print(avg_peaks)
    all_poly_lines = []
    pl = Polylidar3D(**polylidar_kwargs)
    avg_peaks_mat = MatrixDouble(avg_peaks)
    t0 = time.perf_counter()
    all_planes, all_polygons = pl.extract_planes_and_polygons_optimized(tri_mesh, avg_peaks_mat)
    t1 = time.perf_counter()


    polylidar_time = (t1 - t0) * 1000
    
    for i in range(avg_peaks.shape[0]):
        avg_peak = avg_peaks[i, :]
        rm, _ = R.align_vectors([[0, 0, 1]], [avg_peak])
        polygons_for_normal = all_polygons[i]
        # print(polygons_for_normal)
        if len(polygons_for_normal) > 0:
            poly_lines, _ = filter_and_create_open3d_polygons(vertices, polygons_for_normal, rm=rm)
            all_poly_lines.extend(poly_lines)

    return all_poly_lines, polylidar_time

def run_test(mesh, callback=None, stride=2):
    # Create Pseudo 3D Surface Mesh using Delaunay Triangulation and Polylidar
    polylidar_kwargs = dict(alpha=0.0, lmax=0.10, min_triangles=1000,
                            z_thresh=0.01, norm_thresh=0.95, norm_thresh_min=0.92, min_hole_vertices=6)
    # Create Polylidar TriMesh
    tri_mesh = open_3d_mesh_to_trimesh(mesh)
    vertices = np.asarray(tri_mesh.vertices)

    all_poly_lines, polylidar_time = extract_all_dominant_planes(tri_mesh, vertices, polylidar_kwargs)
    mesh_3d_polylidar = []
    mesh_3d_polylidar.extend(flatten([line_mesh.cylinder_segments for line_mesh in all_poly_lines]))
    mesh_3d_polylidar.append(mesh)

    time_polylidar3D = polylidar_time
    polylidar_3d_alg_name = 'Polylidar3D with Provided Mesh'
    callback(polylidar_3d_alg_name, time_polylidar3D, mesh_3d_polylidar)

def callback(alg_name, execution_time,mesh=None):
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -0.7])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    logging.info("%s took (ms): %.2f", alg_name, execution_time)
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
        if i < 1:
            # Dense mesh needs to be smoothed
            t0 = time.perf_counter()
            mesh = mesh.filter_smooth_laplacian(5, 0.75)
            t1 = time.perf_counter()
            logging.info("Laplacian Smoothing took (ms): %.2f",(t1-t0) * 1000)
            # o3d.io.write_triangle_mesh('test.ply', mesh)
        mesh.compute_triangle_normals()
        # o3d.visualization.draw_geometries([mesh, grid_ls, axis_frame])
        run_test(mesh, callback=callback, stride=2)


if __name__ == "__main__":
    main()

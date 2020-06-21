import time
import logging
import warnings
import numpy as np
from copy import deepcopy
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

warnings.filterwarnings("ignore", message="Optimal rotation is not uniquely or poorly defined ")
np.set_printoptions(precision=4, suppress=True)

from examples.python.util.realsense_util import (get_realsense_data, get_frame_data, R_Standard_d400, prep_mesh,
                                                 create_open3d_pc, extract_mesh_planes, COLOR_PALETTE, create_open_3d_mesh)
from examples.python.util.mesh_util import get_mesh_data_iterator

from polylidar import (Polylidar3D, MatrixDouble, MatrixFloat, MatrixInt,
                       create_tri_mesh_copy, bilateral_filter_normals)

from polylidar.polylidarutil.open3d_util import construct_grid, create_lines, flatten
from polylidar.polylidarutil.plane_filtering import filter_planes_and_holes

from fastga import GaussianAccumulatorS2, MatX3d
from fastga.peak_and_cluster import find_peaks_from_accumulator


import open3d as o3d


def filter_and_create_open3d_polygons(points, polygons, rm=None, line_radius=0.005):
    " Apply polygon filtering algorithm, return Open3D Mesh Lines "
    # config_pp = dict(filter=dict(hole_area=dict(min=0.0, max=100.0), hole_vertices=dict(min=3), plane_area=dict(min=0.05)),
    #                  positive_buffer=0.02, negative_buffer=0.05, simplify=0.02)
    config_pp = dict(filter=dict(hole_area=dict(min=0.00, max=100.0), hole_vertices=dict(min=3), plane_area=dict(min=0.05)),
                     positive_buffer=0.00, negative_buffer=0.0, simplify=0.01)
    t1 = time.perf_counter()
    planes, obstacles = filter_planes_and_holes(polygons, points, config_pp, rm=rm)
    t2 = time.perf_counter()
    logging.info("Plane Filtering Took (ms): %.2f", (t2 - t1) * 1000)
    all_poly_lines = create_lines(planes, obstacles, line_radius=line_radius)
    return all_poly_lines, (t2 - t1) * 1000


def open_3d_mesh_to_trimesh(mesh: o3d.geometry.TriangleMesh):
    triangles = np.asarray(mesh.triangles)
    vertices = np.asarray(mesh.vertices)
    triangles = np.ascontiguousarray(triangles)
    vertices_mat = MatrixDouble(vertices)
    triangles_mat = MatrixInt(triangles)
    tri_mesh = create_tri_mesh_copy(vertices_mat, triangles_mat)
    return tri_mesh


def extract_all_dominant_planes(tri_mesh, vertices, polylidar_kwargs, ds=50, min_samples=10000):
    ga = GaussianAccumulatorS2(level=4, max_phi=180)
    triangle_normals = np.asarray(tri_mesh.triangle_normals)
    num_normals = triangle_normals.shape[0]

    # Downsample, TODO improve this
    ds_normals = int(num_normals / ds)
    to_sample = max(min([num_normals, min_samples]), ds_normals)
    ds_step = int(num_normals / to_sample)

    triangle_normals_ds = np.ascontiguousarray(triangle_normals[:num_normals:ds_step, :])
    # A copy occurs here for triangle_normals......if done in c++ there would be no copy
    ga.integrate(MatX3d(triangle_normals_ds))
    gaussian_normals = np.asarray(ga.get_bucket_normals())
    accumulator_counts = np.asarray(ga.get_normalized_bucket_counts())
    _, _, avg_peaks, _ = find_peaks_from_accumulator(gaussian_normals, accumulator_counts)
    logging.info("Processing mesh with %d triangles", num_normals)
    logging.info("Dominant Plane Normals")
    print(avg_peaks)

    avg_peaks_selected = np.copy(avg_peaks[[0, 1], :])
    pl = Polylidar3D(**polylidar_kwargs)
    avg_peaks_mat = MatrixDouble(avg_peaks_selected)

    tri_set = pl.extract_tri_set(tri_mesh, avg_peaks_mat)
    t0 = time.perf_counter()

    all_planes, all_polygons = pl.extract_planes_and_polygons_optimized(tri_mesh, avg_peaks_mat)
    t1 = time.perf_counter()
    polylidar_time = (t1 - t0) * 1000

    all_poly_lines = []
    for i in range(avg_peaks_selected.shape[0]):
        avg_peak = avg_peaks[i, :]
        rm, _ = R.align_vectors([[0, 0, 1]], [avg_peak])
        polygons_for_normal = all_polygons[i]
        # print(polygons_for_normal)
        if len(polygons_for_normal) > 0:
            poly_lines, _ = filter_and_create_open3d_polygons(vertices, polygons_for_normal, rm=rm)
            all_poly_lines.extend(poly_lines)

    return all_planes, tri_set, all_poly_lines, polylidar_time


def split_triangles(mesh):
    """
    Split the mesh in independent triangles    
    """
    triangles = np.asarray(mesh.triangles).copy()
    vertices = np.asarray(mesh.vertices).copy()

    triangles_3 = np.zeros_like(triangles)
    vertices_3 = np.zeros((len(triangles) * 3, 3), dtype=vertices.dtype)

    for index_triangle, t in enumerate(triangles):
        index_vertex = index_triangle * 3
        vertices_3[index_vertex] = vertices[t[0]]
        vertices_3[index_vertex + 1] = vertices[t[1]]
        vertices_3[index_vertex + 2] = vertices[t[2]]

        triangles_3[index_triangle] = np.arange(index_vertex, index_vertex + 3)

    mesh_return = deepcopy(mesh)
    mesh_return.triangles = o3d.utility.Vector3iVector(triangles_3)
    mesh_return.vertices = o3d.utility.Vector3dVector(vertices_3)
    mesh_return.triangle_normals = mesh.triangle_normals
    mesh_return.paint_uniform_color([0.5, 0.5, 0.5])
    return mesh_return


def assign_some_vertex_colors(mesh, triangle_indices, triangle_colors, mask=None):
    """Assigns vertex colors by given normal colors
    NOTE: New mesh is returned

    Arguments:
        mesh {o3d:TriangleMesh} -- Mesh
        normal_colors {ndarray} -- Normals Colors

    Returns:
        o3d:TriangleMesh -- New Mesh with painted colors
    """
    split_mesh = split_triangles(mesh)
    vertex_colors = np.asarray(split_mesh.vertex_colors)
    triangles = np.asarray(split_mesh.triangles)
    if mask is not None:
        triangles = triangles[mask, :]

    if isinstance(triangle_indices, list):
        for triangle_set, color in zip(triangle_indices, triangle_colors):
            triangle_set = np.asarray(triangle_set)
            for i in range(np.asarray(triangle_set).shape[0]):
                # import ipdb; ipdb.set_trace()
                t_idx = triangle_set[i]
                p_idx = triangles[t_idx, :]
                vertex_colors[p_idx] = color
    else:
        for i in range(triangle_indices.shape[0]):
            # import ipdb; ipdb.set_trace()
            t_idx = triangle_indices[i]
            color = triangle_colors[i, :]
            p_idx = triangles[t_idx, :]
            vertex_colors[p_idx] = color
    if not split_mesh.has_triangle_normals():
        split_mesh.compute_triangle_normals()
    split_mesh.compute_vertex_normals()

    return split_mesh


def paint_planes(o3d_mesh, planes):
    # colors = np.arange(0, 0+ len(planes))
    colors = [0, 3]
    all_colors = plt.cm.get_cmap('tab10')(colors)[:, :3]

    # planes_list = [np.copy(plane) for plane in planes]
    # planes_list = np.

    new_mesh = assign_some_vertex_colors(o3d_mesh, planes, all_colors)
    return new_mesh


def run_test(mesh, callback=None, stride=2):
    # Create Pseudo 3D Surface Mesh using Delaunay Triangulation and Polylidar
    polylidar_kwargs = dict(alpha=0.0, lmax=0.10, min_triangles=80,
                            z_thresh=0.06, norm_thresh=0.97, norm_thresh_min=0.92, min_hole_vertices=6)
    # Create Polylidar TriMesh
    tri_mesh = open_3d_mesh_to_trimesh(mesh)
    bilateral_filter_normals(tri_mesh, 3, 0.1, 0.1)
    vertices = np.asarray(tri_mesh.vertices)
    normals_smooth = np.asarray(tri_mesh.triangle_normals)
    mesh.triangle_normals = o3d.utility.Vector3dVector(normals_smooth)

    o3d.visualization.draw_geometries([mesh], width=600, height=500)

    planes, tri_set, all_poly_lines, polylidar_time = extract_all_dominant_planes(tri_mesh, vertices, polylidar_kwargs)
    time_polylidar3D = polylidar_time
    polylidar_3d_alg_name = 'Polylidar3D with Provided Mesh'

    planes_tri_set = [np.argwhere(np.asarray(tri_set) == i) for i in range(1, 3)]
    # import ipdb; ipdb.set_trace()
    mesh_tri_set = paint_planes(mesh, planes_tri_set)
    callback(polylidar_3d_alg_name, time_polylidar3D, mesh_tri_set)

    mesh_segment = paint_planes(mesh, planes)
    callback(polylidar_3d_alg_name, time_polylidar3D, mesh_segment)

    mesh_3d_polylidar = []
    mesh_3d_polylidar.extend(flatten([line_mesh.cylinder_segments for line_mesh in all_poly_lines]))
    mesh_3d_polylidar.append(mesh_segment)
    callback(polylidar_3d_alg_name, time_polylidar3D, mesh_3d_polylidar)


def callback(alg_name, execution_time, mesh=None):
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -0.7])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    logging.info("%s took (ms): %.2f", alg_name, execution_time)
    if mesh:
        if isinstance(mesh, list):
            o3d.visualization.draw_geometries(
                [*mesh, axis_frame], width=600, height=500)
        else:
            o3d.visualization.draw_geometries([mesh, axis_frame], width=600, height=500)


def main():
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -1.0])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-1.0, translate=[0, 0.0, 0.0])                            

    for i, mesh in enumerate(get_mesh_data_iterator()):
        if i < 2:
            continue
            # o3d.io.write_triangle_mesh('test.ply', mesh)

        run_test(mesh, callback=callback, stride=2)


if __name__ == "__main__":
    main()


"""
{
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : false,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 1.8801286101866053, 3.0496817724620651, 3.0527941421471674 ],
			"boundingbox_min" : [ -2.250705786558127, -3.7003182275379354, -0.71199999999999997 ],
			"field_of_view" : 60.0,
			"front" : [ -0.84642734312728896, -0.017445203308442493, 0.53221839285015882 ],
			"lookat" : [ -0.079533817840752988, -0.70163904564578894, 1.329269560817836 ],
			"up" : [ 0.5324286864188722, -0.044560013322916106, 0.8453011883884558 ],
			"zoom" : 0.13999999999999962
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}
"""

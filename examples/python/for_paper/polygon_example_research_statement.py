# This example requires a mesh that I have not distributed.

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
from examples.python.util.helper_polylidar import extract_all_dominant_plane_normals

from polylidar import (Polylidar3D, MatrixDouble, MatrixFloat, MatrixInt,
                       create_tri_mesh_copy, bilateral_filter_normals)

from polylidar.polylidarutil.open3d_util import construct_grid, create_lines, flatten
from polylidar.polylidarutil.plane_filtering import filter_planes_and_holes


from fastga import GaussianAccumulatorS2Beta, MatX3d, IcoCharts
from fastga.peak_and_cluster import find_peaks_from_accumulator


import open3d as o3d


def filter_and_create_open3d_polygons(points, polygons, rm=None, line_radius=0.005):
    " Apply polygon filtering algorithm, return Open3D Mesh Lines "
    config_pp = dict(filter=dict(hole_area=dict(min=0.01, max=100.0), hole_vertices=dict(min=4), plane_area=dict(min=0.05)),
                     positive_buffer=0.000, negative_buffer=0.01, simplify=0.01)
    # config_pp = dict(filter=dict(hole_area=dict(min=0.00, max=100.0), hole_vertices=dict(min=3), plane_area=dict(min=0.05)),
    #                  positive_buffer=0.00, negative_buffer=0.0, simplify=0.01)
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
    ga = GaussianAccumulatorS2Beta(level=4)
    ico = IcoCharts(level=4)

    fast_ga_kwargs = dict(find_peaks_kwargs=dict(threshold_abs=15, min_distance=1, exclude_border=False, indices=False),
                          cluster_kwargs=dict(t=0.28, criterion='distance'),
                          average_filter=dict(min_total_weight=0.1))

    avg_peaks, _, _, _, alg_timings = extract_all_dominant_plane_normals(
        tri_mesh, ga_=ga, ico_chart_=ico, **fast_ga_kwargs)
    logging.info("Dominant Plane Normals")
    print(avg_peaks)

    avg_peaks_selected = np.copy(avg_peaks[[0, 1, 2, 3, 4], :])
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
    polylidar_kwargs = dict(alpha=0.0, lmax=0.15, min_triangles=100,
                            z_thresh=0.20, norm_thresh=0.95, norm_thresh_min=0.90, min_hole_vertices=6)
    # Create Polylidar TriMesh
    tri_mesh = open_3d_mesh_to_trimesh(mesh)
    # bilateral_filter_normals(tri_mesh, 3, 0.1, 0.1)
    vertices = np.asarray(tri_mesh.vertices)
    normals_smooth = np.asarray(tri_mesh.triangle_normals)
    mesh.triangle_normals = o3d.utility.Vector3dVector(normals_smooth)
    o3d.visualization.draw_geometries([mesh], width=600, height=500)

    planes, tri_set, all_poly_lines, polylidar_time = extract_all_dominant_planes(tri_mesh, vertices, polylidar_kwargs)
    time_polylidar3D = polylidar_time
    polylidar_3d_alg_name = 'Polylidar3D with Provided Mesh'

    # planes_tri_set = [np.argwhere(np.asarray(tri_set) == i)  for i in range(1, 3)]
    # # import ipdb; ipdb.set_trace()
    # mesh_tri_set = paint_planes(mesh, planes_tri_set)
    # callback(polylidar_3d_alg_name, time_polylidar3D, mesh_tri_set)

    # mesh_segment = paint_planes(mesh, planes)
    # callback(polylidar_3d_alg_name, time_polylidar3D, mesh_segment)

    mesh_3d_polylidar = []
    mesh_3d_polylidar.extend(flatten([line_mesh.cylinder_segments for line_mesh in all_poly_lines]))
    mesh_3d_polylidar.append(mesh)
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

    mesh = o3d.io.read_triangle_mesh('fixtures/meshes/Table_edit.ply')
    mesh.compute_vertex_normals()

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
			"boundingbox_max" : [ 4.0542712211608887, 1.2899999618530273, 2.809999942779541 ],
			"boundingbox_min" : [ -2.2899999618530273, -1.0299999713897705, -2.5627658367156982 ],
			"field_of_view" : 60.0,
			"front" : [ 0.061353428751916628, -0.93672755075531344, 0.34464075852448922 ],
			"lookat" : [ 0.232231386497287, 0.6505503162493752, 0.45416176227377059 ],
			"up" : [ -0.006158775652966561, -0.34563996918805912, -0.93834699401774302 ],
			"zoom" : 0.25999999999999956
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}
"""

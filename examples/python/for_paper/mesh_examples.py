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

from polylidar.polylidarutil.open3d_util import construct_grid, create_lines, flatten, open_3d_mesh_to_trimesh
from polylidar.polylidarutil.plane_filtering import filter_planes_and_holes

from fastga import GaussianAccumulatorS2, MatX3d, IcoCharts
from fastga.peak_and_cluster import find_peaks_from_accumulator, find_peaks_from_ico_charts


import open3d as o3d


def filter_and_create_open3d_polygons(points, polygons, rm=None, line_radius=0.01,
                                      config_pp=dict(filter=dict(hole_area=dict(min=0.1, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.25)),
                                                     positive_buffer=0.02, negative_buffer=0.05, simplify=0.02)):
    " Apply polygon filtering algorithm, return Open3D Mesh Lines "
    # config_pp = dict(filter=dict(hole_area=dict(min=0.00, max=100.0), hole_vertices=dict(min=3), plane_area=dict(min=0.05)),
    #                  positive_buffer=0.00, negative_buffer=0.0, simplify=0.01)
    t1 = time.perf_counter()
    planes, obstacles = filter_planes_and_holes(polygons, points, config_pp, rm=rm)
    t2 = time.perf_counter()
    logging.info("Plane Filtering Took (ms): %.2f", (t2 - t1) * 1000)
    all_poly_lines = create_lines(planes, obstacles, line_radius=line_radius)
    return all_poly_lines, (t2 - t1) * 1000


def down_sample_normals(triangle_normals, down_sample_fraction=0.12, min_samples=10000, flip_normals=False, **kwargs):
    num_normals = triangle_normals.shape[0]
    to_sample = int(down_sample_fraction * num_normals)
    to_sample = max(min([num_normals, min_samples]), to_sample)
    ds_step = int(num_normals / to_sample)
    triangle_normals_ds = np.ascontiguousarray(triangle_normals[:num_normals:ds_step, :])
    if flip_normals:
        triangle_normals_ds = triangle_normals_ds * -1.0
    return triangle_normals_ds

def get_image_peaks(ico_chart, ga, level=2, with_o3d=False,
                    find_peaks_kwargs=dict(threshold_abs=15, min_distance=1, exclude_border=True, indices=False),
                    cluster_kwargs=dict(t=0.10, criterion='distance'),
                    average_filter=dict(min_total_weight=0.01),
                    **kwargs):

    normalized_bucket_counts_by_vertex = ga.get_normalized_bucket_counts_by_vertex(True)

    t1 = time.perf_counter()
    ico_chart.fill_image(normalized_bucket_counts_by_vertex)  # this takes microseconds
    # plt.imshow(np.asarray(ico_chart.image))
    # plt.show()
    peaks, clusters, avg_peaks, avg_weights = find_peaks_from_ico_charts(ico_chart, np.asarray(
        normalized_bucket_counts_by_vertex), find_peaks_kwargs=find_peaks_kwargs, cluster_kwargs=cluster_kwargs, average_filter=average_filter)
    t2 = time.perf_counter()

    gaussian_normals_sorted = np.asarray(ico_chart.sphere_mesh.vertices)
    # Create Open3D structures for visualization
    if with_o3d:
        pcd_all_peaks = get_pc_all_peaks(peaks, clusters, gaussian_normals_sorted)
        arrow_avg_peaks = get_arrow_normals(avg_peaks, avg_weights)
    else:
        pcd_all_peaks = None
        arrow_avg_peaks = None

    elapsed_time = (t2 - t1) * 1000
    timings = dict(t_fastga_peak=elapsed_time)

    logging.debug("Peak Detection - Took (ms): %.2f", (t2 - t1) * 1000)


    return avg_peaks, pcd_all_peaks, arrow_avg_peaks, timings


def extract_all_dominant_planes(tri_mesh, vertices, polylidar_kwargs, config_pp, ds=50, min_samples=10000):
    ga = GaussianAccumulatorS2(level=4, max_phi=180)
    ico = IcoCharts(level=4)

    triangle_normals = np.asarray(tri_mesh.triangle_normals)
    num_normals = triangle_normals.shape[0]
    triangle_normals_ds = down_sample_normals(triangle_normals)


    # Get the data
    t0 = time.perf_counter()
    ga.integrate(MatX3d(triangle_normals_ds))
    t1 = time.perf_counter()
    avg_peaks, _, _, timings = get_image_peaks(ico, ga, level=4, with_o3d=False)
    timings['t_fastga_integrate'] = (t1-t0) * 1000
    timings['t_fastga_total'] = timings['t_fastga_integrate'] + timings['t_fastga_peak']


    logging.info("Processing mesh with %d triangles", num_normals)
    logging.info("Dominant Plane Normals")
    print(avg_peaks)

    avg_peaks_selected = np.copy(avg_peaks[[0, 1, 2, 3], :])
    pl = Polylidar3D(**polylidar_kwargs)
    avg_peaks_mat = MatrixDouble(avg_peaks_selected)

    # debugging purposes, ignore
    tri_set = pl.extract_tri_set(tri_mesh, avg_peaks_mat)

    t0 = time.perf_counter()
    all_planes, all_polygons = pl.extract_planes_and_polygons_optimized(tri_mesh, avg_peaks_mat)
    t1 = time.perf_counter()
    timings['t_polylidar_planepoly'] = (t1 - t0) * 1000

    all_poly_lines = []
    for i in range(avg_peaks_selected.shape[0]):
        avg_peak = avg_peaks[i, :]
        rm, _ = R.align_vectors([[0, 0, 1]], [avg_peak])
        polygons_for_normal = all_polygons[i]
        # print(polygons_for_normal)
        if len(polygons_for_normal) > 0:
            poly_lines, _ = filter_and_create_open3d_polygons(vertices, polygons_for_normal, rm=rm, config_pp=config_pp)
            all_poly_lines.extend(poly_lines)

    return all_planes, tri_set, all_poly_lines, timings

def run_test(mesh, callback=None, polylidar_kwargs=None, config_pp=None):
    tri_mesh = open_3d_mesh_to_trimesh(mesh)
    # bilateral_filter_normals(tri_mesh, 3, 0.1, 0.1)
    vertices = np.asarray(tri_mesh.vertices)
    normals_smooth = np.asarray(tri_mesh.triangle_normals)
    mesh.triangle_normals = o3d.utility.Vector3dVector(normals_smooth)

    planes, tri_set, all_poly_lines, timings = extract_all_dominant_planes(
        tri_mesh, vertices, polylidar_kwargs, config_pp)
    time_polylidar3D = timings['t_polylidar_planepoly']
    polylidar_3d_alg_name = 'Polylidar3D with Provided Mesh'

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
                [*mesh], width=800, height=500)
        else:
            o3d.visualization.draw_geometries([mesh], width=800, height=500)


def main():
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -1.0])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-1.0, translate=[0, 0.0, 0.0])

    polylidar_kwargs_basement = dict(alpha=0.0, lmax=0.10, min_triangles=80,
                                     z_thresh=0.08, norm_thresh=0.95, norm_thresh_min=0.95, min_hole_vertices=6)

    config_pp_basement = dict(filter=dict(hole_area=dict(min=0.05, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.06)),
                              positive_buffer=0.0, negative_buffer=0.025, simplify=0.01)

    polylidar_kwargs_mainfloor = dict(alpha=0.0, lmax=0.10, min_triangles=1000,
                                      z_thresh=0.08, norm_thresh=0.95, norm_thresh_min=0.95, min_hole_vertices=6)

    config_pp_main_floor = dict(filter=dict(hole_area=dict(min=0.1, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.25)),
                                positive_buffer=0.02, negative_buffer=0.05, simplify=0.02)

    polylidar_kwargs = [polylidar_kwargs_mainfloor, None, polylidar_kwargs_basement]
    config_pp = [config_pp_main_floor, None, config_pp_basement]

    for i, mesh in enumerate(get_mesh_data_iterator()):
        if i == 1:
            continue
        run_test(mesh, callback=callback, polylidar_kwargs=polylidar_kwargs[i], config_pp=config_pp[i])


if __name__ == "__main__":
    main()


"""
MainFloor View 1
{
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : false,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 4.6879752961101335, 4.3041731602087356, 1.1239513038042122 ],
			"boundingbox_min" : [ -1.7143359241036764, -4.4827524540588648, -1.2944623277755183 ],
			"field_of_view" : 60.0,
			"front" : [ 0.53039465128384522, -0.61345881428120275, 0.58510665444018772 ],
			"lookat" : [ 1.1083255653557145, 0.45595517159720361, -1.4139722312039875 ],
			"up" : [ -0.37208261429501627, 0.45169957452210624, 0.81087731656270567 ],
			"zoom" : 0.25999999999999956
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}


MainFloor View 2
{
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : false,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 4.6879752961101335, 4.3041731602087356, 1.1239513038042122 ],
			"boundingbox_min" : [ -1.7143359241036764, -4.4827524540588648, -1.2944623277755183 ],
			"field_of_view" : 60.0,
			"front" : [ 0.33172820056487995, -0.73579018685271158, 0.59039749481280723 ],
			"lookat" : [ 0.01577387102866264, 1.2098965809101954, -0.32128930881301754 ],
			"up" : [ -0.24179049551880405, 0.53861595454388955, 0.80711226591320351 ],
			"zoom" : 0.099999999999999617
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}



Basement View 1
{
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : false,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 1.8764505760969685, 3.0280973667097442, 3.045776668203259 ],
			"boundingbox_min" : [ -2.2365574934452548, -3.6804227036671078, -0.71199999999999997 ],
			"field_of_view" : 60.0,
			"front" : [ -0.70399218989352985, 0.54906926280507251, 0.45046413976209737 ],
			"lookat" : [ 0.2914599315270186, -0.61171091088701091, 1.5318870940033267 ],
			"up" : [ 0.33399643257519951, -0.30379868815633204, 0.89227391539903933 ],
			"zoom" : 0.16
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}

Basement View 2
{
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : false,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 1.8764505760969685, 3.0280973667097442, 3.045776668203259 ],
			"boundingbox_min" : [ -2.2365574934452548, -3.6804227036671078, -0.71199999999999997 ],
			"field_of_view" : 60.0,
			"front" : [ 0.81538935556246117, 0.54274210251126309, 0.20142296045160274 ],
			"lookat" : [ -0.94542479921885303, -1.0842317654700471, 1.650916470745605 ],
			"up" : [ -0.17901684954803626, -0.094491727278059712, 0.97929785104119604 ],
			"zoom" : 0.14000000000000001
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}
"""

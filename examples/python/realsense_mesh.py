import time
import math
import sys
from os import path, listdir
from os.path import exists, isfile, join, splitext
import re
import logging
import pickle

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from polylidar import extractPlanesAndPolygons, extract_planes_and_polygons_from_mesh, extract_point_cloud_from_float_depth, extract_uniform_mesh_from_float_depth
from polylidarutil import (plot_polygons_3d, generate_3d_plane, set_axes_equal, plot_planes_3d,
                           scale_points, rotation_matrix, apply_rotation, COLOR_PALETTE)
from polylidarutil.open3d_util import construct_grid, create_lines, flatten
from polylidarutil.plane_filtering import filter_planes_and_holes

from scipy.spatial import Delaunay
from scipy.stats import describe
import open3d as o3d

DIR_NAME = path.dirname(__file__)
FIXTURES_DIR = path.join(DIR_NAME, '../../tests', 'fixtures')
REALSENSE_DIR = path.join(FIXTURES_DIR, 'realsense')
REALSENSE_COLOR_DIR = path.join(REALSENSE_DIR, 'color')
REALSENSE_DEPTH_DIR = path.join(REALSENSE_DIR, 'depth')
REALSENSE_SCENE_DIR = path.join(REALSENSE_DIR, 'scene')
REALSENSE_TRAJECTORY = path.join(REALSENSE_SCENE_DIR, 'trajectory.log')
REALSENSE_INTRINSICS = path.join(
    REALSENSE_DIR, 'camera_intrinsic_rgb_424.json')


# Conversion between D400 to T265
H_t265_d400 = np.array([
    [1, 0, 0, 0],
    [0, -1.0, 0, 0],
    [0, 0, -1.0, 0],
    [0, 0, 0, 1]])

# Conversion from D400 frame to "Standard" frame (z-axis= to [0,0,1])
R_Standard_d400 = np.array([
    [1, 0, 0, 0],
    [0, 0.0, 1.0, 0],
    [0, -1.0, 0, 0],
    [0, 0, 0, 1]])


logging.basicConfig(level=logging.INFO)


### Read Saved Data from Realsense Directory ###
# Some of this code is pulled from Open3D to read saved RGBD Files
def sorted_alphanum(file_list_ordered):
    def convert(text): return int(text) if text.isdigit() else text
    def alphanum_key(key): return [convert(c)
                                   for c in re.split('([0-9]+)', key)]
    return sorted(file_list_ordered, key=alphanum_key)


def get_file_list(path_, extension=None):
    if extension is None:
        file_list = [path.join(path_, f)
                     for f in listdir(path_) if isfile(join(path_, f))]
    else:
        file_list = [
            path.join(path_, f)
            for f in listdir(path_)
            if isfile(join(path_, f)) and splitext(f)[1] == extension
        ]
    file_list = sorted_alphanum(file_list)
    file_list = [path.abspath(file_) for file_ in file_list]
    return file_list


def get_rgbd_file_lists(path_color=REALSENSE_COLOR_DIR, path_depth=REALSENSE_DEPTH_DIR):
    color_files = get_file_list(path_color, ".jpg") + \
        get_file_list(path_color, ".png")
    depth_files = get_file_list(path_depth, ".png")
    return color_files, depth_files


def read_trajectory(filename):
    traj = []
    with open(filename, 'r') as f:
        metastr = f.readline()
        while metastr:
            mat = np.zeros(shape=(4, 4))
            for i in range(4):
                matstr = f.readline()
                mat[i, :] = np.fromstring(matstr, dtype=float, sep=' \t')
            traj.append(mat)
            metastr = f.readline()
    return traj


def convert_trajectory(traj, inv=True):
    new_traj = []
    for i in range(len(traj)):
        extrinsic_1 = np.linalg.inv(H_t265_d400) @ traj[i] @ H_t265_d400
        if inv:
            extrinsic_1 = np.linalg.inv(extrinsic_1)
        new_traj.append(extrinsic_1)
    return new_traj


def get_realsense_data(color_dir=REALSENSE_COLOR_DIR, depth_dir=REALSENSE_DEPTH_DIR,
                       trajectory_fpath=REALSENSE_TRAJECTORY, intrinsic_fpath=REALSENSE_INTRINSICS):
    color_files, depth_files = get_rgbd_file_lists(
        path_color=color_dir, path_depth=depth_dir)
    traj = convert_trajectory(read_trajectory(trajectory_fpath))
    intrinsics = o3d.io.read_pinhole_camera_intrinsic(intrinsic_fpath)
    return color_files, depth_files, traj, intrinsics

### End Read Saved Data from Realsense Directory ###


def create_open3d_pc(points):
    """ Creates an Open3D point cloud """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


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
    for i, plane in enumerate(planes):
        if color is None:
            color_ = COLOR_PALETTE[i % (len(COLOR_PALETTE) - 1)]
        else:
            color_ = COLOR_PALETTE[0]
        tris = np.ascontiguousarray(np.flip(triangles[plane, :], 1))
        mesh = create_open_3d_mesh(tris, points, color_)
        meshes.append(mesh)
    return meshes


def get_frame_data(idx, color_files, depth_files, traj, intrinsic, depth_trunc=3.0, stride=2):
    """Gets Frame Data

    Arguments:
        idx {int} -- Index of frame
        color_files {list} -- list of color images
        depth_files {list} -- list of depth images
        traj {list} -- list of extrinsic matrices corresponding to frames
        intrinsic {Open3D intrisics} -- Open3D intrinsics array

    Keyword Arguments:
        depth_trunc {float} -- How much to truncate depth image in meters (default: {3.0})
        stride {int} -- stride for downsampling pont cloud (default: {2})

    Returns:
        tuple -- PointCloud, RGBD Image, extrinsics (D4XX Frame -> Global Standard Frame)
    """
    depth_1 = o3d.io.read_image(depth_files[idx])
    color_1 = o3d.io.read_image(color_files[idx])

    extrinsic = traj[idx]
    rgbd_image_1 = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_1, depth_1, convert_rgb_to_intensity=False, depth_trunc=depth_trunc)

    pcd_1 = o3d.geometry.PointCloud.create_from_depth_image(
        depth_1, intrinsic, extrinsic, stride=stride, depth_trunc=depth_trunc)
    return pcd_1, rgbd_image_1, R_Standard_d400 @ np.linalg.inv(extrinsic)


def prep_mesh(mesh):
    " Prepares mesh for visualization "
    mesh_list = mesh
    if not isinstance(mesh_list, list):
        mesh_list = [mesh]
    for mesh_ in mesh_list:
        mesh_.compute_triangle_normals()
        mesh_.paint_uniform_color(COLOR_PALETTE[0])


def filter_and_create_open3d_polygons(points, polygons):
    " Apply polygon filtering algorithm, return Open3D Mesh Lines "
    config_pp = dict(filter=dict(hole_area=dict(min=0.025, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.5)),
                     positive_buffer=0.00, negative_buffer=0.02, simplify=0.01)
    planes, obstacles = filter_planes_and_holes(polygons, points, config_pp)
    all_poly_lines = create_lines(planes, obstacles, line_radius=0.01)
    return all_poly_lines


def run_test(pcd, rgbd, intrinsics, extrinsics, bp_alg=dict(radii=[0.02, 0.02]), poisson=dict(depth=8), callback=None, stride=2):
    points = np.asarray(pcd.points)
    # Create Pseudo 3D Surface Mesh using Delaunay Triangulation and Polylidar
    polylidar_kwargs = dict(alpha=0.0, lmax=0.10, minTriangles=100,
                            zThresh=0.03, normThresh=0.99, normThreshMin=0.95, minHoleVertices=6)
    t1 = time.perf_counter()
    delaunay, planes, polygons = extractPlanesAndPolygons(points, **polylidar_kwargs)
    t2 = time.perf_counter()
    all_poly_lines = filter_and_create_open3d_polygons(points, polygons)
    triangles = np.asarray(delaunay.triangles).reshape(int(len(delaunay.triangles) / 3), 3)
    mesh_2d_polylidar = extract_mesh_planes(points, triangles, planes, COLOR_PALETTE[0])
    mesh_2d_polylidar.extend(flatten([line_mesh.cylinder_segments for line_mesh in all_poly_lines]))
    time_mesh_2d_polylidar = (t2 - t1) * 1000
    polylidar_alg_name = 'Polylidar2D'
    callback(polylidar_alg_name, time_mesh_2d_polylidar, pcd, mesh_2d_polylidar)
    # Uniform Mesh Grid
    polylidar_inputs, timings = make_uniform_grid_mesh(np.asarray(
        rgbd.depth), np.ascontiguousarray(intrinsics.intrinsic_matrix), extrinsics, stride=stride)
    mesh_uniform_grid = create_open_3d_mesh(polylidar_inputs['triangles'], polylidar_inputs['vertices'])
    time_mesh_uniform = timings['mesh_creation']
    uniform_alg_name = 'Uniform Grid Mesh'
    callback(uniform_alg_name, time_mesh_uniform, pcd, mesh_uniform_grid)
    # Polylidar3D with Uniform Mesh Grid
    # pickle.dump(polylidar_inputs, open('realsense_mesh.pkl', 'wb'))
    vertices = polylidar_inputs['vertices']
    triangles = polylidar_inputs['triangles']
    halfedges = polylidar_inputs['halfedges']
    t1 = time.perf_counter()
    planes, polygons = extract_planes_and_polygons_from_mesh(vertices, triangles, halfedges, **polylidar_kwargs)
    t2 = time.perf_counter()
    all_poly_lines = filter_and_create_open3d_polygons(vertices, polygons)
    triangles = triangles.reshape(int(triangles.shape[0] / 3), 3)
    mesh_3d_polylidar = extract_mesh_planes(vertices, triangles, planes)
    mesh_3d_polylidar.extend(flatten([line_mesh.cylinder_segments for line_mesh in all_poly_lines]))
    time_polylidar3D = (t2 - t1) * 1000
    polylidar_3d_alg_name = 'Polylidar with Uniform Grid Mesh'
    callback(polylidar_3d_alg_name, time_polylidar3D,
             create_open3d_pc(vertices), mesh_3d_polylidar)

    # Estimate Point Cloud Normals
    t3 = time.perf_counter()
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.10, max_nn=20))
    t4 = time.perf_counter()
    time_estimate_point_normals = (t4 - t3) * 1000
    point_normal_alg_name = 'Point Normal Estimation'
    callback(point_normal_alg_name, time_estimate_point_normals, pcd, None)
    # Create True 3D Surface Mesh using Ball Pivot Algorithm
    radii = o3d.utility.DoubleVector(bp_alg['radii'])
    t5 = time.perf_counter()
    mesh_ball_pivot = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, radii)
    prep_mesh(mesh_ball_pivot)
    t6 = time.perf_counter()
    time_mesh_ball_pivot = (t6 - t5) * 1000
    ball_point_alg_name = 'Ball Pivot'
    callback(ball_point_alg_name, time_mesh_ball_pivot, pcd, mesh_ball_pivot)
    # Create True 3D Surface Mesh using Poisson Reconstruction Algorithm
    t7 = time.perf_counter()
    mesh_poisson, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, **poisson)
    vertices_to_remove = densities < np.quantile(densities, 0.1)
    mesh_poisson.remove_vertices_by_mask(vertices_to_remove)
    t8 = time.perf_counter()
    prep_mesh(mesh_poisson)
    time_mesh_poisson = (t8 - t7) * 1000
    poisson_alg_name = 'Poisson'
    callback(poisson_alg_name, time_mesh_poisson, pcd, mesh_poisson)

    results = [
        dict(alg=polylidar_alg_name, mesh=mesh_2d_polylidar,
             execution_time=time_mesh_2d_polylidar),
        dict(alg=point_normal_alg_name, mesh=None,
             execution_time=time_estimate_point_normals),
        dict(alg=ball_point_alg_name, mesh=mesh_ball_pivot,
             execution_time=time_mesh_ball_pivot),
        dict(alg=poisson_alg_name, mesh=mesh_poisson,
             execution_time=time_mesh_poisson)
    ]
    return results


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
    points, triangles, halfedges = extract_uniform_mesh_from_float_depth(im, intrinsics, stride=stride)
    t1 = time.perf_counter()
    points = np.asarray(points)
    triangles = np.asarray(triangles)
    halfedges = np.asarray(halfedges)
    points = points.reshape((int(points.shape[0] / 3), 3))
    # Rotate Point Cloud
    points = np.column_stack((points, np.ones(points.shape[0])))
    points = np.ascontiguousarray(((extrinsics @ points.T).T)[:, :3])
    t2 = time.perf_counter()
    polylidar_inputs = dict(
        vertices=points, triangles=triangles, halfedges=halfedges)
    timings = dict(mesh_creation=(t1 - t0) * 1000, pc_rotation=(t2 - t1) * 1000)
    return polylidar_inputs, timings


def callback(alg_name, execution_time, pcd, mesh=None):
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -0.7])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    logging.info("%s took %.2f milliseconds", alg_name, execution_time)
    if mesh:
        if isinstance(mesh, list):
            o3d.visualization.draw_geometries(
                [*mesh, pcd, grid_ls, axis_frame])
        else:
            o3d.visualization.draw_geometries([mesh, pcd, grid_ls, axis_frame])


def main():
    color_files, depth_files, traj, intrinsics = get_realsense_data()
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -0.7])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    for idx in range(len(color_files)):
        if idx < 3:
            continue
        pcd, rgbd, extrinsics = get_frame_data(idx, color_files, depth_files, traj, intrinsics, stride=3)
        pcd = pcd.rotate(R_Standard_d400[:3, :3], center=False)

        logging.info("File %r - Point Cloud; Size: %r", idx, np.asarray(pcd.points).shape[0])
        o3d.visualization.draw_geometries([pcd, grid_ls, axis_frame])
        results = run_test(pcd, rgbd, intrinsics, extrinsics, callback=callback, stride=3)


if __name__ == "__main__":
    main()

import time
import math
from os import path, listdir
from os.path import exists, isfile, join, splitext
import re
import logging

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from polylidar import extractPlanesAndPolygons
from polylidarutil import (plot_polygons_3d, generate_3d_plane, set_axes_equal, plot_planes_3d,
                           scale_points, rotation_matrix, apply_rotation, COLOR_PALETTE)
from polylidarutil.open3d_util import construct_grid

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
REALSENSE_INTRINSICS = path.join(REALSENSE_DIR, 'camera_intrinsic_rgb_424.json')


H_t265_d400 = np.array([
    [1, 0, 0, 0],
    [0, -1.0, 0, 0],
    [0, 0, -1.0, 0],
    [0, 0, 0, 1]])

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
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def create_open_3d_mesh(triangles, points, color=[1, 0, 0]):
    mesh_2d = o3d.geometry.TriangleMesh()
    mesh_2d.triangles = o3d.utility.Vector3iVector(triangles)
    mesh_2d.vertices = o3d.utility.Vector3dVector(points)
    mesh_2d.compute_vertex_normals()
    mesh_2d.compute_triangle_normals()
    mesh_2d.paint_uniform_color(color)
    return mesh_2d


def extract_mesh_planes(points, delaunay, planes, color=None):
    triangles = np.asarray(delaunay.triangles).reshape(
        int(len(delaunay.triangles) / 3), 3)
    meshes = []
    for i, plane in enumerate(planes):
        if color is None:
            color = COLOR_PALETTE[i]
        tris = np.ascontiguousarray(np.flip(triangles[plane, :], 1))
        mesh = create_open_3d_mesh(tris, points, color)
        meshes.append(mesh)
    return meshes


def get_colored_point_cloud(idx, color_files, depth_files, traj, intrinsic, depth_trunc=3.0, stride=2):
    depth_1 = o3d.io.read_image(depth_files[idx])
    color_1 = o3d.io.read_image(color_files[idx])

    # depth_np = np.ascontiguousarray(np.asarray(depth_1)[::stride, ::stride])
    # color_np = np.ascontiguousarray(np.asarray(color_1)[::stride, ::stride])
    # print(depth_np.dtype)
    # print(depth_np.shape)
    # depth_1 = o3d.geometry.Image(depth_np)
    # color_1 = o3d.geometry.Image(color_np)
    extrinsic = traj[idx]
    rgbd_image_1 = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_1, depth_1, convert_rgb_to_intensity=False, depth_trunc=depth_trunc)

    # pcd_1 = o3d.geometry.PointCloud.create_from_rgbd_image(
    #     rgbd_image_1, intrinsic=intrinsic, extrinsic=extrinsic)
    pcd_1 = o3d.geometry.PointCloud.create_from_depth_image(depth_1, intrinsic, extrinsic, stride=stride, depth_trunc=depth_trunc)
    return pcd_1, rgbd_image_1, R_Standard_d400 @ np.linalg.inv(extrinsic)

def prep_mesh(mesh):
    mesh_list = mesh
    if not isinstance(mesh_list, list):
        mesh_list = [mesh]
    for mesh_ in mesh_list:
        mesh_.compute_triangle_normals()
        mesh_.paint_uniform_color(COLOR_PALETTE[0])

def run_test(pcd, rgbd, intrinsics, extrinsics, bp_alg=dict(radii=[0.02, 0.02]), poisson=dict(depth=8), callback=None):
    points = np.asarray(pcd.points)
    t1 = time.perf_counter()
    # Create Pseudo 3D Surface Mesh using Delaunay Triangulation and Polylidar
    delaunay, planes, polygons = extractPlanesAndPolygons(
        points, alpha=0.0, lmax=0.10, minTriangles=20, zThresh=0.03, normThresh=0.98, normThreshMin=0.90)
    t2 = time.perf_counter()
    mesh_2d_polylidar = extract_mesh_planes(points, delaunay, planes, COLOR_PALETTE[0])
    time_mesh_2d_polylidar = (t2 - t1) * 1000
    polylidar_alg_name = 'Polylidar2D'
    callback(polylidar_alg_name, time_mesh_2d_polylidar, pcd, mesh_2d_polylidar)
    # Uniform Mesh Grid
    t1 = time.perf_counter()
    mesh_uniform_grid = make_uniform_grid_mesh(rgbd, intrinsics, extrinsics)
    t2 = time.perf_counter()
    prep_mesh(mesh_uniform_grid)
    time_mesh_uniform = (t2 - t1) * 1000
    uniform_alg_name = 'UniformGrid'
    callback(uniform_alg_name, time_mesh_uniform, pcd, mesh_uniform_grid)

    # Estimate Point Cloud Normals
    t3 = time.perf_counter()
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.10, max_nn=20))
    t4 = time.perf_counter()
    time_estimate_point_normals = (t4 - t3) * 1000
    point_normal_alg_name = 'Point Normal Estimation'
    callback(point_normal_alg_name, time_estimate_point_normals, pcd, None)
    # Create True 3D Surface Mesh using Ball Pivot Algorithm
    radii = o3d.utility.DoubleVector(bp_alg['radii'])
    t5 = time.perf_counter()
    mesh_ball_pivot = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, radii)
    prep_mesh(mesh_ball_pivot)
    t6 = time.perf_counter()
    time_mesh_ball_pivot = (t6 - t5) * 1000
    ball_point_alg_name = 'Ball Pivot'
    callback(ball_point_alg_name, time_mesh_ball_pivot, pcd, mesh_ball_pivot)
    # Create True 3D Surface Mesh using Poisson Reconstruction Algorithm
    t7 = time.perf_counter()
    mesh_poisson, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, **poisson)
    vertices_to_remove = densities < np.quantile(densities, 0.1)
    mesh_poisson.remove_vertices_by_mask(vertices_to_remove)
    t8 = time.perf_counter()
    prep_mesh(mesh_poisson)
    time_mesh_poisson = (t8 - t7) * 1000
    poisson_alg_name = 'Poisson'
    callback(poisson_alg_name, time_mesh_poisson, pcd, mesh_poisson)

    results = [
        dict(alg=polylidar_alg_name, mesh=mesh_2d_polylidar, execution_time=time_mesh_2d_polylidar),
        dict(alg=point_normal_alg_name, mesh=None, execution_time=time_estimate_point_normals),
        dict(alg=ball_point_alg_name, mesh=mesh_ball_pivot, execution_time=time_mesh_ball_pivot),
        dict(alg=poisson_alg_name, mesh=mesh_poisson, execution_time=time_mesh_poisson)
    ]
    return results

def get_point(i, j, im, intrinsics, extrinsics):
    z = im[i,j]
    pp = intrinsics.get_principal_point()
    fl = intrinsics.get_focal_length()
    x = (j - pp[0]) * z / fl[0]
    y = (i - pp[1]) * z / fl[1]
    return [x,y,z]
    # point = (extrinsics @ np.array([[x,y,z,1]]).T)[:3,0].tolist()
    # return point, z

def make_uniform_grid_mesh(rgbd_image, intrinsics, extrinsics, stride=2):
    depth = rgbd_image.depth
    im = np.asarray(depth)
    rows = im.shape[0]
    cols = im.shape[1]
    triangles = []
    points = []
    for i in range(0,rows, stride):
        for j in range(0, cols, stride):
            p1 = get_point(i,j,im, intrinsics, extrinsics)
            points.append(p1)

    cols_pseudo = cols //stride + 1

    for i in range(0, rows - stride, stride):
        for j in range(0, cols - stride, stride):
            p1_idx = (i//stride) * cols_pseudo + (j//stride)
            p2_idx = (i//stride) * cols_pseudo + ((j+stride)//stride)
            p3_idx = ((i+stride)//stride) * cols_pseudo + ((j+stride)//stride)
            p4_idx = ((i+stride)//stride) * cols_pseudo + (j//stride)

            p1 = points[p1_idx]
            p2 = points[p2_idx]
            p3 = points[p3_idx]
            p4 = points[p4_idx]
            
            if p1[2] > 0 and p2[2] > 0 and p3[2] > 0:
                triangles.append([p1_idx, p2_idx, p3_idx])
            if p3[2] > 0 and p4[2] > 0 and p1[2] > 0:
                triangles.append([p3_idx, p4_idx, p1_idx])

    points = np.array(points)
    points = np.column_stack((points, np.ones(points.shape[0])))
    points = np.ascontiguousarray(((extrinsics @ points.T).T)[:,:3])
    triangles = np.array(triangles)

    mesh = create_open_3d_mesh(triangles, points)
    return mesh



def callback(alg_name, execution_time, pcd, mesh=None):
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    axis_frame.translate([0, 0.8, -0.8])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    logging.info("%r took %.2f milliseconds", alg_name, execution_time)
    if mesh:
        if isinstance(mesh, list):
            o3d.visualization.draw_geometries([*mesh, pcd, grid_ls, axis_frame])
        else:
            o3d.visualization.draw_geometries([mesh, pcd, grid_ls, axis_frame])

def main():
    color_files, depth_files, traj, intrinsics = get_realsense_data()
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    axis_frame.translate([0, 0.8, -0.8])
    grid_ls = construct_grid(size=2, n=200, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    for idx in range(len(color_files)):
        if idx < 2:
            continue
        pcd, rgbd, extrinsics = get_colored_point_cloud(
            idx, color_files, depth_files, traj, intrinsics)
        pcd = pcd.rotate(R_Standard_d400[:3,:3], center=False)
        # pcd = pcd.voxel_down_sample(voxel_size=0.02)

        logging.info("File %r - Point Cloud; Size: %r", idx, np.asarray(pcd.points).shape[0])
        o3d.visualization.draw_geometries([pcd, grid_ls, axis_frame])
        results = run_test(pcd, rgbd, intrinsics, extrinsics, callback=callback)

            


if __name__ == "__main__":
    main()

import time
import math
import sys
from os import path, listdir
from os.path import exists, isfile, join, splitext
import re
import logging

import numpy as np
from polylidar.polylidarutil import COLOR_PALETTE
import open3d as o3d

DIR_NAME = path.dirname(__file__)
FIXTURES_DIR = path.join(DIR_NAME, '../../..', 'fixtures')
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


def create_open_3d_mesh(triangles, points, triangle_normals=None, color=COLOR_PALETTE[0], counter_clock_wise=True):
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
    if points.ndim == 1:
        points = points.reshape((int(points.shape[0] / 3), 3))
    if triangles.ndim == 1:
        triangles = triangles.reshape((int(triangles.shape[0] / 3), 3))
        # Open 3D expects triangles to be counter clockwise
    if not counter_clock_wise:
        triangles = np.ascontiguousarray(np.flip(triangles, 1))
    mesh_2d.triangles = o3d.utility.Vector3iVector(triangles)
    mesh_2d.vertices = o3d.utility.Vector3dVector(points)
    if triangle_normals is None:
        mesh_2d.compute_vertex_normals()
        mesh_2d.compute_triangle_normals()
    elif triangle_normals.ndim == 1:
        triangle_normals_ = triangle_normals.reshape((int(triangle_normals.shape[0] / 3), 3))
        mesh_2d.triangle_normals = o3d.utility.Vector3dVector(triangle_normals_)
    mesh_2d.paint_uniform_color(color)
    return mesh_2d


def extract_mesh_planes(points, triangles, planes, counter_clock_wise=True, color=None):
    " Converts Polylidar Mesh Planes into Open3D format "

    meshes = []
    color_ = color
    for i, plane in enumerate(planes):
        if color is None:
            color_ = COLOR_PALETTE[i % (len(COLOR_PALETTE) - 1)]
        else:
            color_ = COLOR_PALETTE[0]
        # tris = np.ascontiguousarray(np.flip(triangles[plane, :], 1))
        tris = np.ascontiguousarray(triangles[plane, :])
        mesh = create_open_3d_mesh(tris, points, color=color_, counter_clock_wise=counter_clock_wise)
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

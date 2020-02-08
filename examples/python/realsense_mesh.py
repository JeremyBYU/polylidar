import time
import math
import sys
from os import path, listdir
from os.path import exists, isfile, join, splitext
import re
import logging

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


def extract_mesh_planes(points, triangles, planes, color=None):
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
    depth_1 = o3d.io.read_image(depth_files[idx])
    color_1 = o3d.io.read_image(color_files[idx])


    extrinsic = traj[idx]
    rgbd_image_1 = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_1, depth_1, convert_rgb_to_intensity=False, depth_trunc=depth_trunc)

    # intrinsic_np =  np.ascontiguousarray(np.asarray(intrinsic.intrinsic_matrix))
    # print(intrinsic_np.dtype, intrinsic_np.shape, intrinsic_np.flags)
    # print(intrinsic_np)
    # depth_np = np.asarray(rgbd_image_1.depth)
    # t0 = time.perf_counter()
    # points = extract_point_cloud_from_float_depth(depth_np, intrinsic_np)
    # t1 = time.perf_counter()
    # print(t1-t0)
    # points = np.asarray(points)
    # print(points[:10])
    # pointer, read_only_flag = points.__array_interface__['data']
    # print("Data address", hex(pointer))
    # points = points.reshape(int(points.shape[0]/3), 3)
    # sys.exit(0)

    pcd_1 = o3d.geometry.PointCloud.create_from_depth_image(
        depth_1, intrinsic, extrinsic, stride=stride, depth_trunc=depth_trunc)
    return pcd_1, rgbd_image_1, R_Standard_d400 @ np.linalg.inv(extrinsic)


def prep_mesh(mesh):
    mesh_list = mesh
    if not isinstance(mesh_list, list):
        mesh_list = [mesh]
    for mesh_ in mesh_list:
        mesh_.compute_triangle_normals()
        mesh_.paint_uniform_color(COLOR_PALETTE[0])


def filter_and_create_open3d_polygons(points, polygons):
    config_pp = dict(filter=dict(hole_area=dict(min=0.025, max=0.785), hole_vertices=dict(min=6), plane_area=dict(min=0.5)),
                     positive_buffer=0.01, negative_buffer=0.03, simplify=0.02)
    planes, obstacles = filter_planes_and_holes(polygons, points, config_pp)
    all_poly_lines = create_lines(planes, obstacles, line_radius=0.01)
    return all_poly_lines


def run_test(pcd, rgbd, intrinsics, extrinsics, bp_alg=dict(radii=[0.02, 0.02]), poisson=dict(depth=8), callback=None):
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
    t1 = time.perf_counter()
    mesh_uniform_grid, polylidar_inputs = make_uniform_grid_mesh2(
        rgbd, intrinsics, extrinsics)
    t2 = time.perf_counter()
    prep_mesh(mesh_uniform_grid)
    time_mesh_uniform = (t2 - t1) * 1000
    uniform_alg_name = 'Uniform Grid Mesh'
    callback(uniform_alg_name, time_mesh_uniform, pcd, mesh_uniform_grid)
    # Polylidar3D with Uniform Mesh Grid
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


def get_point(i, j, im, intrinsics):
    z = im[i, j]
    pp = intrinsics.get_principal_point()
    fl = intrinsics.get_focal_length()
    x = (j - pp[0]) * z / fl[0]
    y = (i - pp[1]) * z / fl[1]
    return [x, y, z]
    # point = (extrinsics @ np.array([[x,y,z,1]]).T)[:3,0].tolist()
    # return point, z


def create_point_cloud_from_rgbd_image(im, rows, cols, intrinsics, stride=2):
    points = []
    cols_stride = math.ceil(cols / stride)
    rows_stride = math.ceil(rows / stride)
    points = np.zeros((cols_stride * rows_stride, 3))
    # Create Point Cloud
    # Invalid points (no depth) will still be created and map to [0,0,0]
    # These point will NOT exist in the triangulation, but exist in the point array
    pnt_cnt = 0
    for i in range(0, rows, stride):
        for j in range(0, cols, stride):
            p1 = get_point(i, j, im, intrinsics)
            points[pnt_cnt, :] = p1
            pnt_cnt += 1

    return points


def create_uniform_mesh_from_image(rows, cols, points, stride=2):
    triangles = []
    # This represents the number of rows and columns of the downsampled POINT CLOUD
    cols_stride = math.ceil(cols / stride)
    rows_stride = math.ceil(rows / stride)
    # This represent the number of rows and columns of the UNIFORM TRIANGULAR MESH
    cols_tris = cols_stride - 1
    rows_tris = rows_stride - 1
    # These are the maximum number of triangles that can ever be in the mesh
    max_triangles = cols_tris * rows_tris * 2
    # This will count valid points and triangles
    tri_cnt = 0
    pix_cnt = 0
    # Invalid triangle marker
    max_value = np.iinfo(np.uint64).max
    # Create an array representing the full mesh. The *position* of each element is
    # the triangles global index. The *value* is its TRUE index in the triangles mesh array
    # These are the same if no invalid triangles occur
    valid_tri = np.full(max_triangles, max_value, dtype=np.uint64)

    # Create the triangles array for the mesh
    # Loop through every 4 point square collection of points and create two triangles
    # Each triangle holds 3 point INDICES representing its vertices
    # A triangle can only be created if a valid point exists (depth > 0)
    for i in range(0, rows_tris):
        for j in range(0, cols_tris):
            p1_idx = i * cols_stride + j
            p2_idx = i * cols_stride + j + 1
            p3_idx = (i + 1) * cols_stride + j + 1
            p4_idx = (i + 1) * cols_stride + j

            p1 = points[p1_idx, 2]
            p2 = points[p2_idx, 2]
            p3 = points[p3_idx, 2]
            p4 = points[p4_idx, 2]

            if p1 > 0 and p2 > 0 and p3 > 0:
                # Create the first triangle in the square (borders on right)
                triangles.append([p1_idx, p2_idx, p3_idx])
                valid_tri[pix_cnt * 2] = tri_cnt
                tri_cnt += 1
            if p3 > 0 and p4 > 0 and p1 > 0:
                # Create the second triangle in the square (borders on left)
                triangles.append([p3_idx, p4_idx, p1_idx])
                valid_tri[pix_cnt * 2 + 1] = tri_cnt
                tri_cnt += 1
            pix_cnt += 1

    triangles = np.array(triangles, dtype=np.uint64)  # convert to numpy
    return triangles, valid_tri


def extract_halfedges_from_uniform_mesh(rows, cols, triangles, valid_tri, stride=2):
    cols_stride = math.ceil(cols / stride)
    rows_stride = math.ceil(rows / stride)
    # This represent the number of rows and columns of the UNIFORM TRIANGULAR MESH
    cols_tris = cols_stride - 1
    rows_tris = rows_stride - 1

    # Invalid triangle marker
    max_value = np.iinfo(np.uint64).max
    halfedges = np.full(triangles.shape[0] * 3, max_value, dtype=np.uint64)

    for i in range(rows_tris):
        for j in range(cols_tris):
            # These are the triangle indexes in the global full mesh
            t_global_idx_first = (cols_tris * i + j) * 2
            t_global_idx_second = (cols_tris * i + j) * 2 + 1
            # We convert these global meshes to our valid mesh indices
            t_valid_idx_first = valid_tri[t_global_idx_first]
            t_valid_idx_second = valid_tri[t_global_idx_second]
            # Check if first triangle is valid, if so extract half edges
            if (t_valid_idx_first != max_value):
                # We have a valid first triangle
                # Check if we are on the top of the RGBD Image, if so then we have a border top edge
                if i == 0:
                    t_valid_idx_top = max_value  # indicates this edge has no border
                else:
                    # Gets the triangle one row up from this one, math is from implicit structure
                    t_global_idx_top = t_global_idx_first - 2 * cols_tris + 1
                    # Convert to valid mesh index
                    t_valid_idx_top = valid_tri[t_global_idx_top]
                # Check if we are on the right side of the RGBD Image, if so than we have a border on the right
                if j >= cols_tris - 1:
                    t_valid_idx_right = max_value  # indicates this edge has no border
                else:
                    # Gets the triangle one cell to the right, math is from implicit structure
                    t_global_idx_right = t_global_idx_first + 3
                    t_valid_idx_right = valid_tri[t_global_idx_right]
                # SET edges if valid
                if t_valid_idx_top != max_value:
                    halfedges[int(t_valid_idx_first * 3)] = t_valid_idx_top * 3
                if t_valid_idx_right != max_value:
                    halfedges[int(t_valid_idx_first * 3 + 1)
                              ] = t_valid_idx_right * 3 + 1
                if t_valid_idx_second != max_value:
                    halfedges[int(t_valid_idx_first * 3 + 2)
                              ] = t_valid_idx_second * 3 + 2

            # Check if second triangle is valid, if so extract half edges
            if (t_valid_idx_second != max_value):
                # We have a valid second triangle
                # Check if we are on the bottom of the RGBD Image, if so then we have a border bottom edge
                if i == rows_tris - 1:
                    t_valid_idx_bottom = max_value
                else:
                    t_global_idx_bottom = t_global_idx_second + 2 * cols_tris - 1
                    t_valid_idx_bottom = valid_tri[t_global_idx_bottom]
                # Check if we are on the left side of the RGBD Image, if so than we have a border on the left
                if j == 0:
                    t_valid_idx_left = max_value
                else:
                    t_global_idx_left = t_global_idx_second - 3
                    t_valid_idx_left = valid_tri[t_global_idx_left]
                # Set Edges
                if t_valid_idx_bottom != max_value:
                    halfedges[int(t_valid_idx_second * 3)
                              ] = t_valid_idx_bottom * 3
                if t_valid_idx_left != max_value:
                    halfedges[int(t_valid_idx_second * 3 + 1)
                              ] = t_valid_idx_left * 3 + 1
                if t_valid_idx_first != max_value:
                    halfedges[int(t_valid_idx_second * 3 + 2)
                              ] = t_valid_idx_first * 3 + 2
    return halfedges

def make_uniform_grid_mesh2(rgbd_image, intrinsics, extrinsics, stride=2):
    """Create a Unifrom Grid Mesh from an RGBD Image

    Arguments:
        rgbd_image {rgdb} -- Open3D RGBD Image
        intrinsics {intrinsics} -- Open3D Intrinsics
        extrinsics {ndarray} -- 4X4 Numpy array

    Keyword Arguments:
        stride {int} -- Stride for creating point cloud (default: {2})

    Returns:
        tupl
    """
    im = np.asarray(rgbd_image.depth)
    rows = im.shape[0]
    cols = im.shape[1]
    intrinsics_ = np.ascontiguousarray(intrinsics.intrinsic_matrix)
    t0 = time.perf_counter()
    points, triangles, halfedges = extract_uniform_mesh_from_float_depth(im, intrinsics_, stride=stride)
    t1 = time.perf_counter()
    print(t1 - t0)
    points = np.asarray(points)
    triangles = np.asarray(triangles)
    halfedges = np.asarray(halfedges)
    points = points.reshape((int(points.shape[0] / 3), 3))
    # TODO Need to create half edges STILL
    # Rotate Point Cloud
    points = np.column_stack((points, np.ones(points.shape[0])))
    points = np.ascontiguousarray(((extrinsics @ points.T).T)[:, :3])
    t2 = time.perf_counter()
    print(t2 - t1)
    # Create open3D mesh
    mesh = create_open_3d_mesh(triangles.reshape((int(triangles.shape[0] / 3), 3)), points)
    # halfedges = np.asarray(o3d.geometry.HalfEdgeTriangleMesh.extract_halfedges(mesh))
    # Also create the polylidar desired representation of the mesh
    polylidar_inputs = dict(
        vertices=points, triangles=triangles, halfedges=halfedges)
    return mesh, polylidar_inputs

def make_uniform_grid_mesh(rgbd_image, intrinsics, extrinsics, stride=2):
    """Create a Unifrom Grid Mesh from an RGBD Image

    Arguments:
        rgbd_image {rgdb} -- Open3D RGBD Image
        intrinsics {intrinsics} -- Open3D Intrinsics
        extrinsics {ndarray} -- 4X4 Numpy array

    Keyword Arguments:
        stride {int} -- Stride for creating point cloud (default: {2})

    Returns:
        tuple(mesh, dict) -- Open3D mesh and Polylidar Inputs
    """
    im = np.asarray(rgbd_image.depth)
    rows = im.shape[0]
    cols = im.shape[1]
    points = create_point_cloud_from_rgbd_image(im, rows, cols, intrinsics, stride=stride)
    triangles, valid_tri = create_uniform_mesh_from_image(rows, cols, points, stride=stride)
    halfedges = extract_halfedges_from_uniform_mesh(rows, cols, triangles, valid_tri, stride=stride)

    # Rotate Point Cloud
    points = np.column_stack((points, np.ones(points.shape[0])))
    points = np.ascontiguousarray(((extrinsics @ points.T).T)[:, :3])
    # Create open3D mesh
    mesh = create_open_3d_mesh(triangles, points)
    # halfedges = np.asarray(o3d.geometry.HalfEdgeTriangleMesh.extract_halfedges(mesh))
    # Also create the polylidar desired representation of the mesh
    polylidar_inputs = dict(
        vertices=points, triangles=triangles.flatten(), halfedges=halfedges)
    return mesh, polylidar_inputs


def callback(alg_name, execution_time, pcd, mesh=None):
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -0.7])
    grid_ls = construct_grid(
        size=2, n=20, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    logging.info("%r took %.2f milliseconds", alg_name, execution_time)
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
    grid_ls = construct_grid(
        size=2, n=200, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    for idx in range(len(color_files)):
        if idx < 3:
            continue
        pcd, rgbd, extrinsics = get_frame_data(
            idx, color_files, depth_files, traj, intrinsics)
        pcd = pcd.rotate(R_Standard_d400[:3, :3], center=False)
        # pcd = pcd.voxel_down_sample(voxel_size=0.02)

        logging.info("File %r - Point Cloud; Size: %r",
                     idx, np.asarray(pcd.points).shape[0])
        o3d.visualization.draw_geometries([pcd, grid_ls, axis_frame])
        results = run_test(pcd, rgbd, intrinsics,
                           extrinsics, callback=callback)


if __name__ == "__main__":
    main()

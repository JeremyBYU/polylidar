"""
Demonstrates Polygon Extraction on some RealSense data.
Warning - Haven't looked at this specific file in a while.
TODO - Clean this up
"""
import time
import logging
import sys

import numpy as np

from examples.python.util.realsense_util import (get_realsense_data, get_frame_data, R_Standard_d400, prep_mesh,
                                                 create_open3d_pc, extract_mesh_planes, COLOR_PALETTE)

from polylidar import (Polylidar3D, MatrixDouble, MatrixFloat, extract_tri_mesh_from_float_depth,
                       extract_point_cloud_from_float_depth)

from polylidar.polylidarutil.open3d_util import construct_grid, create_lines, flatten, create_open_3d_mesh_from_tri_mesh
from polylidar.polylidarutil.plane_filtering import filter_planes_and_holes

import open3d as o3d


def filter_and_create_open3d_polygons(points, polygons):
    " Apply polygon filtering algorithm, return Open3D Mesh Lines "
    config_pp = dict(filter=dict(hole_area=dict(min=0.025, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.5)),
                     positive_buffer=0.00, negative_buffer=0.02, simplify=0.01)
    planes, obstacles = filter_planes_and_holes(polygons, points, config_pp)
    all_poly_lines = create_lines(planes, obstacles, line_radius=0.01)
    return all_poly_lines


def run_test(pcd, rgbd, intrinsics, extrinsics, bp_alg=dict(radii=[0.02, 0.02]), poisson=dict(depth=8), callback=None, stride=2):
    points = np.asarray(pcd.points)
    polylidar_kwargs = dict(alpha=0.0, lmax=0.10, min_triangles=100,
                            z_thresh=0.04, norm_thresh=0.90, norm_thresh_min=0.90, min_hole_vertices=6)
    pl = Polylidar3D(**polylidar_kwargs)

    ##### Treat data an an unorganized point clouds                                ########
    ##### Create Surface Mesh using 2.5 Delaunay Triangulation and extract Polygons########
    points_mat = MatrixDouble(points)
    t1 = time.perf_counter()
    mesh, planes, polygons = pl.extract_planes_and_polygons(points_mat)
    t2 = time.perf_counter()

    # Visualization Code
    all_poly_lines = filter_and_create_open3d_polygons(points, polygons)
    triangles = np.asarray(mesh.triangles)
    mesh_2d_polylidar = extract_mesh_planes(points, triangles, planes, mesh.counter_clock_wise, COLOR_PALETTE[0])
    mesh_2d_polylidar.extend(flatten([line_mesh.cylinder_segments for line_mesh in all_poly_lines]))
    time_mesh_2d_polylidar = (t2 - t1) * 1000
    polylidar_alg_name = 'Polylidar2D'
    callback(polylidar_alg_name, time_mesh_2d_polylidar, pcd, mesh_2d_polylidar)

    ###### Treat data as an **Organized** 3D Point Cloud #########
    ###### Creates a true 3D mesh and is much master using organized structure of point cloud 
    tri_mesh, t_mesh_creation = make_uniform_grid_mesh(np.asarray(
        rgbd.depth), np.ascontiguousarray(intrinsics.intrinsic_matrix), extrinsics, stride=stride)

    # Visualization of only the mesh
    mesh_uniform_grid = create_open_3d_mesh_from_tri_mesh(tri_mesh)
    uniform_alg_name = 'Fast Uniform Mesh'
    callback(uniform_alg_name, t_mesh_creation, pcd, mesh_uniform_grid)

    # Exctact Polygons with Polylidar3D using the Uniform Mesh. Dominant Plane normal is 0,0,1.
    t1 = time.perf_counter()
    planes, polygons = pl.extract_planes_and_polygons(tri_mesh)
    t2 = time.perf_counter()

    # Visualization Code of Polygons
    vertices_np = np.assarray(tri_mesh.vertices)
    all_poly_lines = filter_and_create_open3d_polygons(vertices_np, polygons)
    mesh_3d_polylidar = extract_mesh_planes(vertices_np, triangles, planes, tri_mesh.counter_clock_wise)
    mesh_3d_polylidar.extend(flatten([line_mesh.cylinder_segments for line_mesh in all_poly_lines]))
    time_polylidar3D = (t2 - t1) * 1000
    polylidar_3d_alg_name = 'Polylidar with Uniform Grid Mesh'
    callback(polylidar_3d_alg_name, time_polylidar3D,
             create_open3d_pc(vertices_np), mesh_3d_polylidar)

    ##### Uncomment if you are interested in other mesh creation techniques #####

    # # Estimate Point Cloud Normals
    # t3 = time.perf_counter()
    # pcd.estimate_normals(
    #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.10, max_nn=20))
    # t4 = time.perf_counter()
    # time_estimate_point_normals = (t4 - t3) * 1000
    # point_normal_alg_name = 'Point Normal Estimation'
    # callback(point_normal_alg_name, time_estimate_point_normals, pcd, None)
    # # Create True 3D Surface Mesh using Ball Pivot Algorithm
    # radii = o3d.utility.DoubleVector(bp_alg['radii'])
    # t5 = time.perf_counter()
    # mesh_ball_pivot = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #     pcd, radii)
    # prep_mesh(mesh_ball_pivot)
    # t6 = time.perf_counter()
    # time_mesh_ball_pivot = (t6 - t5) * 1000
    # ball_point_alg_name = 'Ball Pivot'
    # callback(ball_point_alg_name, time_mesh_ball_pivot, pcd, mesh_ball_pivot)
    # # Create True 3D Surface Mesh using Poisson Reconstruction Algorithm
    # t7 = time.perf_counter()
    # mesh_poisson, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    #     pcd, **poisson)
    # vertices_to_remove = densities < np.quantile(densities, 0.1)
    # mesh_poisson.remove_vertices_by_mask(vertices_to_remove)
    # t8 = time.perf_counter()
    # prep_mesh(mesh_poisson)
    # time_mesh_poisson = (t8 - t7) * 1000
    # poisson_alg_name = 'Poisson'
    # callback(poisson_alg_name, time_mesh_poisson, pcd, mesh_poisson)

    # results = [
    #     dict(alg=polylidar_alg_name, mesh=mesh_2d_polylidar,
    #          execution_time=time_mesh_2d_polylidar),
    #     dict(alg=point_normal_alg_name, mesh=None,
    #          execution_time=time_estimate_point_normals),
    #     dict(alg=ball_point_alg_name, mesh=mesh_ball_pivot,
    #          execution_time=time_mesh_ball_pivot),
    #     dict(alg=poisson_alg_name, mesh=mesh_poisson,
    #          execution_time=time_mesh_poisson)
    # ]
    # return results


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
    tri_mesh = extract_tri_mesh_from_float_depth(MatrixFloat(
        im), MatrixDouble(intrinsics), MatrixDouble(extrinsics), stride=stride)
    t1 = time.perf_counter()

    return tri_mesh, (t1-t0) * 1000


def callback(alg_name, execution_time, pcd, mesh=None):
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -0.7])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-1.0, translate=[0, 1.0, 0.0])
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
        if idx < 4:
            continue
        pcd, rgbd, extrinsics = get_frame_data(idx, color_files, depth_files, traj, intrinsics, stride=2)
        pcd = pcd.rotate(R_Standard_d400[:3, :3], center=False)

        logging.info("File %r - Point Cloud; Size: %r", idx, np.asarray(pcd.points).shape[0])
        o3d.visualization.draw_geometries([pcd, grid_ls, axis_frame])
        results = run_test(pcd, rgbd, intrinsics, extrinsics, callback=callback, stride=2)


if __name__ == "__main__":
    main()

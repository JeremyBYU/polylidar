import time
import logging
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt
import numpy as np

from examples.python.realsense_util import (get_realsense_data, get_frame_data, R_Standard_d400, prep_mesh,
                                            create_open3d_pc, extract_mesh_planes, COLOR_PALETTE, create_open_3d_mesh)

from polylidar import (extractPlanesAndPolygons, extract_planes_and_polygons_from_mesh, extract_tri_mesh_from_float_depth,
                      extract_point_cloud_from_float_depth, extract_uniform_mesh_from_float_depth)

from polylidarutil.open3d_util import construct_grid, create_lines, flatten
from polylidarutil.plane_filtering import filter_planes_and_holes

import open3d as o3d


def show_geoms(pcd, mesh=None):
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -0.7])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    if mesh:
        if isinstance(mesh, list):
            o3d.visualization.draw_geometries(
                [*mesh, pcd, grid_ls, axis_frame])
        else:
            o3d.visualization.draw_geometries([mesh, pcd, grid_ls, axis_frame])

def run_test(pcd, rgbd, intrinsics, extrinsics, stride=2):
    im = np.asarray(rgbd.depth)
    intrinsics = np.ascontiguousarray(intrinsics.intrinsic_matrix)
    # print(extrinsics)
    # extr_temp = np.identity(4)
    tri_mesh = extract_tri_mesh_from_float_depth(im, intrinsics, extrinsics, stride=stride)
    triangles = np.asarray(tri_mesh.triangles)
    vertices = np.asarray(tri_mesh.vertices)
    triangle_normals = np.asarray(tri_mesh.triangle_normals)

    vertices = vertices.reshape((int(vertices.shape[0] / 3), 3))
    triangle_normals = triangle_normals.reshape((int(triangle_normals.shape[0] / 3), 3))

    mesh = create_open_3d_mesh(triangles, vertices)
    show_geoms(pcd, mesh=mesh)
    return triangle_normals, mesh


def get_quiver(triangle_normals, ds=10):
    # Make the direction data for the arrows
    triangle_normals = triangle_normals[::ds, :]
    x = triangle_normals[:, 0]
    y = triangle_normals[:, 1]
    z = triangle_normals[:, 2]

    u = triangle_normals[:, 0]
    v = triangle_normals[:, 1]
    w = triangle_normals[:, 2]
    return [x,y,z,u,v,w]

def plot_normals(triangle_normals_1, triangle_normals_2):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    quiver1 = get_quiver(triangle_normals_1)
    quiver2 = get_quiver(triangle_normals_2)

    ax.set_xlim3d(-1.5, 1.5)
    ax.set_ylim3d(-1.5,1.5)
    ax.set_zlim3d(-1.5,1.5)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.quiver(*quiver1, length=0.1, normalize=True, colors='r')
    ax.quiver(*quiver2, length=0.1, normalize=False, colors='b')

    plt.show()

def main():
    color_files, depth_files, traj, intrinsics = get_realsense_data()
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    axis_frame.translate([0, 0.8, -0.7])
    grid_ls = construct_grid(size=2, n=20, plane_offset=-0.8, translate=[0, 1.0, 0.0])
    for idx in range(len(color_files)):
        if idx < 2:
            continue
        pcd, rgbd, extrinsics = get_frame_data(idx, color_files, depth_files, traj, intrinsics, stride=3)
        pcd = pcd.rotate(R_Standard_d400[:3, :3], center=False)

        logging.info("File %r - Point Cloud; Size: %r", idx, np.asarray(pcd.points).shape[0])
        # o3d.visualization.draw_geometries([pcd, grid_ls, axis_frame])
        triangle_normals, mesh = run_test(pcd, rgbd, intrinsics, extrinsics, stride=3)
        plot_normals(triangle_normals,np.asarray(mesh.triangle_normals))

if __name__ == "__main__":
    main()
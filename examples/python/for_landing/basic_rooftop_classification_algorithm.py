"""Example Multipolygon Extraction
"""
import time
import math

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


from polylidar import MatrixDouble, Polylidar3D
from polylidar.polylidarutil import (plot_polygons_3d, generate_3d_plane, set_axes_equal, plot_planes_3d,
                                     scale_points, rotation_matrix, apply_rotation)


def extract_polygons(opc, classes, config):
    pl = Polylidar3D(**config['polylidar'])
    ga = GaussianAccumulatorS2Beta(level=config['fastga']['level'])
    ico = IcoCharts(level=config['fastga']['level'])
    # 1. Create mesh
    tri_mesh = create_meshes_cuda(
        opc, **config['mesh']['filter'])

    mesh_o3d = create_open_3d_mesh_from_tri_mesh(tri_mesh)
    classes = classes.reshape((np.asarray(mesh_o3d.vertices).shape[0], 1))
    classes_mat = MatrixUInt8(classes)
    tri_mesh.set_vertex_classes(classes_mat, True)
    # alg_timings.update(timings)
    # 2. Get dominant plane normals
    avg_peaks, _, _, _, alg_timings = extract_all_dominant_plane_normals(
        tri_mesh, ga_=ga, ico_chart_=ico, **config['fastga'])

    # alg_timings.update(timings)
    # 3. Extract Planes and Polygons
    planes, obstacles, timings = extract_planes_and_polygons_from_classified_mesh(tri_mesh, avg_peaks, pl_=pl,
                                                                        filter_polygons=True, optimized=True,
                                                                        postprocess=config['polygon']['postprocess'])

    return mesh_o3d, planes, obstacles, timings

def main():
    np.random.seed(1)
    # generate random plane with hole
    plane = generate_3d_plane(bounds_x=[0, 10, 0.5], bounds_y=[0, 10, 0.5], holes=[
        [[3, 5], [3, 5]]], height_noise=0.02, planar_noise=0.02)
    # Generate top of box (causing the hole that we see)
    box_top = generate_3d_plane(bounds_x=[3, 5, 0.2], bounds_y=[3, 5, 0.2], holes=[
    ], height_noise=0.02, height=2, planar_noise=0.02)
    # Generate side of box (causing the hole that we see)
    box_side = generate_3d_plane(bounds_x=[0, 2, 0.2], bounds_y=[
        0, 2, 0.2], holes=[], height_noise=0.02, planar_noise=0.02)
    rm = rotation_matrix([0, 1, 0], -math.pi / 2.0)
    box_side = apply_rotation(rm, box_side) + [5, 3, 0]
    # All points joined together
    points = np.concatenate((plane, box_side, box_top))

    points_mat = MatrixDouble(points)
    polylidar_kwargs = dict(alpha=0.0, lmax=1.0, min_triangles=20, z_thresh=0.1, norm_thresh_min=0.94)
    polylidar = Polylidar3D(**polylidar_kwargs)

    elev = 15.0
    azim = -35

    # Show Point Cloud
    print("Should see point raw point cloud")
    fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1,
                           subplot_kw=dict(projection='3d'))
    # plot points
    ax.scatter(*scale_points(points), s=20.0, c=points[:, 2], cmap=plt.cm.plasma)
    set_axes_equal(ax)
    ax.view_init(elev=elev, azim=azim)
    fig.savefig("assets/scratch/Basic25DAlgorithm_pointcloud.pdf", bbox_inches='tight')
    fig.savefig("assets/scratch/Basic25DAlgorithm_pointcloud.png", bbox_inches='tight', pad_inches=-0.8)
    plt.show()

    # Extracts planes and polygons, time
    t1 = time.time()
    mesh, planes, polygons = polylidar.extract_planes_and_polygons(points_mat)
    t2 = time.time()
    print("Polylidar Took {:.2f} milliseconds".format((t2 - t1) * 1000))

    triangles = np.asarray(mesh.triangles)
    all_planes = [np.arange(triangles.shape[0])]

    # Show Triangulation
    fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1,
                           subplot_kw=dict(projection='3d'))

    plot_planes_3d(points, triangles, all_planes, ax, alpha=0.0, z_value=-4.0)
    plot_planes_3d(points, triangles, all_planes, ax, alpha=0.5)
    # plot points
    ax.scatter(*scale_points(points), s=0.1, c='k')
    set_axes_equal(ax, ignore_z=True)
    ax.set_zlim3d([-4, 6])
    ax.view_init(elev=elev, azim=azim)
    print("Should see triangulation point cloud")
    fig.savefig("assets/scratch/Basic25DAlgorithm_mesh.pdf", bbox_inches='tight')
    fig.savefig("assets/scratch/Basic25DAlgorithm_mesh.png", bbox_inches='tight', pad_inches=-0.8)
    plt.show()

    print("")
    print("Should see two planes extracted, please rotate.")
    fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1,
                           subplot_kw=dict(projection='3d'))
    # plot all triangles
    plot_polygons_3d(points, polygons, ax)
    plot_planes_3d(points, triangles, planes, ax)
    # plot points
    ax.scatter(*scale_points(points), c='k', s=0.1)
    set_axes_equal(ax)
    ax.view_init(elev=elev, azim=azim)
    fig.savefig("assets/scratch/Basic25DAlgorithm_polygons.pdf", bbox_inches='tight')
    fig.savefig("assets/scratch/Basic25DAlgorithm_polygons.png", bbox_inches='tight', pad_inches=-0.8)
    plt.show()
    print("")


if __name__ == "__main__":
    main()

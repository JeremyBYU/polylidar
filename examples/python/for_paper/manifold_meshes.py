import time
import math

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib 

matplotlib.rc('xtick', labelsize=14) 
matplotlib.rc('ytick', labelsize=14)
# matplotlib.rc('ztick', labelsize=20)
matplotlib.rc('axes', labelsize=18) 


from polylidar import Delaunator, MatrixDouble, Polylidar3D
from polylidar.polylidarutil import (plot_polygons_3d, generate_3d_plane, set_axes_equal, plot_planes_3d,
                           scale_points, rotation_matrix, apply_rotation)



def set_labels(ax):
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

def triangulate_and_combine(points, points_bad, triangles_bad): 
    points_mat = MatrixDouble(points)
    mesh = Delaunator(points_mat)
    mesh.triangulate()

    triangles = np.asarray(mesh.triangles)
    points_new = np.concatenate([points, points_bad], axis=0)
    triangles_new = np.concatenate([triangles, triangles_bad], axis=0)
    all_planes = [np.arange(triangles_new.shape[0])]

    return mesh, points_new, triangles_new, all_planes

def main():
    np.random.seed(1)
    # generate random plane with hole

    z_lim = [-4, 6]
    elev = 10.0
    azim = -35

    points = generate_3d_plane(bounds_x=[2.0, 6.0, 1.0], bounds_y=[2.0, 6.0, 0.5], holes=[], planar_noise=0.0, height_noise=0.01)

    points_bad = np.array([
        [3.0, 3.5, 0.0],
        [3.5, 3.5, 1.0],
        [4.0, 3.5, 0.0]
    ])
    end_point_idx = points.shape[0]
    triangles_bad = np.array([
        [end_point_idx, end_point_idx + 1, end_point_idx + 2]
    ])

    mesh, points_new, triangles_new, all_planes = triangulate_and_combine(points, points_bad, triangles_bad)

    # Show Triangulation
    fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1,
                    subplot_kw=dict(projection='3d'))

    plot_planes_3d(points_new, triangles_new, all_planes, ax, alpha=0.5)
    set_axes_equal(ax, ignore_z=True)
    ax.set_zlim3d(z_lim)
    ax.view_init(elev=elev, azim=azim)
    plt.show()


    points = generate_3d_plane(bounds_x=[2.0, 6.0, 1.0], bounds_y=[2.0, 6.0, 0.5], holes=[], planar_noise=0.0, height_noise=0.01)
    points[18, 2] = points[18, 2] + 0.30
    points_bad = np.array([
        [3.0, 3.5, 0.0],
        [3.5, 4.0, 0.20],
        [4.0, 3.5, 0.0]
    ])
    end_point_idx = points.shape[0]
    triangles_bad = np.array([
        [end_point_idx, end_point_idx + 1, end_point_idx + 2]
    ])

    mesh, points_new, triangles_new, all_planes = triangulate_and_combine(points, points_bad, triangles_bad)
    # all_planes.append([13, 14])
    # Show Triangulation
    fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1,
                    subplot_kw=dict(projection='3d'))

    plot_planes_3d(points_new, triangles_new, all_planes, ax, alpha=0.5)
    # plot_planes_3d(points_new, triangles_new, [[triangles_new.shape[0]-1]], ax, alpha=1.0, color=(1,0,0))
    
    ax.plot_trisurf(*scale_points(points_new, z_value=None, z_scale=1.0),triangles=triangles_new[[triangles_new.shape[0]-1]], color=(1,0,0,1.0),  edgecolor=(0,0,0,0.3), linewidth=0.5, antialiased=True, zorder=1.5)
    ax.plot_trisurf(*scale_points(points_new, z_value=None, z_scale=1.0),triangles=triangles_new[[13,14]], color=(0,1,0,0.5),  edgecolor=(0,0,0,0.3), linewidth=0.5, antialiased=False, zorder=0.5)
    # plot_planes_3d(points_new, triangles_new, [[13,14]], ax, alpha=0.9, color=(0,1,0))
    set_axes_equal(ax, ignore_z=True)
    ax.set_zlim3d(z_lim)
    ax.view_init(elev=elev, azim=azim)
    plt.show()




   

if __name__ == "__main__":
    main()

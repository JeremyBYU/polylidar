import time
import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, MultiPolygon
from descartes import PolygonPatch


from polylidar import Delaunator, MatrixDouble, Polylidar3D
from polylidar.polylidarutil import (plot_polygons_3d, generate_3d_plane, set_axes_equal,
                           scale_points, rotation_matrix, apply_rotation, get_point)
from polylidar.polylidarutil.plane_filtering import get_points

COLOR_PALETTE = list(map(colors.to_rgb, plt.rcParams['axes.prop_cycle'].by_key()['color']))


matplotlib.rc('xtick', labelsize=18) 
matplotlib.rc('ytick', labelsize=18)
# matplotlib.rc('ztick', labelsize=20)
matplotlib.rc('axes', labelsize=18) 


def plot_polygons(polygons, points, ax, shell_color='green', hole_color='orange'):
    for poly in polygons:
        shell_coords = [get_point(pi, points) for pi in poly.shell]
        outline = Polygon(shell=shell_coords)
        outlinePatch = PolygonPatch(outline, ec=shell_color, fill=False, linewidth=3, zorder=4)
        ax.add_patch(outlinePatch)

        for hole_poly in poly.holes:
            shell_coords = [get_point(pi, points) for pi in hole_poly]
            outline = Polygon(shell=shell_coords)
            outlinePatch = PolygonPatch(outline, ec=hole_color, fill=False, linewidth=3, zorder=4)
            ax.add_patch(outlinePatch)

def plot_planes_3d(lidar_building, triangles, planes, ax, alpha=[1.0], z_value=None, z_scale=1.0, color=None):
    ax_planes = []
    for i, plane in enumerate(planes):
        triangles_plane = triangles[plane]
        color_ = COLOR_PALETTE[i] if color is None else color
        color_ = color_ + (alpha[i], )
        ax_plane = ax.plot_trisurf(*scale_points(lidar_building, z_value=z_value, z_scale=z_scale),triangles=triangles_plane, color=color_,  edgecolor=(0,0,0,0.3), linewidth=0.5)
        ax_planes.append(ax_plane)
    return ax_planes

def main():
    # np.random.seed(5)
    np.random.seed(9)
    # generate random plane with hole
    plane = generate_3d_plane(bounds_x=[0, 10, 0.5], bounds_y=[0, 10, 0.5], holes=[
                            [[3, 5], [3, 5]]], height_noise=0.05, planar_noise=0.10)
    # Generate top of box (causing the hole that we see)
    box_top = generate_3d_plane(bounds_x=[3, 5, 0.2], bounds_y=[3, 5, 0.2], holes=[
    ], height_noise=0.02, height=2, planar_noise=0.02)
    # Generate side of box (causing the hole that we see)
    box_side = generate_3d_plane(bounds_x=[0, 2, 0.2], bounds_y=[
                                0, 2, 0.2], holes=[], height_noise=0.02, planar_noise=0.02)
    rm = rotation_matrix([0,1,0], -math.pi/2.0)
    box_side = apply_rotation(rm, box_side) + [5, 3, 0]
    # box_side = r.apply(box_side) + [5, 3, 0]
    # All points joined together
    points = np.concatenate((plane, box_side, box_top))

    points_mat = MatrixDouble(points)
    polylidar_kwargs = dict(alpha=0.0, lmax=1.0, min_triangles=20, z_thresh=0.1, norm_thresh_min=0.94)
    polylidar = Polylidar3D(**polylidar_kwargs)

    elev = 15.0
    azim = -125


    # Extracts planes and polygons, time
    t1 = time.time()
    mesh, planes, polygons = polylidar.extract_planes_and_polygons(points_mat)
    t2 = time.time()
    print("Polylidar Took {:.2f} milliseconds".format((t2 - t1) * 1000))

    triangles = np.asarray(mesh.triangles)
    all_planes = [np.arange(triangles.shape[0])]
    
    print("")
    print("Should see a mesh segments")
    fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1,
                        subplot_kw=dict(projection='3d'))
    # plot all triangles
    # plot_polygons_3d(points, polygons, ax)
    rm_45 = rotation_matrix([0,1,0], -math.pi/4.0)
    points_rot = apply_rotation(rm_45, points)
    plot_planes_3d(points_rot, triangles, [planes[1]], ax, alpha=[0.85])
    # plot points
    # ax.scatter(*scale_points(points), c='k', s=0.1)
    set_axes_equal(ax)
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    fig.savefig("assets/scratch/PolygonExtraction_a.pdf", bbox_inches='tight')
    # fig.savefig("assets/scratch/Basic25DAlgorithm_polygons.png", bbox_inches='tight', pad_inches=-0.8)
    plt.show()

    # Show 2D Projection and Polygon Extraction
    print("")
    print("Should see projected vertices to geometric plane")
    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5,5))
    simplices_plane = triangles[planes[1], :]
    plot_polygons([polygons[1]], points[:, :2], ax, shell_color=COLOR_PALETTE[4], hole_color=COLOR_PALETTE[4] )
    ax.triplot(points[:,0], points[:,1], simplices_plane, c='k', alpha=0.75)
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    fig.savefig("assets/scratch/PolygonExtraction_b1.pdf", bbox_inches='tight')
    plt.show()

    # Show 2D Projection and Polygon Extraction
    print("")
    print("Should see projected vertices to geometric plane")
    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5,5))
    simplices_plane = triangles[planes[1], :]
    plot_polygons([polygons[1]], points[:, :2], ax )
    ax.triplot(points[:,0], points[:,1], simplices_plane, c='k', alpha=0.75)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    fig.savefig("assets/scratch/PolygonExtraction_b2.pdf", bbox_inches='tight')
    plt.show()

    poly = polygons[1]
    shell_coords = get_points(poly.shell, points)
    hole_coords = [get_points(hole, points) for hole in poly.holes]
    poly_shape = Polygon(shell=shell_coords, holes=hole_coords)

    # Show Simplification 
    print("")
    print("Should see simplified Polygon")
    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5,5))

    patch = PolygonPatch(poly_shape, fill=True, alpha=0.5)
    ax.add_patch(patch)
    # plot_polygons([polygons[1]], points[:, :2], ax)
    poly_simplify = poly_shape.simplify(.2, preserve_topology=True)
    patch = PolygonPatch(poly_simplify, color='k', fill=False, linewidth=1.5, linestyle='--', zorder=1)
    ax.add_patch(patch)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    print(f"Before Vertices: {len(poly_shape.exterior.coords)}")
    print(f"After Vertices: {len(poly_simplify.exterior.coords)}")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    fig.savefig("assets/scratch/PolygonExtraction_c.pdf", bbox_inches='tight')
    plt.show()

    # Show Positive Buffer
    print("")
    print("Should see positive buffer Polygon")
    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5,5))

    patch = PolygonPatch(poly_shape, fill=True, alpha=0.5)
    ax.add_patch(patch)
    # plot_polygons([polygons[1]], points[:, :2], ax)
    poly_buffer = poly_simplify.buffer(.2)
    patch = PolygonPatch(poly_buffer, color='k', fill=False, linewidth=1.5, linestyle='--', zorder=1)
    ax.add_patch(patch)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    print(f"After Vertices: {len(poly_buffer.exterior.coords)}")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    fig.savefig("assets/scratch/PolygonExtraction_d.pdf", bbox_inches='tight')
    plt.show()

    # Show Negative Buffer
    print("")
    print("Should see negative buffer Polygon")
    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5,5))

    patch = PolygonPatch(poly_shape, fill=True, alpha=0.5)
    ax.add_patch(patch)
    # plot_polygons([polygons[1]], points[:, :2], ax)
    poly_buffer = poly_buffer.buffer(-.2)
    patch = PolygonPatch(poly_buffer, color='k', fill=False, linewidth=1.5, linestyle='--', zorder=1)
    ax.add_patch(patch)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    print(f"After Vertices: {len(poly_buffer.exterior.coords)}")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    fig.savefig("assets/scratch/PolygonExtraction_e.pdf", bbox_inches='tight')
    plt.show()



if __name__ == "__main__":
    main()

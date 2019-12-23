import time
import math

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from polylidar import extractPlanesAndPolygons
from polylidarutil import (generate_test_points, generate_3d_plane, set_axes_equal, plot_planes_3d,
                           scale_points, rotation_matrix, apply_rotation, set_up_axes, COLOR_PALETTE)

# arguments to polylidar used throughout this example
polylidar_kwargs = dict(alpha=0.0, lmax=1.0,
                        minTriangles=20, zThresh=0.1, normThresh=0.98)
np.random.seed(1)
# generate random plane
plane = generate_3d_plane(bounds_x=[0, 10, 0.5], bounds_y=[
                          0, 10, 0.5], holes=[], height_noise=0.02, planar_noise=0.02)
# Generate 2 walls
box_side = generate_3d_plane(bounds_x=[0, 10, 0.5], bounds_y=[
                             0, 10, 0.5], holes=[], height_noise=0.02, planar_noise=0.02)
rm = rotation_matrix([0, 1, 0], -math.pi/2.0)
box_side_left = apply_rotation(rm, box_side) + [0, 0, 0]  # first wall
box_side_right = apply_rotation(rm, box_side) + [10, 0, 0]  # second wall
# All points joined together
points = np.concatenate((plane, box_side_left, box_side_right))

# create figure and axes
fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1, num='Base Frame',
                       subplot_kw=dict(projection='3d'))
# plot points
ax.scatter(*scale_points(points), c='k', s=0.4)
set_up_axes(fig, ax)
plt.show(block=False)
print("Raw Point Cloud of ground floor and two walls (Base Frame). Don't close figure.")
print("This example will show how to extract all three planes.")
print("During this process you will learn the limitations of Polylidar for 3D point clouds and how to work around them.")
input("Press Enter to Continue: ")
print("")

# extract ground plane
print("Extracting ground plane from raw point cloud. Bottom Plane Extracted")
delaunay, planes, polygons = extractPlanesAndPolygons(
    points,  **polylidar_kwargs)
triangles = np.asarray(delaunay.triangles).reshape(
    int(len(delaunay.triangles) / 3), 3)
plot_planes_3d(points, triangles, planes, ax)
plt.show(block=False)
input("Press Enter to Continue: ")
print("")


# Rotate Point Cloud for walls
print("The walls must be rotated to be extracted such that their normal is (0,0,1). Rotated Frame.")
fig2, ax2 = plt.subplots(figsize=(10, 10), nrows=1, ncols=1,  num='Rotated Frame',
                         subplot_kw=dict(projection='3d'))
# Transpose provides reverse rotation
pc_rot = apply_rotation(rm.T, points)
scatter = ax2.scatter(*scale_points(pc_rot), c='k', s=0.4)
set_up_axes(fig, ax2)
plt.show(block=False)
input("Press Enter to Continue: ")
print("")

# Seperate point clouds
print("Unfortunately these 2 walls will interefere with eachother when projected to the z=0 xy-Plane).")
print("They must be seperated into two different point clouds (orange/green)")
ax2.clear()
pc_top = pc_rot[pc_rot[:, 2] > -5.0, :]
pc_bottom = pc_rot[pc_rot[:, 2] < -5.0, :]
scatter1 = ax2.scatter(*scale_points(pc_top), c=[COLOR_PALETTE[1]], s=0.4)
scatter2 = ax2.scatter(*scale_points(pc_bottom), c=[COLOR_PALETTE[2]], s=0.4)
plt.show(block=False)

input("Press Enter to Continue: ")
print("")
# Extract planes from top and bottom
t1 = time.time()
delaunay_top, planes_top, polygons_top = extractPlanesAndPolygons(
    pc_top,  **polylidar_kwargs)
triangles_top = np.asarray(delaunay_top.triangles).reshape(
    int(len(delaunay_top.triangles) / 3), 3)
delaunay_bot, planes_bot, polygons_bot = extractPlanesAndPolygons(
    pc_bottom,  **polylidar_kwargs)
triangles_bot = np.asarray(delaunay_bot.triangles).reshape(
    int(len(delaunay_bot.triangles) / 3), 3)
t2 = time.time()


plot_planes_3d(pc_top, triangles_top, planes_top, ax2, color=COLOR_PALETTE[1])
plot_planes_3d(pc_bottom, triangles_bot, planes_bot, ax2, color=COLOR_PALETTE[2])
plt.show(block=False)
print("Showing newly extracted top and bottom planes in Rotated Frame.")
input("Press Enter to Continue: ")
print("")


print("Putting the extracted planes all together back in the Base Frame")
# just need to rotate the point cloud back to Base Frame
pc_top = apply_rotation(rm, pc_top) 
pc_bottom = apply_rotation(rm, pc_bottom)
plot_planes_3d(pc_top, triangles_top, planes_top, ax, color=COLOR_PALETTE[1])
plot_planes_3d(pc_bottom, triangles_bot, planes_bot, ax, color=COLOR_PALETTE[2])
fig.show()
input("Press Enter to Continue: ")

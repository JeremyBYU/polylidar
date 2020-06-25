"""
Simple example demonstrating a bilateral filter implented in C++.
Note that this is NOT the accelerated bilateral filter discussed in the Paper. This is just something fun I tried out that works for generalized meshes.
The accelerated bilateral filter for **organized** point clouds is found in OrganizedPointFilters repo (not this repo).
Requires: Fixtures data
"""
import time
import logging
import warnings

import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d


from examples.python.util.mesh_util import get_mesh_data_iterator
from polylidar import bilateral_filter_normals
from polylidar.polylidarutil.open3d_util import open_3d_mesh_to_trimesh

def main():
    for i, mesh in enumerate(get_mesh_data_iterator()):
        if i < 0:
            continue
        mesh.compute_vertex_normals()
        mesh.compute_triangle_normals()
        print("Before")
        o3d.visualization.draw_geometries([mesh])

        tri_mesh = open_3d_mesh_to_trimesh(mesh)
        t1 = time.perf_counter()
        bilateral_filter_normals(tri_mesh, iterations=20, sigma_length=0.1, sigma_angle=0.1)
        t2 = time.perf_counter()
        print(t2-t1)
        normals_smooth = np.asarray(tri_mesh.triangle_normals)
        mesh.triangle_normals = o3d.utility.Vector3dVector(normals_smooth)

        print("After")
        o3d.visualization.draw_geometries([mesh])



if __name__ == "__main__":
    main()

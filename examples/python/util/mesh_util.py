import time
import math
import sys
from os import path, listdir
from os.path import exists, isfile, join, splitext
import re
import logging

import numpy as np
from polylidar.polylidarutil import COLOR_PALETTE
from polylidar.polylidarutil.line_mesh import o3d_major_version
import open3d as o3d
from scipy.spatial.transform import Rotation as R

DIR_NAME = path.dirname(__file__)
FIXTURES_DIR = path.join(DIR_NAME, '../../..', 'fixtures')
MESHES_DIR = path.join(FIXTURES_DIR, 'meshes')

DENSE_MESH = path.join(MESHES_DIR, 'dense_first_floor_map_smoothed.ply')
SPARSE_MESH = path.join(MESHES_DIR, 'sparse_basement.ply')
BASEMENT_CHAIR = path.join(MESHES_DIR, 'basement_chair_5cm.ply')

ALL_MESHES = [DENSE_MESH, SPARSE_MESH, BASEMENT_CHAIR]
# ALL_MESHES_ROTATIONS = [R.from_rotvec(-np.pi / 2 * np.array([1, 0, 0])),
#                         R.from_rotvec(-np.pi / 2 * np.array([1, 0, 0]))]

ALL_MESHES_ROTATIONS = [None, None, None]


def get_mesh_data_iterator():
    for i, (mesh_fpath, r) in enumerate(zip(ALL_MESHES, ALL_MESHES_ROTATIONS)):
        example_mesh = o3d.io.read_triangle_mesh(str(mesh_fpath))
        if r is not None:
            center = [0, 0, 0] if o3d_major_version > 9 else True
            example_mesh = example_mesh.rotate(r.as_matrix(), center=center)
        example_mesh_filtered = example_mesh
        example_mesh_filtered.compute_triangle_normals()
        yield example_mesh_filtered


def main():
    for i, mesh in enumerate(get_mesh_data_iterator()):
        if i < 1:
            continue
        colors = np.asarray(mesh.vertex_colors)
        colors2 = np.column_stack((colors[:, 2], colors[:, 1], colors[:, 0]))
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors2)
        o3d.io.write_triangle_mesh('test.ply', mesh)

if __name__ == "__main__":
    main()

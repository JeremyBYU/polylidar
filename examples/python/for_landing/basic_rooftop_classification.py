import time
import logging
import sys
import functools
import operator
from pathlib import Path

import numpy as np
import open3d as o3d
import matplotlib.cm as cm
import yaml


from polylidar import (Polylidar3D, MatrixDouble, MatrixFloat, extract_tri_mesh_from_float_depth,
                       extract_point_cloud_from_float_depth, MatrixUInt8)

from polylidar.polylidarutil.open3d_util import construct_grid, create_lines, flatten, create_open_3d_mesh_from_tri_mesh
from polylidar.polylidarutil.plane_filtering import filter_planes_and_holes
from polylidar.polylidarutil.line_mesh import LineMesh

from organizedpointfilters.utility.helper import create_meshes_cuda
from fastga import GaussianAccumulatorS2Beta, IcoCharts

from examples.python.util.helper_polylidar import extract_all_dominant_plane_normals, extract_planes_and_polygons_from_mesh, extract_planes_and_polygons_from_classified_mesh
from examples.python.util.helper_mesh import create_open_3d_mesh_from_tri_mesh



def convert_to_square_image(pc, invert=False, row=None):
    if invert:
        row = int(pc.shape[0])
        col = int(pc.shape[1])
        new_pc = pc.reshape((row * col, 3))
    else:
        if row is None:
            row = int(np.sqrt(pc.shape[0]))
            col = row
        else:
            col = int(pc.shape[0] / row)
        shape = (row, col, 3) if pc.ndim > 1 else (row, col)
        new_pc = pc.reshape(shape)
    return new_pc


def create_color(classes):
    classes_ = (classes)
    colors = cm.tab20(classes_)
    colors = colors[:, :, :3]
    colors = np.copy(convert_to_square_image(colors, True))
    return colors

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

def handle_shapes(planes, obstacles, line_radius=0.15):
    all_polys = []

    for plane, _ in planes:
        points = np.array(plane.exterior)
        line_mesh = LineMesh(points, colors=[0,1,0], radius=line_radius)
        all_polys.append(line_mesh)

    for plane, _ in obstacles:
        points = np.array(plane.exterior)
        line_mesh = LineMesh(points, colors=[1, 0, 0], radius=line_radius)
        all_polys.append(line_mesh)

    all_meshes = [poly.cylinder_segments for poly in all_polys]
    all_meshes = functools.reduce(operator.iconcat, all_meshes, [])

    return all_polys, all_meshes

def main():
    # Load yaml file
    config = None
    with open('./examples/python/for_landing/PolylidarParams.yaml') as file:
        try:
            config = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print("Error parsing yaml")

    example = Path("./fixtures/unreal/example1")
    pc_lidar = np.load(example / "140-0-0.npy")

    pc_image = convert_to_square_image(pc_lidar[:, :3], row=64)
    classes = convert_to_square_image(pc_lidar[:, 3], row=64).astype(np.uint8)
    colors = create_color(classes)
    mask = classes == 4
    classes[~mask] =  0
    classes[mask] = 1


    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_lidar[:, :3]))
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])


    mesh_o3d, planes, obstacles, time = extract_polygons(pc_image, classes, config)


    all_polys, all_meshes = handle_shapes(planes, obstacles)
    all_meshes = [mesh_o3d, *all_meshes]
    o3d.visualization.draw_geometries([mesh_o3d, pcd, *all_meshes])

    


if __name__ == "__main__":
    main()

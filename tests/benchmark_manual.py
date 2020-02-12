from pathlib import Path
from os import path
import pickle

from polylidar import extractPlanesAndPolygons, extract_planes_and_polygons_from_mesh, extract_point_cloud_from_float_depth, extract_uniform_mesh_from_float_depth

DIR_NAME = path.dirname(__file__)
FIXTURES_DIR = path.join(DIR_NAME, 'fixtures')
REALSENSE_DIR = path.join(FIXTURES_DIR, 'realsense')
BIG_MESH = path.join(REALSENSE_DIR, 'realsense_mesh_big.pkl')

MAX_ITER = 100


def get_data_from_file(fname):
    return pickle.load(open(fname, 'rb'))

def main():
    polylidar_kwargs = dict(alpha=0.0, lmax=0.10, minTriangles=100,
                            zThresh=0.03, normThresh=0.99, normThreshMin=0.95, minHoleVertices=6)
    data = get_data_from_file(BIG_MESH)
    for _ in range(MAX_ITER):
        _,_ = extract_planes_and_polygons_from_mesh(data['vertices'], data['triangles'], data['halfedges'], **polylidar_kwargs)


if __name__ == "__main__":
    main()
from os import path
import pytest
import numpy as np


from tests.helpers.utils import load_csv, verify_points, basic_polylidar_verification, verify_all_polygons_are_valid, load_npy
from polylidar import extractPlanesAndPolygons, extractPolygons


def test_building1(benchmark, building1, basic_params):
    benchmark(extractPlanesAndPolygons, building1, **basic_params)


def test_building2(benchmark, building2, basic_params):
    benchmark(extractPlanesAndPolygons, building2, **basic_params)


def test_100k_array_lmax(benchmark, np_100K_array, params_lmax):
    benchmark(extractPolygons, np_100K_array, **params_lmax)

def test_100k_array_3d_lmax(benchmark, np_100K_array_3d, params_lmax):
    benchmark(extractPolygons, np_100K_array_3d, **params_lmax)

def test_clusters(benchmark, cluster_groups):
    points = cluster_groups['points']
    polylidar_kwargs = cluster_groups['polylidar_kwargs']
    benchmark(extractPolygons, points, **polylidar_kwargs)

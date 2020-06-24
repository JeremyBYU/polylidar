from os import path
import pytest
import numpy as np


from tests.python.helpers.utils import verify_points, basic_polylidar_verification, verify_all_polygons_are_valid, verify_all


np.random.seed(1)

def test_verify_building1(building1):
    verify_points(building1, 4)

def test_verify_building2(building2):
    verify_points(building2, 4)

def test_verify_hardcase1(hardcase1):
    verify_points(hardcase1, 2)

def test_building1(building1, basic_params):
    verify_all(building1, basic_params)


def test_building2(building2, basic_params):
    verify_all(building2, basic_params)

def test_hardcase1(hardcase1, hardcase1_params):
    verify_all(hardcase1, hardcase1_params)

# def test_bad_convex_hull(bad_convex_hull, bad_convex_hull_params):
#     verify_all(bad_convex_hull, bad_convex_hull_params)

def test_random_points(random_points, params_lmax):
    verify_all(random_points, params_lmax)

def test_clusters(cluster_groups):
    points = cluster_groups['points']
    polylidar_kwargs = cluster_groups['polylidar_kwargs']
    verify_all(points, polylidar_kwargs)



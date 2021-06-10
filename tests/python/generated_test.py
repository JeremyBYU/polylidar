from os import path
import pytest
import numpy as np


from tests.python.helpers.utils import verify_points, basic_polylidar_verification, verify_all_polygons_are_valid, verify_all


np.random.seed(1)

def test_random_points(random_points, params_lmax):
    verify_all(random_points, params_lmax)

def test_clusters(cluster_groups):
    points = cluster_groups['points']
    polylidar_kwargs = cluster_groups['polylidar_kwargs']
    verify_all(points, polylidar_kwargs)



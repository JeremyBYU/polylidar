from os import path
import pytest
import numpy as np


from tests.python.helpers.utils import load_csv, load_npy
from polylidar.polylidarutil import generate_test_points, get_estimated_lmax

np.random.seed(1)


@pytest.fixture
def building1():
    return load_csv('building1.csv')


@pytest.fixture
def building2():
    return load_csv("building2.csv")


@pytest.fixture
def hardcase1():
    return load_csv("hardcase1.csv")


# @pytest.fixture
# def bad_convex_hull():
#     return load_npy("possible_error_free.npy")


@pytest.fixture()
def basic_params():
    return dict(alpha=0.5)


@pytest.fixture()
def params_lmax():
    return dict(alpha=0.0, lmax=100.0)


@pytest.fixture()
def hardcase1_params():
    return dict(alpha=0.0, lmax=20.0)


@pytest.fixture
def np_100K_array():
    return load_csv("100K_array_2d.csv")


@pytest.fixture
def np_100K_array_3d():
    return load_csv("100K_array_3d.csv")


@pytest.fixture()
def bad_convex_hull_params():
    return dict(alpha=0.0, lmax=1300.0)

@pytest.fixture(params=range(2, 11))
def num_groups(request):
    return request.param

@pytest.fixture(params=[1000, 10000])
def group_size(request):
    return request.param

@pytest.fixture
def cluster_groups(num_groups, group_size):
    cluster_params = dict(num_groups=num_groups, group_size=group_size)
    points = generate_test_points(**cluster_params)
    lmax = get_estimated_lmax(**cluster_params)
    polylidar_kwargs = dict(alpha=0.0, lmax=lmax)
    return dict(points=points, polylidar_kwargs=polylidar_kwargs, cluster_params=cluster_params)

# This creates 100 numpy arrays frangin from (1000,2) -> (100000,2)
ts = range(1000, 100000, 1000)
@pytest.fixture(params=ts)
def random_points(request):
    points = np.random.randn(request.param, 2) * 100 + 700000
    return points

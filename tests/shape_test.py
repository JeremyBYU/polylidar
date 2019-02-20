from os import path
import pytest
import numpy as np


from tests.helpers.utils import load_csv, verify_points, basic_polylidar_verification, verify_all_polygons_are_valid
from polylidar import extractPlanesAndPolygons, extractPolygons


@pytest.fixture
def building1():
    return load_csv('building1.csv')

@pytest.fixture
def building2():
    return load_csv("building2.csv")

@pytest.fixture
def hardcase1():
    return load_csv("hardcase1.csv")

@pytest.fixture()
def basic_params():
    return dict(alpha=0.5, xyThresh=0.0)

@pytest.fixture()
def hardcase1_params():
    return dict(alpha=0.0, xyThresh=20.0)

def test_verify_building1(building1):
    verify_points(building1, 4)

def test_verify_building2(building2):
    verify_points(building2, 4)

def test_verify_hardcase1(hardcase1):
    verify_points(hardcase1, 2)

def test_building1(building1, basic_params):
    delaunay, planes, polygons = extractPlanesAndPolygons(building1, **basic_params)
    # Basic test to ensure no obvious errors occurred
    basic_polylidar_verification(building1, delaunay, planes, polygons)
    # Ensure that the polygons returned are valid
    verify_all_polygons_are_valid(polygons, building1)
    # Test just polygon extraction
    polygons = extractPolygons(building1, **basic_params)
    # Ensure that the polygons returned are valid
    verify_all_polygons_are_valid(polygons, building1)


def test_building2(building2, basic_params):
    delaunay, planes, polygons = extractPlanesAndPolygons(building2, **basic_params)
    # Basic test to ensure no obvious errors occurred
    basic_polylidar_verification(building2, delaunay, planes, polygons)
    # Ensure that the polygons returned are valid
    verify_all_polygons_are_valid(polygons, building2)
    # Ensure that all polygons are as expected
    # Test just polygon extraction
    polygons = extractPolygons(building2, **basic_params)
    # Ensure that the polygons returned are valid
    verify_all_polygons_are_valid(polygons, building2)

def test_hardcase1(hardcase1, hardcase1_params):
    delaunay, planes, polygons = extractPlanesAndPolygons(hardcase1, **hardcase1_params)
    # Basic test to ensure no obvious errors occurred
    basic_polylidar_verification(hardcase1, delaunay, planes, polygons)
    # Ensure that the polygons returned are valid
    verify_all_polygons_are_valid(polygons, hardcase1)
    # Ensure that all polygons are as expected
    # Test just polygon extraction
    polygons = extractPolygons(hardcase1, **hardcase1_params)
    # Ensure that the polygons returned are valid
    verify_all_polygons_are_valid(polygons, hardcase1)


ts = range(1000, 100000, 1000)  # This creates 100 numpy arrays frangin from (1000,2) -> (100000,2)
np.random.seed(1)
@pytest.fixture(params=ts)
def random_points(request):
    points = np.random.randn(request.param, 2) * 100 +  700000
    # points[0, :] = [1.0,1.0]
    # points[10, :] = [1.0,1.0]
    return points

def test_random_points(random_points):
    delaunay, planes, polygons = extractPlanesAndPolygons(random_points)
    # Basic test to ensure no obvious errors occurred
    # basic_polylidar_verification(random_points, delaunay, planes, polygons)
    # Ensure that the polygons returned are valid
    verify_all_polygons_are_valid(polygons, random_points)




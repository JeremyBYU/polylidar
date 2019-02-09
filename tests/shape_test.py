from os import path
import pytest
import numpy as np


from tests.helpers.utils import load_csv, verify_points, basic_polylidar_verification, verify_all_polygons_are_valid
from polylidar import extractPlanesAndPolygons


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
    # Ensure that all polygons are as expected






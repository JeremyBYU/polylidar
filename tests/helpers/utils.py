from os import path
import pytest
import numpy as np

from shapely.geometry import Polygon
from polylidar import extractPlanesAndPolygons

DIR_NAME =path.dirname(path.dirname(__file__))
FIXTURES_DIR = path.join(DIR_NAME, 'fixtures')

def get_poly_coords(outline, points):
    return [get_point(pi, points) for pi in outline]

def get_point(pi, points):
    return [points[pi, 0], points[pi, 1]]

def load_csv(file):
    return np.loadtxt(path.join(FIXTURES_DIR, file), delimiter=',', dtype=np.float64, skiprows=1)

def verify_points(data, n_cols=2):
    assert isinstance(data, np.ndarray)
    assert data.shape[1] == n_cols

def basic_polylidar_verification(points, delaunay, planes, polygons):
    # make sure delaunay has the correct number of points
    assert points.shape[0] == len(delaunay.coords) / 2
    # make sure that some planes were extracted
    assert len(planes) > 0
    # make sure that some polygons were extracted
    assert len(polygons) > 0

def verify_all_polygons_are_valid(polygons, points):
    for poly in polygons:
        verify_valid_polygon(poly, points)

def verify_valid_polygon(poly, points):
    shell_coords = get_poly_coords(poly.shell, points)
    hold_coords = [get_poly_coords(hole, points) for hole in poly.holes]
    # Verify polygon outer shell
    shapelyPoly = Polygon(shell=shell_coords)
    assert shapelyPoly.is_valid
    # Verify polygon outer shell
    shapelyPoly = Polygon(shell=shell_coords, holes=hold_coords)
    assert shapelyPoly.is_valid




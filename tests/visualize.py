
from os import path
import time
import numpy as np
from polylidar import Delaunator, extractPlanesAndPolygons
from polylidarutil import (generate_test_points, plot_points, plot_triangles,
                           plot_triangle_meshes, get_triangles_from_he, get_plane_triangles, plot_polygons)
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from descartes import PolygonPatch

DIR_NAME = path.dirname(__file__)
FIXTURES_DIR = path.join(DIR_NAME, 'fixtures')



def load_csv(file):
    return np.loadtxt(path.join(FIXTURES_DIR, file), delimiter=',', dtype=np.float64, skiprows=1)


# points = np.array(pointsList)
# np.savetxt('hardcase2d.csv', points, fmt='%.4f', header="X,Y")

# points = generate_test_points(num_groups=10000, seed=1)
points = load_csv('building1.csv')
# points = np.load("tests/fixtures/possible_error_free.npy")
# points = np.load("scratch/error_coincident.npy")
# print(points.flags)
points = np.ascontiguousarray(points[:, :2])
# noise = np.random.randn(points.shape[0], 2) * .10
# points[:, :2] = points[:, :2] + noise
# print(points.flags)
print("Point Shape {}".format(points.shape))

t1 = time.time()
delaunay, planes, polygons = extractPlanesAndPolygons(points, xyThresh=0.0, alpha=0.0, lmax=5.0, minTriangles=20)
t2 = time.time()
print("Took {:.2f} milliseconds".format((t2 - t1) * 1000))

# import pdb; pdb.set_trace()

if points.shape[0] < 100000:
    fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
    # plot points
    plot_points(points, ax)
    # plot all triangles
    plot_triangles(get_triangles_from_he(delaunay.triangles, points), ax)
    # plot mesh triangles
    triangle_meshes = get_plane_triangles(planes, delaunay.triangles, points)
    plot_triangle_meshes(triangle_meshes, ax)
    # plot polygons
    plot_polygons(polygons, delaunay, points, ax)

    plt.axis('equal')

    plt.show()

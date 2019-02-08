from os import path
import time
import numpy as np
from polylidar import Delaunator, extractPlanesAndPolygons
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from descartes import PolygonPatch
import seaborn as sns

from testing.fixtures.hardcase1 import pointsList

DIR_NAME =path.dirname(__file__)
FIXTURES_DIR = path.join(DIR_NAME, 'fixtures')

COLOR_PALETTE = sns.color_palette()

def load_csv(file):
    return np.loadtxt(path.join(FIXTURES_DIR, file), delimiter=',', dtype=np.float64)

def plot_polygons(polygons, delaunay, points, ax):
    for poly in polygons:
        shell_coords = [get_point(pi, points) for pi in poly.shell]
        outline = Polygon(shell=shell_coords)
        outlinePatch = PolygonPatch(outline, ec='green', fill=False, linewidth=2)
        ax.add_patch(outlinePatch)

        for hole_poly in poly.holes:
            shell_coords = [get_point(pi, points) for pi in hole_poly]
            outline = Polygon(shell=shell_coords)
            outlinePatch = PolygonPatch(outline, ec='orange', fill=False, linewidth=2)
            ax.add_patch(outlinePatch)

        
def generate_test_points(num_groups=2, dist = 100, group_size=10, seed=1):
    np.random.seed(1)
    sigma = dist / (num_groups)
    cov = np.array([[sigma, 0], [0, sigma]])
    a = np.array([0, 0])
    for i in range(0, num_groups):
        mean = np.random.random(2) * 100
        b = np.random.multivariate_normal(mean, cov, size=group_size)
        a = np.vstack((a, b))

    return a

def get_point(pi, coords):
    return [points[pi, 0], points[pi, 1]]

def get_triangles_from_he(triangles, points):
    triangle_list = []
    for i in range(0, len(triangles), 3):
        # print(triangles)
        triangle = []
        p0 = triangles[i]
        p1 = triangles[i + 1]
        p2 = triangles[i + 2]
        triangle.append(get_point(p0, points))
        triangle.append(get_point(p1, points))
        triangle.append(get_point(p2, points))
        triangle_list.append(triangle)
    return triangle_list

def get_plane_triangles(planes, triangles, points):
    triangle_meshes = []
    for i,plane in enumerate(planes):
        # Get color for plane mesh
        j = i
        if j >= len(COLOR_PALETTE):
            j = j % len(COLOR_PALETTE)

        plane_mesh = {'color': COLOR_PALETTE[j] , 'triangles': []}
        triangle_list = []
        # Get triangle coordinates
        for t in plane:
            # print(triangles)
            triangle = []
            p0 = triangles[t * 3]
            p1 = triangles[t* 3 + 1]
            p2 = triangles[t * 3 + 2]
            triangle.append(get_point(p0, points))
            triangle.append(get_point(p1, points))
            triangle.append(get_point(p2, points))
            triangle_list.append(triangle)
        plane_mesh['triangles'] = triangle_list
        triangle_meshes.append(plane_mesh)
    return triangle_meshes


def get_all_triangles(delaunay, points):
    triangles = delaunay.triangles
    coords = delaunay.coords
    triangle_coords = get_triangles_from_he(triangles, points)
    return triangle_coords

def plot_triangle_meshes(plane_triangles, ax):
    for plane in plane_triangles:
        color = plane['color']
        triangle_coords = plane['triangles']
        plot_triangles(triangle_coords, ax, color=color, fill=True)

def plot_triangles(triangle_coords, ax, color=COLOR_PALETTE[0], fill=False, ec='black'):
    # print(triangle_coords)
    triangles_shapely = [Polygon(triangle) for triangle in triangle_coords]
    for shape in triangles_shapely:
        patch = PolygonPatch(shape, color=color, ec=ec, alpha=0.5, fill=fill)
        ax.add_patch(patch)

def plot_points(points, ax):
    ax.scatter(points[:, 0], points[:, 1], c='k')
    # print(triangles_shapely)

    
# points = np.array(pointsList)
# points = generate_test_points(num_groups=10000, seed=1)
points = load_csv('building6_example2_360.csv')
points = points[:, :3]
print("Point Shape {}".format(points.shape))

t1 = time.time()
delaunay, planes, polygons = extractPlanesAndPolygons(points, alpha=0.5, xyThresh=0.0)
t2 = time.time()
print("Took {:.2f} milliseconds".format((t2 - t1) * 1000))


if points.shape[0] < 10000:
    fig, ax = plt.subplots(figsize=(10,10), nrows=1, ncols=1)

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

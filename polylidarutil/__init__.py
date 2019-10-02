import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors 
from shapely.geometry import Polygon
from descartes import PolygonPatch

COLOR_PALETTE = list(map(colors.to_rgb, plt.rcParams['axes.prop_cycle'].by_key()['color']))

def get_poly_coords(outline, points):
    return [get_point(pi, points) for pi in outline]

def get_point(pi, points):
    """Get a point from a numpy array.
    If the numpy array has 3 dimensions, will return all three (x,y,z)
    Arguments:
        pi {int} -- Point index in numpy array
        points {ndarray} -- Numpy array of points
    
    Returns:
        [list] -- List of poins [x,y,(z)]
    """
    if points.shape[1] > 2:
        return [points[pi, 0], points[pi, 1], points[pi, 2]]
    else:
        return [points[pi, 0], points[pi, 1]]

def convert_to_shapely_polygons(polygons, points, return_first=False, sort=False, mp=False):
    """Convert Polylidar C++ polygons to shapely polygons
    
    Arguments:
        polygons {vector<polygons>} -- Vector of polygons
        points {ndarray} -- ndarray
    
    Keyword Arguments:
        return_first {bool} -- Return the first polygon (default: {False})
        sort {bool} -- Sort polygons by largest number of vertices  (default: {False})
        mp {bool} -- Convert list of polygons to MultiPolygon (default: {False})
    
    Returns:
        [list or single Polygon or Single Multipolygon] -- Its up to you what you want
    """
    if sort:
        polygons.sort(key=lambda poly: len(poly.shell), reverse=True)

    shapely_polygons = []
    for poly in polygons:
        shell_coords = get_poly_coords(poly.shell, points)
        hole_coords = [get_poly_coords(hole, points) for hole in poly.holes]
        poly_shape = Polygon(shell=shell_coords, holes=hole_coords)
        
        shapely_polygons.append(poly_shape)

    # Return only the largest by number of vertices
    if shapely_polygons and return_first:
        return shapely_polygons[0]

    # Check if a multipolygon
    if len(shapely_polygons) > 1 and mp:
        return MultiPolygon(shapely_polygons)
    else:
        return shapely_polygons

def set_axes_radius(ax, origin, radius):
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    set_axes_radius(ax, origin, radius)


def generate_3d_plane(bounds_x=[0,10,0.5], bounds_y=[0, 10, 0.5], holes=[[[3,5], [3, 5]]], height=0, planar_noise=0.01, height_noise=0.1):
    X, Y = np.mgrid[bounds_x[0]:bounds_x[1]:bounds_x[2], bounds_y[0]:bounds_y[1]:bounds_y[2]]
    pc = np.column_stack((X.ravel(), Y.ravel()))

    plane_noise = np.abs(np.random.randn(pc.shape[0], 2)) * planar_noise
    pc[:,:2] = pc[:,:2] + plane_noise

    plane_height = (np.abs(np.random.randn(pc.shape[0])) * height_noise) + height
    pc_noisy = np.column_stack((pc[:,0], pc[:, 1], plane_height))

    mask = np.zeros(pc_noisy.shape[0], dtype=bool)
    for hole in holes:
        holes_x, holes_y = hole
        mask1 = (pc_noisy[:,0] > holes_x[0]) & (pc_noisy[:,0] < holes_x[1])
        mask2 = (pc_noisy[:,1] > holes_y[0]) & (pc_noisy[:,1] < holes_y[1])
        mask3 = mask1 & mask2
        mask = mask | mask3

    return pc_noisy[~mask]


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

        
def generate_test_points(num_groups=2, dist=100, group_size=10, seed=1, spread_factor=1.0):
    np.random.seed(seed)
    sigma = math.sqrt(dist / (num_groups)) * spread_factor
    cov = np.array([[sigma, 0], [0, sigma]])
    a = np.array([0, 0])
    for i in range(0, num_groups):
        mean = np.random.random(2) * dist
        b = np.random.multivariate_normal(mean, cov, size=group_size)
        a = np.vstack((a, b))

    return a

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
    triangle_coords = get_triangles_from_he(triangles, points)
    return triangle_coords


def scale_points(points, z_value=None, z_scale=1.0):
    if z_value is not None:
        return points[:,0], points[:, 1], np.ones_like(points[:, 1]) * z_value
    else:
        return points[:,0], points[:, 1], points[:,2] * z_scale

def plot_planes_3d(lidar_building, triangles, planes, ax, alpha=1.0, z_scale=1.0):
    ax_planes = []
    for i, plane in enumerate(planes):
        triangles_plane = triangles[plane]
        ax_plane = ax.plot_trisurf(*scale_points(lidar_building, z_scale=z_scale),triangles=triangles_plane, color=(COLOR_PALETTE[i] + (alpha, )),  edgecolor=(0,0,0,0.3), linewidth=0.5)
        ax_planes.append(ax_plane)
    return ax_planes

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
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors 
from shapely.geometry import Polygon, MultiPolygon
from descartes import PolygonPatch

COLOR_PALETTE = list(map(colors.to_rgb, plt.rcParams['axes.prop_cycle'].by_key()['color']))

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

def apply_rotation(rm, points):
    return (rm @ points.T).T

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

def set_axes_radius(ax, origin, radius, ignore_z=False):
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    if not ignore_z:
        ax.set_zlim3d([origin[2] - radius, origin[2] + radius])


def set_axes_equal(ax, ignore_z=False):
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
    set_axes_radius(ax, origin, radius, ignore_z=ignore_z)


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


def plot_polygons(polygons, points, ax, linewidth=2, shell_color='green', hole_color='orange'):
    for poly in polygons:
        shell_coords = [get_point(pi, points) for pi in poly.shell]
        outline = Polygon(shell=shell_coords)
        outlinePatch = PolygonPatch(outline, ec=shell_color, fill=False, linewidth=linewidth)
        ax.add_patch(outlinePatch)

        for hole_poly in poly.holes:
            shell_coords = [get_point(pi, points) for pi in hole_poly]
            outline = Polygon(shell=shell_coords)
            outlinePatch = PolygonPatch(outline, ec=hole_color, fill=False, linewidth=linewidth)
            ax.add_patch(outlinePatch)

        
def generate_test_points(num_groups=2, dist=100, group_size=10, seed=1):
    np.random.seed(seed)
    sigma = dist / (num_groups * 2 - 1)
    cov = np.array([[sigma, 0], [0, sigma]])
    a = np.array([0, 0])
    for i in range(0, num_groups):
        mean = np.random.random(2) * dist
        b = np.random.multivariate_normal(mean, cov, size=group_size)
        a = np.vstack((a, b))

    return a

def get_estimated_lmax(num_groups=2, dist=100, group_size=10, scale_factor=4, **kwargs):
    clust_radius = math.sqrt(dist / (num_groups * 2 - 1))
    clust_point_density = (group_size * .50) / (math.pi * clust_radius * clust_radius)
    lmax = scale_factor / math.sqrt(clust_point_density)
    return lmax

def get_triangles_from_list(triangles:np.ndarray, points):
    triangle_list = []
    skip = 3 if triangles.ndim == 1 else 1
    for i in range(0, len(triangles), skip):
        # print(triangles)
        triangle = []
        if triangles.ndim == 1:
            p0 = triangles[i]
            p1 = triangles[i + 1]
            p2 = triangles[i + 2]
        else:
            p0 = triangles[i, 0]
            p1 = triangles[i, 1]
            p2 = triangles[i, 2]
        triangle.append(get_point(p0, points))
        triangle.append(get_point(p1, points))
        triangle.append(get_point(p2, points))
        triangle_list.append(triangle)

    return triangle_list

def get_colored_planar_segments(planes, triangles:np.ndarray, points):
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
            if triangles.ndim == 1:
                p0 = triangles[t * 3]
                p1 = triangles[t* 3 + 1]
                p2 = triangles[t * 3 + 2]
            else:
                p0 = triangles[t, 0]
                p1 = triangles[t, 1]
                p2 = triangles[t, 2]
            triangle.append(get_point(p0, points))
            triangle.append(get_point(p1, points))
            triangle.append(get_point(p2, points))
            triangle_list.append(triangle)
        plane_mesh['triangles'] = triangle_list
        triangle_meshes.append(plane_mesh)
    return triangle_meshes

def get_all_triangles(delaunay, points):
    triangles = delaunay.triangles
    triangle_coords = get_triangles_from_list(triangles, points)
    return triangle_coords


def scale_points(points, z_value=None, z_scale=1.0):
    if z_value is not None:
        return points[:,0], points[:, 1], np.ones_like(points[:, 1]) * z_value
    else:
        return points[:,0], points[:, 1], points[:,2] * z_scale

def set_up_axes(fig, ax, x_label='X', y_label='Y', z_label='Z', axis_equal=True):
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_zlabel(z_label)
    if axis_equal:
        set_axes_equal(ax)

def plot_planes_3d(lidar_building, triangles, planes, ax, alpha=1.0, z_value=None, z_scale=1.0, color=None):
    ax_planes = []
    for i, plane in enumerate(planes):
        triangles_plane = triangles[plane]
        color_ = COLOR_PALETTE[i] if color is None else color
        color_ = color_ + (alpha, )
        ax_plane = ax.plot_trisurf(*scale_points(lidar_building, z_value=z_value, z_scale=z_scale),triangles=triangles_plane, color=color_,  edgecolor=(0,0,0,0.3), linewidth=0.5)
        ax_planes.append(ax_plane)
    return ax_planes

def plot_polygons_3d(points, polygons, ax, color=None, linewidth=6):
    for i, polygon in enumerate(polygons):
        shell = points[np.array(polygon.shell), :]
        color_ = COLOR_PALETTE[i] if color is None else color
        ax.plot(shell[:, 0], shell[:,1], shell[:, 2], c=color_, linewidth=linewidth, label="Polygon Outline")
        for hole in polygon.holes:
            hole = points[np.array(hole), :]
            ax.plot(hole[:, 0], hole[:,1], hole[:, 2], c='k', linewidth=linewidth)

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
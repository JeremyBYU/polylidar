from copy import deepcopy
import time
import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

from examples.python.realsense_util import create_open_3d_mesh

GOLDEN_RATIO = (1.0 + np.sqrt(5.0)) / 2.0
ICOSAHEDRON_TRUE_RADIUS = np.sqrt(1 + np.power(GOLDEN_RATIO, 2))
ICOSAHEDRON_SCALING = 1.0 / ICOSAHEDRON_TRUE_RADIUS
ICOSAHEDRON_SCALED_EDGE_LENGTH = ICOSAHEDRON_SCALING * 2.0

EXAMPLE_MESH_1 = './tests/fixtures/realsense/example_mesh.ply'
EXAMPLE_MESH_2 = './scratch/dense_first_floor_map.ply'
EXAMPLE_MESH_3 = './scratch/sparse_basement.ply'

ALL_MESHES = [EXAMPLE_MESH_1, EXAMPLE_MESH_2, EXAMPLE_MESH_3]
ALL_MESHES_ROTATIONS = [np.identity(3), R.from_rotvec(-np.pi / 2 * np.array([1, 0, 0])),
                        R.from_rotvec(-np.pi / 2 * np.array([1, 0, 0]))]


def get_colors(inp, colormap, vmin=None, vmax=None):
    norm = plt.Normalize(vmin, vmax)
    return colormap(norm(inp))


class GaussianAccumulator(object):
    def __init__(self, gaussian_normals, leafsize=16):
        super().__init__()
        self.nbuckets = gaussian_normals.shape[0]
        self.kdtree = cKDTree(gaussian_normals, leafsize=leafsize)
        self.accumulator = np.zeros(self.nbuckets, dtype=np.float64)
        self.colors = np.zeros_like(gaussian_normals)
        self.gaussian_normals = gaussian_normals

    def integrate(self, normals):
        query_size = normals.shape[0]
        t0 = time.perf_counter()
        _, neighbors = self.kdtree.query(normals)
        t1 = time.perf_counter()
        elapsed_time = (t1 - t0) * 1000
        print("KD tree size: {}; Query Size (K): {}; Execution Time(ms): {:.1f}".format(
            self.nbuckets, query_size, elapsed_time))
        for idx in neighbors:
            self.accumulator[idx] = self.accumulator[idx] + 1

    def normalize(self):
        self.accumulator = self.accumulator / np.max(self.accumulator)
        self.colors = get_colors(self.accumulator, plt.cm.viridis)[:, :3]


def cantor_mapping(k1, k2):
    return int(((k1 + k2) * (k1 + k2 + 1)) / 2.0 + k2)


def generate_key_from_point(p1_idx, p2_idx):
    lower_idx, higher_idx = (
        p1_idx, p2_idx) if p1_idx < p2_idx else (p2_idx, p1_idx)
    return cantor_mapping(lower_idx, higher_idx)


def construct_midpoint(p1_idx, p2_idx, vertices):
    p1 = vertices[p1_idx]
    p2 = vertices[p2_idx]
    midpoint_on_plane = (p2 + p1) / 2.0
    scaling = 1 / np.linalg.norm(midpoint_on_plane)
    midpoint_on_sphere = midpoint_on_plane * scaling
    return midpoint_on_sphere


def get_point_idx(p1_idx, p2_idx, point_to_idx_map, vertices):
    point_key = generate_key_from_point(p1_idx, p2_idx)
    if point_to_idx_map.get(point_key):
        # Existing point Idx
        return point_to_idx_map[point_key]
    else:
        # New point idx
        point_to_idx_map[point_key] = len(vertices)
        midpoint_on_sphere = construct_midpoint(p1_idx, p2_idx, vertices)
        vertices.append(midpoint_on_sphere)
        return point_to_idx_map[point_key]


def plot_meshes(*meshes, shift=True):
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame()
    axis.translate([-2.0, 0, 0])
    translate_meshes = []
    current_x = 0.0
    if shift:
        for i, mesh in enumerate(meshes):
            bbox = mesh.get_axis_aligned_bounding_box()
            x_extent = bbox.get_extent()[0]
            translate_meshes.append(mesh.translate(
                [current_x + x_extent / 2.0, 0, 0]))
            current_x += x_extent + 0.5
    else:
        translate_meshes = meshes

    o3d.visualization.draw_geometries([axis, *translate_meshes])


def generate_sphere_examples():
    ico = o3d.geometry.TriangleMesh.create_icosahedron(
        radius=ICOSAHEDRON_SCALING)
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1, resolution=20)

    ico.compute_vertex_normals()
    ico.compute_triangle_normals()
    sphere.compute_vertex_normals()
    sphere.compute_triangle_normals()

    return ico, sphere


def refine_icosahedron(triangles, vertices, level=2):
    vertices = list(vertices)
    triangles = triangles.tolist()
    # HashMap that maps a midpoint (identified by two point indexes) to its own point index
    point_to_idx_map = dict()
    for i in range(level):
        triangles_refined = []
        # loop through every triangle and create 4 new ones based upon midpoints
        for triangle in triangles:
            p1_idx = triangle[0]
            p2_idx = triangle[1]
            p3_idx = triangle[2]

            # Create new points (if not existing) and return point index
            p4_idx = get_point_idx(p1_idx, p2_idx, point_to_idx_map, vertices)
            p5_idx = get_point_idx(p2_idx, p3_idx, point_to_idx_map, vertices)
            p6_idx = get_point_idx(p3_idx, p1_idx, point_to_idx_map, vertices)
            # Create the four new triangles
            t1 = [p1_idx, p4_idx, p6_idx]
            t2 = [p4_idx, p2_idx, p5_idx]
            t3 = [p6_idx, p5_idx, p3_idx]
            t4 = [p6_idx, p4_idx, p5_idx]
            # Append triangles to the new refined triangle array
            triangles_refined.extend([t1, t2, t3, t4])
        # overwrite existing triangles with this new array
        triangles = triangles_refined
    vertices = np.array(vertices)
    triangles = np.array(triangles)
    return vertices, triangles


def generate_family_of_icosahedron(triangles, vertices, family=[1, 2, 3, 4]):
    meshes = []
    for level in family:
        new_vertices, new_triangles = refine_icosahedron(
            triangles, vertices, level=level)
        new_mesh = create_open_3d_mesh(new_triangles, new_vertices)
        meshes.append(new_mesh)
    return meshes


def calc_angle_delta(mesh, level):
    normals = np.asarray(mesh.triangle_normals)
    v1 = normals[0, :]
    if level == 0:
        v2 = normals[1, :]
    else:
        v2 = normals[3, :]
    diff = v1 @ v2
    deg = np.rad2deg(np.arccos(diff))
    return deg


def draw_normals(normals, line_length=0.05):
    normal_tips = normals * (1 + line_length)
    num_lines = normals.shape[0]
    all_points = np.vstack((normals, normal_tips))
    lines = [[i, i + num_lines] for i in range(num_lines)]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(all_points),
        lines=o3d.utility.Vector2iVector(lines),
    )

    return line_set


def visualize_refinement(ico, level=2, plot=False):
    vertices, triangles = refine_icosahedron(np.asarray(
        ico.triangles), np.asarray(ico.vertices), level=level)
    new_mesh = create_open_3d_mesh(triangles, vertices)
    # create lineset of normals
    top_normals = np.asarray(new_mesh.triangle_normals)
    # mask = top_normals[:, 2] >= 0.0
    # new_mesh.remove_triangles_by_mask(~mask)
    # top_normals = np.ascontiguousarray(top_normals[top_normals[:, 2] >= 0.0, :])

    line_set = draw_normals(top_normals)
    pcd_normals = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(top_normals))
    pcd_normals.paint_uniform_color([0.5, 0.5, 0.5])
    if plot:
        plot_meshes(new_mesh, line_set, pcd_normals, shift=False)
    return new_mesh, top_normals


def split_triangles(mesh):
    """
    Split the mesh in independent triangles    
    """
    triangles = np.asarray(mesh.triangles).copy()
    vertices = np.asarray(mesh.vertices).copy()

    triangles_3 = np.zeros_like(triangles)
    vertices_3 = np.zeros((len(triangles) * 3, 3), dtype=vertices.dtype)

    for index_triangle, t in enumerate(triangles):
        index_vertex = index_triangle * 3
        vertices_3[index_vertex] = vertices[t[0]]
        vertices_3[index_vertex + 1] = vertices[t[1]]
        vertices_3[index_vertex + 2] = vertices[t[2]]

        triangles_3[index_triangle] = np.arange(index_vertex, index_vertex + 3)

    mesh_return = deepcopy(mesh)
    mesh_return.triangles = o3d.utility.Vector3iVector(triangles_3)
    mesh_return.vertices = o3d.utility.Vector3dVector(vertices_3)
    mesh_return.paint_uniform_color([0.5, 0.5, 0.5])
    return mesh_return


def assign_vertex_colors(mesh, normal_colors):
    """Assigns vertex colors by given normal colors
    NOTE: New mesh is returned

    Arguments:
        mesh {o3d:TriangleMesh} -- Mesh
        normal_colors {ndarray} -- Normals Colors

    Returns:
        o3d:TriangleMesh -- New Mesh with painted colors
    """
    split_mesh = split_triangles(mesh)
    vertex_colors = np.asarray(split_mesh.vertex_colors)
    triangles = np.asarray(split_mesh.triangles)
    for i in range(triangles.shape[0]):
        color = normal_colors[i, :]
        p_idx = triangles[i, :]
        vertex_colors[p_idx] = color

    return split_mesh


def visualize_gaussian_integration(refined_icosahedron_mesh, gaussian_normals, mesh, ds=10, plot=False):
    to_integrate_normals = np.asarray(mesh.triangle_normals)
    num_normals = to_integrate_normals.shape[0]
    to_integrate_normals = to_integrate_normals[np.random.choice(
        num_normals, int(num_normals / ds)), :]
    ga = GaussianAccumulator(gaussian_normals)
    ga.integrate(to_integrate_normals)
    ga.normalize()
    colored_icosahedron = assign_vertex_colors(
        refined_icosahedron_mesh, ga.colors)
    if plot:
        plot_meshes(colored_icosahedron, mesh)

    return ga, colored_icosahedron


def convert_phi_theta(normals, top_half=True):
    phi_theta = np.zeros((normals.shape[0], 2))
    xy = normals[:, 0]**2 + normals[:, 1]**2
    phi_theta[:, 0] = np.arctan2(np.sqrt(xy), normals[:, 2])  # for elevation angle defined from Z-axis down
    phi_theta[:, 1] = np.arctan2(normals[:, 1], normals[:, 0])
    mask = phi_theta[:, 0] < np.pi / 2.0
    phi_theta = phi_theta[mask, :]
    return phi_theta, mask


def convert_stereographic(normals, top_half=True):
    mask = normals[:, 2] > 0
    normals_new = normals[mask, :]
    projection = np.zeros((normals_new.shape[0], 2))
    projection[:, 0] = normals_new[:, 0] / (1 - normals_new[:, 2])
    projection[:, 1] = normals_new[:, 1] / (1 - normals_new[:, 2])
    return projection, mask


def convert_phi_theta_cetered(normals, top_half=True):
    mask = normals[:, 2] > 0
    normals_new = normals[mask, :]
    projection = np.zeros((normals_new.shape[0], 2))
    xy = normals_new[:, 0]**2 + normals_new[:, 1]**2
    for i in range(normals_new.shape[0]):
        normal = normals_new[i, :]
        phi = np.arccos(normal[2])
        phi = -phi if normal[1] < 0 else phi
        theta = np.arctan2(normal[1], normal[0])
        theta = theta + np.pi / 2.0
        theta = theta - np.pi if theta > np.pi / 2.0 else theta
        projection[i, :] = [phi, theta]

    return projection, mask


def down_proj(normals, top_half=True):
    mask = normals[:, 2] > 0
    normals_new = normals[mask, :]
    projection = np.zeros((normals_new.shape[0], 2))
    projection = normals_new[:, :2]

    return projection, mask


def azimuth_equidistant(normals, top_half=True):
    mask = normals[:, 2] > -0.2
    normals_new = normals[mask, :]
    projection = np.zeros((normals_new.shape[0], 2))
    xy = normals_new[:, 0]**2 + normals_new[:, 1]**2
    r_proj = np.sqrt(xy)
    theta = np.arctan2(normals_new[:, 1], normals_new[:, 0])
    phi = np.zeros_like(theta)
    for i in range(normals_new.shape[0]):
        normal = normals_new[i, :]
        phi_ = np.arccos(normal[2])
        # phi_ = -phi_ if normal[1] < 0 else phi_
        phi[i] = phi_

    projection[:, 0] = phi * np.sin(theta)
    projection[:, 1] = - phi * np.cos(theta)

    return projection, mask


def plot_projection(ga):

    projections = [("Spherical Coordinates", "phi", "theta", "convert_phi_theta"),
                   ("Spherical Coordinates Centered", "phi", "theta", "convert_phi_theta_cetered"),
                   ("Steographic Projection", "x*", "y*", "convert_stereographic"),
                   ("Project To Plane", "x", "y", "down_proj"),
                   ("Azimuth Equidistant", "x*", "y*", "azimuth_equidistant"),
                   ]
    fig, axs = plt.subplots(3, 2, figsize=(5, 7))
    axs = axs.reshape(-1)
    for i, (title_name, xlabel, ylabel, function_name) in enumerate(projections):
        ax = axs[i]
        proj, mask = globals()[function_name](ga.gaussian_normals)
        ax.scatter(proj[:, 0], proj[:, 1], c=ga.colors[mask, :])
        ax.set_title(title_name)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.axis('equal')
    fig.tight_layout()

    plt.show()


def main():

    ico, sphere = generate_sphere_examples()
    ico_copy = o3d.geometry.TriangleMesh(ico)
    family = [1, 2, 3, 4, 5, 6]
    meshes = generate_family_of_icosahedron(np.asarray(
        ico.triangles), np.asarray(ico.vertices), family)
    family.insert(0, 0)
    meshes.insert(0, ico)
    for level, mesh in zip(family, meshes):
        angle_diff = calc_angle_delta(mesh, level)
        print("Refinement Level: {}; Number of Triangles: {}, Angle Difference: {:.1f}".format(
            level, np.array(mesh.triangles).shape[0], angle_diff))
    meshes.insert(0, sphere)
    # plot_meshes(*meshes)
    # Show our chosen refined example
    refined_icosphere, gaussian_normals = visualize_refinement(
        ico_copy, level=3, plot=False)
    # Get an Example Mesh
    for i, (mesh_fpath, r) in enumerate(zip(ALL_MESHES, ALL_MESHES_ROTATIONS)):
        if i < 1:
            continue
        example_mesh = o3d.io.read_triangle_mesh(mesh_fpath)
        example_mesh = example_mesh.rotate(r.as_matrix())
        example_mesh.compute_triangle_normals()
        # plot_meshes(example_mesh)

        ga, colored_icosahedron = visualize_gaussian_integration(
            refined_icosphere, gaussian_normals, example_mesh)
        plot_projection(ga)
        plot_meshes(colored_icosahedron, example_mesh)


if __name__ == "__main__":
    main()


# 'filter_smooth_laplacian', 'filter_smooth_simple',
# for smooth_alg in ['filter_smooth_taubin']:
#     new_mesh = o3d.geometry.TriangleMesh(example_mesh)
#     func = getattr(new_mesh, smooth_alg)
#     new_mesh = func()
#     plot_meshes(new_mesh)
# Visualize Guassiant Integration

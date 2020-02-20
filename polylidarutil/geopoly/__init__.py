import open3d as o3d
import numpy as np

from examples.python.realsense_util import create_open_3d_mesh


GOLDEN_RATIO = (1.0 + np.sqrt(5.0)) / 2.0
ICOSAHEDRON_TRUE_RADIUS = np.sqrt(1 + np.power(GOLDEN_RATIO, 2))
ICOSAHEDRON_SCALING = 1.0 / ICOSAHEDRON_TRUE_RADIUS
ICOSAHEDRON_SCALED_EDGE_LENGTH = ICOSAHEDRON_SCALING * 2.0

EXAMPLE_MESH = './tests/fixtures/realsense/example_mesh.ply'


def cantor_mapping(k1, k2):
    return int(((k1 + k2) * (k1 + k2 + 1)) / 2.0 + k2)


def generate_key_from_point(p1_idx, p2_idx):
    lower_idx, higher_idx = (p1_idx, p2_idx) if p1_idx < p2_idx else (p2_idx, p1_idx)
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
    axis.translate([-2, 0, 0])
    translate_meshes = []
    if shift:
        for i, mesh in enumerate(meshes):
            translate_meshes.append(mesh.translate([i * 2.0, 0, 0]))
    else:
        translate_meshes = meshes

    o3d.visualization.draw_geometries([axis, *translate_meshes])


def generate_sphere_examples():
    ico = o3d.geometry.TriangleMesh.create_icosahedron(radius=ICOSAHEDRON_SCALING)
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1, resolution=20)

    ico.compute_vertex_normals()
    ico.compute_triangle_normals()
    sphere.compute_vertex_normals()
    sphere.compute_triangle_normals()

    # all_triangle_vertices = vertices[triangles[:, :], :]
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
        new_vertices, new_triangles = refine_icosahedron(triangles, vertices, level=level)
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


def visualize_refinement(ico, level=2):
    vertices, triangles = refine_icosahedron(np.asarray(ico.triangles), np.asarray(ico.vertices), level=level)
    new_mesh = create_open_3d_mesh(triangles, vertices)
    # create lineset of normals
    top_normals = np.asarray(new_mesh.triangle_normals)
    top_normals = np.ascontiguousarray(top_normals[top_normals[:, 2] >= 0.0, :])

    line_set = draw_normals(top_normals)
    pcd_normals = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(top_normals))
    pcd_normals.paint_uniform_color([0.5, 0.5, 0.5])

    plot_meshes(new_mesh, line_set, pcd_normals, shift=False)
    return new_mesh, top_normals
    

def visualize_gaussian_integration(refined_icosahedron_mesh, gaussian_normals, mesh):
    plot_meshes(refined_icosahedron_mesh, mesh)

    to_integrate_normals = np.asarray(mesh.triangle_normals)

    # print(repr(gaussian_normals))
    # print(repr(to_integrate_normals))

def main():

    ico, sphere = generate_sphere_examples()
    family = [1, 2, 3, 4]
    meshes = generate_family_of_icosahedron(np.asarray(ico.triangles), np.asarray(ico.vertices), family)
    family.insert(0, 0)
    meshes.insert(0, ico)
    for level, mesh in zip(family, meshes):
        angle_diff = calc_angle_delta(mesh, level)
        print("Refinement Level: {}; Number of Triangles: {}, Angle Difference: {:.1f}".format(
            level, np.array(mesh.triangles).shape[0], angle_diff))
    meshes.insert(0, ico)
    # plot_meshes(*meshes)
    refined_icosphere, gaussian_normals = visualize_refinement(ico)
    example_mesh = o3d.io.read_triangle_mesh(EXAMPLE_MESH)
    example_mesh.compute_triangle_normals()

    visualize_gaussian_integration(refined_icosphere, gaussian_normals, example_mesh)


if __name__ == "__main__":
    main()

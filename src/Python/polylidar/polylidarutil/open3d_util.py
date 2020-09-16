import numpy as np
import open3d as o3d

from .line_mesh import LineMesh
from polylidar.polylidarutil import COLOR_PALETTE
from polylidar import MatrixDouble, MatrixInt, create_tri_mesh_copy
EXTRINSICS = None

MAX_POLYS = 10
ORANGE = (255 / 255, 188 / 255, 0)
GREEN = (0, 255 / 255, 0)


def open_3d_mesh_to_trimesh(mesh: o3d.geometry.TriangleMesh):
    triangles = np.asarray(mesh.triangles)
    vertices = np.asarray(mesh.vertices)

    triangles = np.ascontiguousarray(triangles)

    vertices_mat = MatrixDouble(vertices)
    triangles_mat = MatrixInt(triangles)
    triangles_mat_np = np.asarray(triangles_mat)

    tri_mesh = create_tri_mesh_copy(vertices_mat, triangles_mat)
    return tri_mesh

def create_open_3d_mesh_from_tri_mesh(tri_mesh):
    """Create an Open3D Mesh given a Polylidar TriMesh"""
    triangles = np.asarray(tri_mesh.triangles)
    vertices = np.asarray(tri_mesh.vertices)
    triangle_normals = np.asarray(tri_mesh.triangle_normals)
    return create_open_3d_mesh(triangles, vertices, triangle_normals, counter_clock_wise=tri_mesh.counter_clock_wise)


def create_open_3d_mesh(triangles, points, triangle_normals=None, color=COLOR_PALETTE[0], counter_clock_wise=True):
    """Create an Open3D Mesh given triangles vertices

    Arguments:
        triangles {ndarray} -- Triangles array
        points {ndarray} -- Points array

    Keyword Arguments:
        color {list} -- RGB COlor (default: {[1, 0, 0]})

    Returns:
        mesh -- Open3D Mesh
    """
    mesh_o3d = o3d.geometry.TriangleMesh()
    if points.ndim == 1:
        points = points.reshape((int(points.shape[0] / 3), 3))
    if triangles.ndim == 1:
        triangles = triangles.reshape((int(triangles.shape[0] / 3), 3))
        # Open 3D expects triangles to be counter clockwise
    if not counter_clock_wise:
        triangles = np.ascontiguousarray(np.flip(triangles, 1))
    mesh_o3d.triangles = o3d.utility.Vector3iVector(triangles)

    mask = np.isnan(points).any(axis=1) # I think that we need this with open3d 0.10.0
    points[mask, :] = [0,0,0]
    mesh_o3d.vertices = o3d.utility.Vector3dVector(points)
    if triangle_normals is None:
        mesh_o3d.compute_vertex_normals()
        mesh_o3d.compute_triangle_normals()
    elif triangle_normals.ndim == 1:
        triangle_normals_ = triangle_normals.reshape((int(triangle_normals.shape[0] / 3), 3))
        mesh_o3d.triangle_normals = o3d.utility.Vector3dVector(triangle_normals_)
    else:
        mesh_o3d.triangle_normals = o3d.utility.Vector3dVector(triangle_normals)
    mesh_o3d.paint_uniform_color(color)
    mesh_o3d.compute_vertex_normals()
    return mesh_o3d

def flatten(l): return [item for sublist in l for item in sublist]


def update_points(pcd, pc):
    pcd.points = o3d.utility.Vector3dVector(pc)


def set_line(line_set, points, lines, colors):
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)


def construct_grid(size=10, n=10, color=[0.5, 0.5, 0.5], plane='xy', plane_offset=-1, translate=[0, 0, 0]):
    grid_ls = o3d.geometry.LineSet()
    my_grid = make_grid(size=size, n=n, color=color, plane=plane, plane_offset=plane_offset, translate=translate)
    set_line(grid_ls, *my_grid)
    return grid_ls


def make_grid(size=10, n=10, color=[0.5, 0.5, 0.5], plane='xy', plane_offset=-1, translate=[0, 0, 0]):
    """draw a grid as a line set"""

    # lineset = o3d.geometry.LineSet()
    s = size / float(n)
    s2 = 0.5 * size
    points = []

    for i in range(0, n + 1):
        x = -s2 + i * s
        points.append([x, -s2, plane_offset])
        points.append([x, s2, plane_offset])
    for i in range(0, n + 1):
        z = -s2 + i * s
        points.append([-s2, z, plane_offset])
        points.append([s2, z, plane_offset])

    points = np.array(points)
    if plane == 'xz':
        points[:, [2, 1]] = points[:, [1, 2]]

    points = points + translate

    n_points = points.shape[0]
    lines = [[i, i + 1] for i in range(0, n_points - 1, 2)]
    colors = [list(color)] * (n_points - 1)
    return points, lines, colors


def clear_polys(all_polys, vis):
    for line_mesh in all_polys:
        line_mesh.remove_line(vis)
    return []


def handle_shapes(vis, planes, obstacles, all_polys, line_radius=0.15):
    all_polys = clear_polys(all_polys, vis)
    for plane, _ in planes:
        points = np.array(plane.exterior)
        line_mesh = LineMesh(points, colors=GREEN, radius=line_radius)
        line_mesh.add_line(vis)
        all_polys.append(line_mesh)

    for plane, _ in obstacles:
        points = np.array(plane.exterior)
        line_mesh = LineMesh(points, colors=ORANGE, radius=line_radius)
        line_mesh.add_line(vis)
        all_polys.append(line_mesh)

    return all_polys


def create_lines(planes, obstacles, line_radius=0.15, rotate_func=None):
    all_polys = []
    for plane, _ in planes:
        points = np.array(plane.exterior)
        if rotate_func:
            points = rotate_func(points)
        line_mesh = LineMesh(points, colors=GREEN, radius=line_radius)
        all_polys.append(line_mesh)

    for plane, _ in obstacles:
        points = np.array(plane.exterior)
        if rotate_func:
            points = rotate_func(points)
        line_mesh = LineMesh(points, colors=ORANGE, radius=line_radius)
        all_polys.append(line_mesh)

    return all_polys


def get_extrinsics(vis):
    ctr = vis.get_view_control()
    camera_params = ctr.convert_to_pinhole_camera_parameters()
    return camera_params.extrinsic


def set_initial_view(vis, extrinsics=[EXTRINSICS]):
    ctr = vis.get_view_control()
    camera_params = ctr.convert_to_pinhole_camera_parameters()
    camera_params.extrinsic = extrinsics
    ctr.convert_from_pinhole_camera_parameters(camera_params)

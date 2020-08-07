import time
import logging

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors

# from polylidar_plane_benchmark.utility.o3d_util import create_open_3d_mesh_from_tri_mesh

import organizedpointfilters as opf

from organizedpointfilters import Matrix3f, Matrix3fRef

from polylidar import extract_tri_mesh_from_organized_point_cloud, MatrixDouble

COLOR_PALETTE = list(map(colors.to_rgb, plt.rcParams['axes.prop_cycle'].by_key()['color']))


# def vector_magnitude(vec):
#     """
#     Calculates a vector's magnitude.
#     Args:
#         - vec ():
#     """
#     magnitude = np.sqrt(np.sum(vec**2))
#     return(magnitude)


# def align_vector_to_another(a=np.array([0, 0, 1]), b=np.array([1, 0, 0])):
#     """
#     Aligns vector a to vector b with axis angle rotation
#     """
#     if np.array_equal(a, b):
#         return None, None
#     axis_ = np.cross(a, b)
#     axis_ = axis_ / np.linalg.norm(axis_)
#     angle = np.arccos(np.dot(a, b))

#     return axis_, angle


# def create_arrow(scale=1, cylinder_radius=None, **kwargs):
#     """
#     Create an arrow in for Open3D
#     """
#     cone_height = scale * 0.2
#     cylinder_height = scale * 0.8
#     cone_radius = cylinder_radius if cylinder_radius else scale / 10
#     cylinder_radius = cylinder_radius if cylinder_radius else scale / 20
#     mesh_frame = o3d.geometry.TriangleMesh.create_arrow(cone_radius=cone_radius,
#                                                         cone_height=cone_height,
#                                                         cylinder_radius=cylinder_radius,
#                                                         cylinder_height=cylinder_height)
#     return(mesh_frame)


# def get_arrow(origin=[0, 0, 0], end=None, vec=None, **kwargs):
#     """
#     Creates an arrow from an origin point to an end point,
#     or create an arrow from a vector vec starting from origin.
#     Args:
#         - end (): End point. [x,y,z]
#         - vec (): Vector. [i,j,k]
#     """
#     # print(end)
#     scale = 10
#     beta = 0
#     gamma = 0
#     T = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
#     T[:3, -1] = origin
#     if end is not None:
#         vec = np.array(end) - np.array(origin)
#     elif vec is not None:
#         vec = np.array(vec)
#     if end is not None or vec is not None:
#         scale = vector_magnitude(vec)
#         mesh = create_arrow(scale, **kwargs)
#         axis, angle = align_vector_to_another(b=vec / scale)
#         if axis is None:
#             axis_a = axis
#         else:
#             axis_a = axis * angle
#             rotation_3x3 = mesh.get_rotation_matrix_from_axis_angle(axis_a)
#     # mesh.transform(T)
#     if axis is not None:
#         mesh = mesh.rotate(rotation_3x3, center=False)
#     mesh.translate(origin)
#     return(mesh)

def create_open_3d_mesh_from_tri_mesh(tri_mesh):
    """Create an Open3D Mesh given a Polylidar TriMesh"""
    triangles = np.asarray(tri_mesh.triangles)
    vertices = np.asarray(tri_mesh.vertices)
    triangle_normals = np.asarray(tri_mesh.triangle_normals)
    return create_open_3d_mesh(triangles, vertices, triangle_normals)


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
    import open3d as o3d
    mesh_2d = o3d.geometry.TriangleMesh()
    if points.ndim == 1:
        points = points.reshape((int(points.shape[0] / 3), 3))
    if triangles.ndim == 1:
        triangles = triangles.reshape((int(triangles.shape[0] / 3), 3))
        # Open 3D expects triangles to be counter clockwise
    if not counter_clock_wise:
        triangles = np.ascontiguousarray(np.flip(triangles, 1))
    mesh_2d.triangles = o3d.utility.Vector3iVector(triangles)
    mesh_2d.vertices = o3d.utility.Vector3dVector(points)
    if triangle_normals is None:
        mesh_2d.compute_vertex_normals()
        mesh_2d.compute_triangle_normals()
    elif triangle_normals.ndim == 1:
        triangle_normals_ = triangle_normals.reshape((int(triangle_normals.shape[0] / 3), 3))
        mesh_2d.triangle_normals = o3d.utility.Vector3dVector(triangle_normals_)
    else:
        mesh_2d.triangle_normals = o3d.utility.Vector3dVector(triangle_normals)
    mesh_2d.paint_uniform_color(color)
    mesh_2d.compute_vertex_normals()
    return mesh_2d


def laplacian_opc(opc, loops=5, _lambda=0.5, kernel_size=3, **kwargs):
    """Performs Laplacian Smoothing on an organized point cloud

    Arguments:
        opc {ndarray} -- Organized Point Cloud MXNX3, Assumed F64

    Keyword Arguments:
        loops {int} -- How many iterations of smoothing (default: {5})
        _lambda {float} -- Weight factor for update (default: {0.5})
        kernel_size {int} -- Kernel Size (How many neighbors to intregrate) (default: {3})

    Returns:
        ndarray -- Smoothed Point Cloud, MXNX3, F64
    """
    opc_float = (np.ascontiguousarray(opc[:, :, :3])).astype(np.float32)

    a_ref = Matrix3fRef(opc_float)

    t1 = time.perf_counter()
    if kernel_size == 3:
        b_cp = opf.filter.laplacian_K3(a_ref, _lambda=_lambda, iterations=loops, **kwargs)
    else:
        b_cp = opf.filter.laplacian_K5(a_ref, _lambda=_lambda, iterations=loops, **kwargs)
    t2 = time.perf_counter()
    logging.debug("OPC Mesh Smoothing Took (ms): %.2f", (t2 - t1) * 1000)

    opc_float_out = np.asarray(b_cp)
    opc_out = opc_float_out.astype(np.float64)

    time_elapsed = (t2 - t1) * 1000

    return opc_out, time_elapsed


def laplacian_opc_cuda(opc, loops=5, _lambda=0.5, kernel_size=3, **kwargs):
    """Performs Laplacian Smoothing on an organized point cloud

    Arguments:
        opc {ndarray} -- Organized Point Cloud MXNX3, Assumed F64

    Keyword Arguments:
        loops {int} -- How many iterations of smoothing (default: {5})
        _lambda {float} -- Weight factor for update (default: {0.5})
        kernel_size {int} -- Kernel Size (How many neighbors to intregrate) (default: {3})

    Returns:
        ndarray -- Smoothed Point Cloud, MXNX3
    """
    import organizedpointfilters.cuda as opf_cuda
    opc_float = (np.ascontiguousarray(opc[:, :, :3])).astype(np.float32)

    t1 = time.perf_counter()
    if kernel_size == 3:
        opc_float_out = opf_cuda.kernel.laplacian_K3_cuda(opc_float, loops=loops, _lambda=_lambda, **kwargs)
    else:
        opc_float_out = opf_cuda.kernel.laplacian_K5_cuda(opc_float, loops=loops, _lambda=_lambda, **kwargs)
    t2 = time.perf_counter()

    logging.debug("OPC CUDA Laplacian Mesh Smoothing Took (ms): %.2f", (t2 - t1) * 1000)

    # only for visualization purposes here
    opc_out = opc_float_out.astype(np.float64)
    time_elapsed = (t2 - t1) * 1000

    return opc_out, time_elapsed

def compute_normals_and_centroids_opc(opc, convert_f64=True, **kwargs):
    """Computes the Normals and Centroid of Implicit Triangle Mesh in the Organized Point Cloud
    
    Arguments:
        opc {ndarray} -- MXNX3
    
    Keyword Arguments:
        convert_f64 {bool} -- Return F64? (default: {True})
    
    Returns:
        ndarray -- Numpy array
    """
    opc_float = (np.ascontiguousarray(opc[:, :, :3])).astype(np.float32)

    a_ref = Matrix3fRef(opc_float)

    t1 = time.perf_counter()
    normals, centroids = opf.filter.compute_normals_and_centroids(a_ref)
    t2 = time.perf_counter()
    logging.debug("OPC Compute Normals and Centroids Took (ms): %.2f", (t2 - t1) * 1000)
    normals_float_out = np.asarray(normals)
    centroids_float_out = np.asarray(centroids)

    if not convert_f64:
        return (normals_float_out, centroids_float_out)

    return (normals_float_out.astype(np.float64), centroids_float_out.astype(np.float64))

def bilateral_opc(opc, loops=5, sigma_length=0.1, sigma_angle=0.261, **kwargs):
    """Performs bilateral normal smoothing on a mesh implicit from an organized point

    Arguments:
        opc {ndarray} -- Organized Point Cloud MXNX3, Assumed Float 64

    Keyword Arguments:
        loops {int} -- How many iterations of smoothing (default: {5})
        sigma_length {float} -- Saling factor for length (default: {0.1})
        sigma_angle {float} -- Scaling factor for angle (default: {0.261})

    Returns:
        ndarray -- MX3 Triangle Normal Array, Float 64
    """
    opc_float = (np.ascontiguousarray(opc[:, :, :3])).astype(np.float32)

    a_ref = Matrix3fRef(opc_float)

    t1 = time.perf_counter()
    normals = opf.filter.bilateral_K3(a_ref, iterations=loops, sigma_length=sigma_length, sigma_angle=sigma_angle)
    t2 = time.perf_counter()
    logging.debug("OPC Bilateral Filter Took (ms): %.2f", (t2 - t1) * 1000)
    normals_float_out = np.asarray(normals)
    normals_out = normals_float_out.astype(np.float64)

    time_elapsed = (t2-t1) * 1000

    return normals_out, time_elapsed

def bilateral_opc_cuda(opc, loops=5, sigma_length=0.1, sigma_angle=0.261, **kwargs):
    """Performs bilateral normal smoothing on a mesh implicit from an organized point

    Arguments:
        opc {ndarray} -- Organized Point Cloud MXNX3, Assumed Float 64

    Keyword Arguments:
        loops {int} -- How many iterations of smoothing (default: {5})
        sigma_length {float} -- Saling factor for length (default: {0.1})
        sigma_angle {float} -- Scaling factor for angle (default: {0.261})

    Returns:
        ndarray -- MX3 Triangle Normal Array, Float 64
    """
    import organizedpointfilters.cuda as opf_cuda
    normals_opc, centroids_opc = compute_normals_and_centroids_opc(opc, convert_f64=False)
    assert normals_opc.dtype == np.float32
    assert centroids_opc.dtype == np.float32

    t1 = time.perf_counter()
    normals_float_out = opf_cuda.kernel.bilateral_K3_cuda(
        normals_opc, centroids_opc, loops=loops, sigma_length=sigma_length, sigma_angle=sigma_angle)
    t2 = time.perf_counter()

    normals_out = normals_float_out.astype(np.float64)

    time_elapsed = (t2-t1) * 1000

    logging.debug("OPC CUDA Bilateral Filter Took (ms): %.2f", (t2 - t1) * 1000)

    return normals_out, time_elapsed


def laplacian_then_bilateral_opc(opc, loops_laplacian=5, _lambda=1.0, kernel_size=3, loops_bilateral=0, sigma_length=0.1, sigma_angle=0.261, **kwargs):
    """Performs Laplacian Smoothing on Point Cloud and then performs Bilateral normal smoothing

    Arguments:
        opc {ndarray} -- Organized Point Cloud (MXNX3)

    Keyword Arguments:
        loops_laplacian {int} -- How many iterations of laplacian smoothing (default: {5})
        _lambda {float} -- Weigh factor for laplacian  (default: {0.5})
        kernel_size {int} -- Kernel Size for Laplacian (default: {3})
        loops_bilateral {int} -- How many iterations of bilateral smoothing (default: {0})
        sigma_length {float} -- Scaling factor for length bilateral (default: {0.1})
        sigma_angle {float} -- Scaling factor for angle bilateral (default: {0.261})

    Returns:
        tuple(ndarray, ndarray) -- Smoothed OPC MXNX3, Smoothed Normals M*NX3, normals arrays are flattened
    """
    opc_float = (np.ascontiguousarray(opc[:, :, :3])).astype(np.float32)

    a_ref = Matrix3fRef(opc_float)

    t1 = time.perf_counter()
    if kernel_size == 3:
        b_cp = opf.filter.laplacian_K3(a_ref, _lambda=_lambda, iterations=loops_laplacian, **kwargs)
    else:
        b_cp = opf.filter.laplacian_K5(a_ref, _lambda=_lambda, iterations=loops_laplacian, **kwargs)
    t2 = time.perf_counter()
    logging.debug("OPC Mesh Smoothing Took (ms): %.2f", (t2 - t1) * 1000)

    opc_float_out = np.asarray(b_cp)
    b_ref = Matrix3fRef(opc_float_out)

    opc_normals_float = opf.filter.bilateral_K3(
        b_ref, iterations=loops_bilateral, sigma_length=sigma_length, sigma_angle=sigma_angle)
    t3 = time.perf_counter()
    logging.debug("OPC Bilateral Normal Filter Took (ms): %.2f", (t3 - t2) * 1000)

    opc_normals_float_out = np.asarray(opc_normals_float)
    total_triangles = int(opc_normals_float_out.size / 3)
    opc_normals_out = opc_normals_float_out.astype(np.float64)
    opc_normals_out = opc_normals_float_out.reshape((total_triangles, 3))

    opc_out = opc_float_out.astype(np.float64)
    # total_points = int(opc_out.size / 3)
    # opc_out = opc_out.reshape((total_points, 3))

    timings = dict(t_laplacian=(t2-t1)*1000, t_bilateral=(t3-t2)*1000)

    return opc_out, opc_normals_out, timings

def laplacian_then_bilateral_opc_cuda(opc, loops_laplacian=5, _lambda=1.0, kernel_size=3, loops_bilateral=0, sigma_length=0.1, sigma_angle=0.261, **kwargs):
    """Performs Laplacian Smoothing on Point Cloud and then performs Bilateral normal smoothing

    Arguments:
        opc {ndarray} -- Organized Point Cloud (MXNX3)

    Keyword Arguments:
        loops_laplacian {int} -- How many iterations of laplacian smoothing (default: {5})
        _lambda {float} -- Weigh factor for laplacian  (default: {0.5})
        kernel_size {int} -- Kernel Size for Laplacian (default: {3})
        loops_bilateral {int} -- How many iterations of bilateral smoothing (default: {0})
        sigma_length {float} -- Scaling factor for length bilateral (default: {0.1})
        sigma_angle {float} -- Scaling factor for angle bilateral (default: {0.261})

    Returns:
        tuple(ndarray, ndarray) -- Smoothed OPC MXNX3, Smoothed Normals M*NX3, the arrays are flattened
    """
    import organizedpointfilters.cuda as opf_cuda
    opc_float = (np.ascontiguousarray(opc[:, :, :3])).astype(np.float32)

    t1 = time.perf_counter()
    if kernel_size == 3:
        opc_float_out = opf_cuda.kernel.laplacian_K3_cuda(opc_float, loops=loops_laplacian, _lambda=_lambda)
    else:
        opc_float_out = opf_cuda.kernel.laplacian_K5_cuda(opc_float, loops=loops_laplacian, _lambda=_lambda)
    t2 = time.perf_counter()

    opc_normals, time_bilateral = bilateral_opc_cuda(opc_float_out, loops=loops_bilateral,
                                     sigma_length=sigma_length, sigma_angle=sigma_angle)

    opc_out = opc_float_out.astype(np.float64)
    # total_points = int(opc_out.size / 3)
    # opc_out = opc_out.reshape((total_points, 3))

    total_triangles = int(opc_normals.size / 3)
    opc_normals = opc_normals.reshape((total_triangles, 3))

    timings = dict(t_laplacian=(t2-t1)*1000, t_bilateral=time_bilateral)

    return opc_out, opc_normals, timings

def create_mesh_from_organized_point_cloud(pcd, rows=500, cols=500, stride=2, calc_normals=True):
    """Create Mesh from organized point cloud
    If an MXNX3 Point Cloud is passed, rows and cols is ignored (we know the row/col from shape)
    If an KX3 Point Cloud is passed, you must pass the row, cols, and stride that correspond to the point cloud

    Arguments:
        pcd {ndarray} -- Numpy array. Either a K X 3 (flattened) or MXNX3

    Keyword Arguments:
        rows {int} -- Number of rows (default: {500})
        cols {int} -- Number of columns (default: {500})
        stride {int} -- Stride used in creating point cloud (default: {2})

    Returns:
        tuple -- Polylidar Tri Mesh and O3D mesh
    """
    pcd_ = pcd
    if pcd.ndim == 3:
        rows = pcd.shape[0]
        cols = pcd.shape[1]
        stride = 1
        pcd_ = pcd.reshape((rows * cols, 3))

    pcd_mat = MatrixDouble(pcd_, copy=True)
    t1 = time.perf_counter()
    tri_mesh, tri_map = extract_tri_mesh_from_organized_point_cloud(pcd_mat, rows, cols, stride, calc_normals=calc_normals)
    t2 = time.perf_counter()
    time_elapsed = (t2 - t1) * 1000
    return tri_mesh, tri_map, time_elapsed

def plot_triangle_normals(normals:np.ndarray, normals2:np.ndarray):
    f, (ax1, ax2) = plt.subplots(1,2) 
    colors = ((normals * 0.5 + 0.5) * 255).astype(np.uint8)
    im = colors.reshape((249, 249, 2, 3))
    im = im[:, :, 1, :]

    colors2 = ((normals2 * 0.5 + 0.5) * 255).astype(np.uint8)
    im2= colors2.reshape((249, 249, 2, 3))
    im2 = im2[:, :, 1, :]

    ax1.imshow(im, origin='upper')
    ax2.imshow(im2, origin='upper')
    plt.show()


def pick_valid_normals(tri_map, opc_normals):
    tri_map_np = np.asarray(tri_map)
    mask = tri_map_np != np.iinfo(tri_map_np.dtype).max
    tri_norms = np.ascontiguousarray(opc_normals[mask,:])
    return tri_norms


def create_meshes(opc, **kwargs):
    """Creates a mesh from a noisy organized point cloud

    Arguments:
        opc {ndarray} -- Must be MXNX3

    Keyword Arguments:
        loops {int} -- How many loop iterations (default: {5})
        _lambda {float} -- weighted iteration movement (default: {0.5})

    Returns:
        [tuple(mesh, o3d_mesh)] -- polylidar mesh and o3d mesh reperesentation
    """
    smooth_opc, opc_normals, timings = laplacian_then_bilateral_opc(opc, **kwargs)
    tri_mesh, tri_map, time_elapsed_mesh = create_mesh_from_organized_point_cloud(smooth_opc, calc_normals=False)
    tri_norms = pick_valid_normals(tri_map, opc_normals)
    opc_normals_cp = MatrixDouble(tri_norms, copy=True) # copy here!!!!!
    # plot_triangle_normals(np.asarray(tri_mesh.triangle_normals), opc_normals)
    tri_mesh.set_triangle_normals(opc_normals_cp) # copy again here....sad
    timings = dict(**timings, t_mesh=time_elapsed_mesh)
    return tri_mesh, timings

def create_meshes_cuda(opc, **kwargs):
    """Creates a mesh from a noisy organized point cloud

    Arguments:
        opc {ndarray} -- Must be MXNX3

    Keyword Arguments:
        loops {int} -- How many loop iterations (default: {5})
        _lambda {float} -- weighted iteration movement (default: {0.5})

    Returns:
        [tuple(mesh, o3d_mesh)] -- polylidar mesh and o3d mesh reperesentation
    """
    smooth_opc, opc_normals, timings = laplacian_then_bilateral_opc_cuda(opc, **kwargs)
    tri_mesh, tri_map, time_elapsed_mesh = create_mesh_from_organized_point_cloud(smooth_opc, calc_normals=False)
    tri_norms = pick_valid_normals(tri_map, opc_normals)
    opc_normals_cp = MatrixDouble(tri_norms, copy=True) # copy here!!!!!
    # plot_triangle_normals(np.asarray(tri_mesh.triangle_normals), opc_normals)
    tri_mesh.set_triangle_normals(opc_normals_cp) # copy again here....sad
    timings = dict(**timings, t_mesh=time_elapsed_mesh)
    return tri_mesh, timings

def create_meshes_cuda_with_o3d(opc, **kwargs):
    """Creates a mesh from a noisy organized point cloud

    Arguments:
        opc {ndarray} -- Must be MXNX3

    Keyword Arguments:
        loops {int} -- How many loop iterations (default: {5})
        _lambda {float} -- weighted iteration movement (default: {0.5})

    Returns:
        [tuple(mesh, o3d_mesh)] -- polylidar mesh and o3d mesh reperesentation
    """
    tri_mesh, timings = create_meshes_cuda(opc, **kwargs)
    tri_mesh_o3d = create_open_3d_mesh_from_tri_mesh(tri_mesh)

    return tri_mesh, tri_mesh_o3d, timings
    


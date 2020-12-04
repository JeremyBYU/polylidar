import time

import numpy as np
from scipy import spatial
from scipy.spatial.transform import Rotation as R
from shapely.geometry import Polygon, JOIN_STYLE
import logging

IDENTITY = R.identity()
logging.basicConfig(level=logging.INFO)

import matplotlib.pyplot as plt


def plot_indices_text(indices, points, ax):
    for i in range(indices.shape[0]):
        ax.text(points[i, 0], points[i, 1], str(indices[i]))


def plot_poly(polygon: Polygon, ax, polygon_indices=None):
    from shapely.geometry import Polygon
    from descartes import PolygonPatch
    outlinePatch = PolygonPatch(Polygon(polygon.exterior), ec='green', fill=False, linewidth=2)
    for i, hole in enumerate(polygon.interiors):
        color = plt.cm.get_cmap('tab20')([i])[0][:3]
        pl = Polygon(shell=hole)
        pp = PolygonPatch(pl, fill=False, linewidth=2, ec=color)
        ax.add_patch(pp)
    if polygon_indices:
        lr_indices = np.asarray(polygon_indices.shell)
        lr_points = np.array(polygon.exterior)
        plot_indices_text(lr_indices, lr_points, ax)
        for i, hole in enumerate(polygon.interiors):
            lr_indices = np.asarray(polygon_indices.holes[i])
            lr_points = np.asarray(hole)
            plot_indices_text(lr_indices, lr_points, ax)

    ax.add_patch(outlinePatch)


def get_points(point_idxs, points):
    return points[point_idxs, :]


def create_kd_tree(shell_coords, hole_coords):
    hole_coords.append(shell_coords)
    all_vertices = np.vstack(hole_coords)
    logging.debug("KD Tree Size: %s", all_vertices.shape)
    kd_tree = spatial.cKDTree(all_vertices, leafsize=10)
    return kd_tree


def add_column(array, z_value):
    ones = np.ones((array.shape[0], 1)) * z_value
    stacked = np.column_stack((array, ones))
    return stacked


def recover_3d(poly, kd_tree, z_value):
    points = np.asarray(poly.exterior)
    if points.shape[1] == 3:
        return poly
    shell_3D = add_column(points, z_value)
    # print(shell_3D.shape)
    t0 = time.perf_counter()
    d, shell_idx = kd_tree.query(shell_3D)
    t1 = time.perf_counter()
    # print(shell_idx.shape)
    kd_data = kd_tree.data[shell_idx, :]
    # print(kd_data.shape)
    shell_3D[:, 2] = kd_data[:, 2]
    holes_lr = []
    t2 = time.perf_counter()
    for hole in poly.interiors:
        hole_lr = add_column(np.array(hole), z_value)
        d, shell_idx = kd_tree.query(hole_lr)
        kd_data = kd_tree.data[shell_idx, :]
        hole_lr[:, 2] = kd_data[:, 2]
        holes_lr.append(hole_lr)
    t3 = time.perf_counter()

    # print("Shell Length: {}; Query KD Tree Shell: {:.2f}; Query KD Tree Holes: {:.2f}".format(
    #     shell_3D.shape[0], (t1-t0) * 1000, (t3-t2) * 1000
    # ))

    poly_3d = Polygon(shell=shell_3D, holes=holes_lr)
    return poly_3d


def filter_planes_and_holes(polygons, points, config_pp, rm=None):
    """Extracts the plane and obstacles returned from polylidar.
    This function performs post-processing of the Polygons returned by Polylidar3D using the Shapely library.
    If the polygons are 3D you must provide a scipy rotation matrix such that the polygon
    align with they XY plane (Shapely can only handle 2D polygons with XY coordinates).

    The basic steps are:
        * Simplification of Polygon by config_pp['simplify]
        * Positive buffer of Polygon by config_pp['positive_buffer]
        * Negative Buffer of POlygons by config_pp['negative_buffer]
        * Simplification of Polygon by config_pp['simplify]
        * Remove polygons whose area is less or greater than data in config_pp['filter]['plane_area']
        * Remove holes whose vertices are less than data in config_pp['filter]['hole_vertices']
        * Remove holes whose area is less or greater than data in config_pp['filter]['hole_area']

    It then returns the shapely polygons of the polygons and holes (obstacles)

    An example config_pp

    .. code-block:: python

        {
            positive_buffer: 0.005 # m, Positively expand polygon.  Fills in small holes
            negative_buffer: 0.03 # m, Negative buffer to polygon. Expands holes and constricts outer hull of polygon
            simplify: 0.02  # m, simplify edges of polygon
            filter: # obstacles must have these characteristics
                hole_area:
                    min: 0.025   # m^2
                    max: 0.785 # m^2
                hole_vertices:
                    min: 6
                plane_area:
                    min: .5 # m^2
        }

    Arguments:
        polygons {list[Polygons]} -- A list of polygons returned from polylidar
        points {ndarray} -- MX3 array
        config_pp {dict} -- Configuration for post processing filtering
        rm {scipy.spatial.RotationMatrix} -- Rotation matrix applied to 3D polygons to make 2D

    Returns:
        tuple -- A list of plane shapely polygons and a list of holes in polygons
    """
    # filtering configuration
    post_filter = config_pp['filter']

    # will hold the plane(s) and obstacles found
    planes = []
    obstacles = []
    # print("Polylidar returned {} polygons, ".format(len(polygons)))
    for poly in polygons:
        t0 = time.perf_counter()
        if rm is not None:
            shell_coords = rm.apply(get_points(poly.shell, points))
            hole_coords = [rm.apply(get_points(hole, points)) for hole in poly.holes]
        else:
            shell_coords = get_points(poly.shell, points)
            hole_coords = [get_points(hole, points) for hole in poly.holes]
        t1 = time.perf_counter()
        poly_shape = Polygon(shell=shell_coords, holes=hole_coords)
        t2 = time.perf_counter()
        # print(poly_shape.is_valid)
        # fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
        # plot_poly(poly_shape, ax, poly)
        # plt.axis('equal')
        # plt.show()
        # print(poly_shape.is_valid)
        # assert poly_shape.is_valid
        area = poly_shape.area
        # logging.info("Got a plane!")
        if post_filter['plane_area']['min'] and area < post_filter['plane_area']['min']:
            # logging.info("Skipping Plane")
            continue
        z_value = shell_coords[0][2]

        t3 = time.perf_counter()
        if config_pp['simplify']:
            poly_shape = poly_shape.simplify(
                tolerance=config_pp['simplify'], preserve_topology=True)
        t4 = time.perf_counter()
        # Perform 2D geometric operations
        if config_pp['positive_buffer']:
            poly_shape = poly_shape.buffer(
                config_pp['positive_buffer'], join_style=JOIN_STYLE.mitre, resolution=4)
        t5 = time.perf_counter()
        if config_pp['negative_buffer']:
            poly_shape = poly_shape.buffer(
                distance=-config_pp['negative_buffer'], join_style=JOIN_STYLE.mitre, resolution=4)
            # if poly_shape.geom_type == 'MultiPolygon':
            #     all_poly_shapes = list(poly_shape.geoms)
            #     poly_shape = sorted(
            #         all_poly_shapes, key=lambda geom: geom.area, reverse=True)[0]
        t6 = time.perf_counter()
        # poly_shape = poly_shape.buffer(distance=config_pp['negative_buffer'], resolution=4)
        if config_pp['simplify']:
            poly_shape = poly_shape.simplify(
                tolerance=config_pp['simplify'], preserve_topology=True)  # False makes fast, but can cause invalid polygons
        t7 = time.perf_counter()
        if poly_shape.geom_type == 'MultiPolygon':
            all_poly_shapes = list(poly_shape.geoms)
            # poly_shape = sorted(
            #     all_poly_shapes, key=lambda geom: geom.area, reverse=True)[0]
        else:
            all_poly_shapes = [poly_shape]

        logging.debug("Rotation: {:.2f}; Polygon Creation: {:.2f}; Simplify 1: {:.2f}; Positive Buffer: {:.2f}; Negative Buffer: {:.2f}; Simplify 2: {:.2f}".format(
            (t1 - t0) * 1000, (t2 - t1) * 1000, (t4 - t3) * 1000, (t5 - t4) * 1000, (t6 - t5) * 1000, (t7 - t6) * 1000
        ))

        # Its possible that our polygon has no broken into a multipolygon
        # Check for this situation and handle it
        # all_poly_shapes = [poly_shape]
        # print(len(all_poly_shapes))
        # iterate through every polygons and check for plane extraction
        for poly_shape in all_poly_shapes:
            area = poly_shape.area
            # print(poly_shape.geom_type, area)
            # logging.info("Plane is big enough still")
            if post_filter['plane_area']['min'] <= 0 or area >= post_filter['plane_area']['min']:
                dim = np.asarray(poly_shape.exterior).shape[1]
                # logging.info("Plane is big enough still")
                if config_pp['negative_buffer'] or config_pp['simplify'] or config_pp['positive_buffer'] and dim < 3:
                    # convert back to 3D coordinates
                    # create kd tree for vertex lookup after buffering operations
                    t8 = time.perf_counter()
                    kd_tree = create_kd_tree(shell_coords, hole_coords)
                    t9 = time.perf_counter()
                    poly_shape = recover_3d(poly_shape, kd_tree, z_value)
                    t10 = time.perf_counter()
                    logging.debug("Create KD Tree: {:.2f}; Recover Polygon 3D Coordinates: {:.2f}".format(
                        (t9 - t8) * 1000, (t10 - t9) * 1000
                    ))

                # Capture the polygon as well as its z height
                # after applying buffering and simplification with shapely/geos all polygons are valid
                # print(poly_shape.is_valid)
                new_plane_polygon = Polygon(shell=poly_shape.exterior)
                planes.append((new_plane_polygon, z_value))

                for hole_lr in poly_shape.interiors:
                    # Filter by number of obstacle vertices, removes noisy holes
                    if len(hole_lr.coords) > post_filter['hole_vertices']['min']:
                        hole_poly = Polygon(shell=hole_lr)
                        area = hole_poly.area
                        # filter by area
                        if post_filter['hole_area']['min'] <= 0.0 or area >= post_filter['hole_area']['min'] and area < post_filter['hole_area']['max']:
                            z_value = hole_lr.coords[0][2]
                            obstacles.append((hole_poly, z_value))
    if rm is not None:
        t11 = time.perf_counter()
        rm_inv = rm.inv()
        for i, (poly, z_value) in enumerate(planes):
            points = np.asarray(poly.exterior)
            new_poly = Polygon(rm_inv.apply(points))
            planes[i] = (new_poly, z_value)

        for i, (poly, z_value) in enumerate(obstacles):
            points = np.asarray(poly.exterior)
            new_poly = Polygon(rm_inv.apply(points))
            obstacles[i] = (new_poly, z_value)
        t12 = time.perf_counter()
        logging.debug("Revert Rotation and Create New Polygons: {:2f}".format((t12 - t11) * 1000))
    return planes, obstacles


def filter_planes(polygons, points, config_pp, rm=None):
    """Converts polygons from Polylidar to Shapely polygons.
    This function performs post-processing of the Polygons returned by Polylidar3D using the Shapely library.
    If the polygons are 3D you must provide a scipy rotation matrix such that the polygon
    align with they XY plane (Shapely can only handle 2D polygons with XY coordinates).

    The basic steps are:
        * Simplification of Polygon by config_pp['simplify]
        * Positive buffer of Polygon by config_pp['positive_buffer]
        * Negative Buffer of POlygons by config_pp['negative_buffer]
        * Simplification of Polygon by config_pp['simplify]
        * Remove polygons whose area is less or greater than data in config_pp['filter]['plane_area']
        * Remove holes whose vertices are less than data in config_pp['filter]['hole_vertices']
        * Remove holes whose area is less or greater than data in config_pp['filter]['hole_area']

    It then returns the shapely polygons of the polygons and holes (obstacles)

    An example config_pp

    .. code-block:: python

        {
            positive_buffer: 0.005 # m, Positively expand polygon.  Fills in small holes
            negative_buffer: 0.03 # m, Negative buffer to polygon. Expands holes and constricts outer hull of polygon
            simplify: 0.02  # m, simplify edges of polygon
            filter: # obstacles must have these characteristics
                hole_area:
                    min: 0.025   # m^2
                    max: 0.785 # m^2
                hole_vertices:
                    min: 6
                plane_area:
                    min: .5 # m^2
        }

    Arguments:
        polygons {list[Polygons]} -- A list of polygons returned from polylidar
        points {ndarray} -- MX3 array
        config_pp {dict} -- Configuration for post processing filtering
        rm {scipy.spatial.RotationMatrix} -- Rotation matrix applied to 3D polygons to make 2D

    Returns:
        List -- A list of planar shapely polygons
    """
    # filtering configuration
    post_filter = config_pp['filter']

    # will hold the plane(s) and obstacles found
    planes = []
    # print("Polylidar returned {} polygons, ".format(len(polygons)))
    for poly in polygons:
        t0 = time.perf_counter()
        if rm is not None:
            shell_coords = rm.apply(get_points(poly.shell, points))
            hole_coords = [rm.apply(get_points(hole, points)) for hole in poly.holes]
        else:
            shell_coords = get_points(poly.shell, points)
            hole_coords = [get_points(hole, points) for hole in poly.holes]
        t1 = time.perf_counter()
        poly_shape = Polygon(shell=shell_coords, holes=hole_coords)
        t2 = time.perf_counter()
        # print("Before: ", poly_shape.is_valid)
        # fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
        # plot_poly(poly_shape, ax, poly)
        # plt.axis('equal')
        # plt.show()
        # print(poly_shape.is_valid)
        # assert poly_shape.is_valid
        area = poly_shape.area
        # logging.info("Got a plane!")
        if post_filter['plane_area']['min'] and area < post_filter['plane_area']['min']:
            # logging.info("Skipping Plane")
            continue
        z_value = shell_coords[0][2]

        t3 = time.perf_counter()
        if config_pp['simplify']:
            poly_shape = poly_shape.simplify(
                tolerance=config_pp['simplify'], preserve_topology=True)
        t4 = time.perf_counter()
        # Perform 2D geometric operations
        if config_pp['positive_buffer']:
            poly_shape = poly_shape.buffer(
                config_pp['positive_buffer'], join_style=JOIN_STYLE.mitre, resolution=4)
        t5 = time.perf_counter()
        if config_pp['negative_buffer']:
            poly_shape = poly_shape.buffer(
                distance=-config_pp['negative_buffer'], join_style=JOIN_STYLE.mitre, resolution=4)
            # if poly_shape.geom_type == 'MultiPolygon':
            #     all_poly_shapes = list(poly_shape.geoms)
            #     poly_shape = sorted(
            #         all_poly_shapes, key=lambda geom: geom.area, reverse=True)[0]
        t6 = time.perf_counter()
        # poly_shape = poly_shape.buffer(distance=config_pp['negative_buffer'], resolution=4)
        if config_pp['simplify']:
            poly_shape = poly_shape.simplify(
                tolerance=config_pp['simplify'], preserve_topology=True)  # False makes fast, but can cause invalid polygons
        t7 = time.perf_counter()
        if poly_shape.geom_type == 'MultiPolygon':
            all_poly_shapes = list(poly_shape.geoms)
            # poly_shape = sorted(
            #     all_poly_shapes, key=lambda geom: geom.area, reverse=True)[0]
        else:
            all_poly_shapes = [poly_shape]

        logging.debug("Rotation: {:.2f}; Polygon Creation: {:.2f}; Simplify 1: {:.2f}; Positive Buffer: {:.2f}; Negative Buffer: {:.2f}; Simplify 2: {:.2f}".format(
            (t1 - t0) * 1000, (t2 - t1) * 1000, (t4 - t3) * 1000, (t5 - t4) * 1000, (t6 - t5) * 1000, (t7 - t6) * 1000
        ))

        # Its possible that our polygon has no broken into a multipolygon
        # Check for this situation and handle it
        # all_poly_shapes = [poly_shape]
        # print(len(all_poly_shapes))
        # iterate through every polygons and check for plane extraction
        for poly_shape in all_poly_shapes:
            area = poly_shape.area
            # print(poly_shape.geom_type, area)
            # logging.info("Plane is big enough still")
            if post_filter['plane_area']['min'] <= 0 or area >= post_filter['plane_area']['min']:
                dim = np.asarray(poly_shape.exterior).shape[1]
                # logging.info("Plane is big enough still")
                if config_pp['negative_buffer'] or config_pp['simplify'] or config_pp['positive_buffer'] and dim < 3:
                    # convert back to 3D coordinates
                    # create kd tree for vertex lookup after buffering operations
                    t8 = time.perf_counter()
                    kd_tree = create_kd_tree(shell_coords, hole_coords)
                    t9 = time.perf_counter()
                    poly_shape = recover_3d(poly_shape, kd_tree, z_value)
                    t10 = time.perf_counter()
                    logging.debug("Create KD Tree: {:.2f}; Recover Polygon 3D Coordinates: {:.2f}".format(
                        (t9 - t8) * 1000, (t10 - t9) * 1000
                    ))

                # Capture the polygon as well as its z height
                # after applying buffering and simplification with shapely/geos all polygons are valid
                # print(poly_shape.is_valid)
                valid_shell = poly_shape.exterior
                valid_holes = []

                for hole_lr in poly_shape.interiors:
                    # Filter by number of obstacle vertices, removes noisy holes
                    if len(hole_lr.coords) > post_filter['hole_vertices']['min']:
                        hole_poly = Polygon(shell=hole_lr)
                        area = hole_poly.area
                        # filter by area
                        if post_filter['hole_area']['min'] <= 0.0 or area >= post_filter['hole_area']['min'] and area < post_filter['hole_area']['max']:
                            valid_holes.append(hole_lr)

                new_plane_polygon = Polygon(shell=poly_shape.exterior, holes=valid_holes)
                planes.append((new_plane_polygon, z_value))
    
    if rm is not None:
        t11 = time.perf_counter()
        rm_inv = rm.inv()
        for i, (poly, z_value) in enumerate(planes):
            shell_points = rm_inv.apply(np.asarray(poly.exterior))
            hole_points = [rm_inv.apply(np.asarray(hole_lr)) for hole_lr in poly.interiors]
            new_poly = Polygon(shell=shell_points, holes=hole_points)
            planes[i] = (new_poly, z_value)
        t12 = time.perf_counter()
        logging.debug("Revert Rotation and Create New Polygons: {:2f}".format((t12 - t11) * 1000))
    return planes

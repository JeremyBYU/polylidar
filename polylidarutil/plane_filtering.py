import numpy as np
from scipy import spatial
from shapely.geometry import Polygon, JOIN_STYLE


def get_points(point_idxs, points):
    return points[point_idxs, :]


def create_kd_tree(shell_coords, hole_coords):
    hole_coords.append(shell_coords)
    all_vertices = np.vstack(hole_coords)
    kd_tree = spatial.KDTree(all_vertices, leafsize=100)
    return kd_tree


def add_column(array, z_value):
    ones = np.ones((array.shape[0], 1)) * z_value
    stacked = np.column_stack((array, ones))
    return stacked


def recover_3d(poly, kd_tree, z_value):
    shell_3D = add_column(np.array(poly.exterior), z_value)
    # print(shell_3D.shape)
    d, shell_idx = kd_tree.query(shell_3D)
    # print(shell_idx.shape)
    kd_data = kd_tree.data[shell_idx, :]
    # print(kd_data.shape)
    shell_3D[:, 2] = kd_data[:, 2]
    holes_lr = []
    for hole in poly.interiors:
        hole_lr = add_column(np.array(hole), z_value)
        d, shell_idx = kd_tree.query(hole_lr)
        kd_data = kd_tree.data[shell_idx, :]
        hole_lr[:, 2] = kd_data[:, 2]
        holes_lr.append(hole_lr)

    poly_3d = Polygon(shell=shell_3D, holes=holes_lr)
    return poly_3d


def filter_planes_and_holes(polygons, points, config_pp):
    """Extracts the plane and obstacles returned from polylidar
    Will filter polygons according to: number of vertices and size
    Will also buffer (dilate) and simplify polygons

    Arguments:
        polygons {list[Polygons]} -- A list of polygons returned from polylidar
        points {ndarray} -- MX3 array
        config_pp {dict} -- Configuration for post processing filtering
          Example Configuration
            filter: # obstacles must have these characteristics
                hole_area:
                    min: 0.025   # m^2
                    max: 0.785 # m^2
                hole_vertices:
                    min: 6
                plane_area:
                    min: .5 # m^2
            # These parameters correspond to Shapely polygon geometry operations
            positive_buffer: 0.005 # m, Positively expand polygon.  Fills in small holes
            negative_buffer: 0.03 # m, Negative buffer to polygon. Expands holes and constricts outer hull of polygon
            simplify: 0.02  # m, simplify edges of polygon

    Returns:
        tuple -- A list of plane shapely polygons and a list of obstacle polygons
    """
    # filtering configuration
    post_filter = config_pp['filter']

    # will hold the plane(s) and obstacles found
    planes = []
    obstacles = []
    for poly in polygons:
        # shell_coords = [get_point(pi, points) for pi in poly.shell]
        shell_coords = get_points(poly.shell, points)
        hole_coords = [get_points(hole, points) for hole in poly.holes]
        poly_shape = Polygon(shell=shell_coords, holes=hole_coords)
        area = poly_shape.area
        # logging.info("Got a plane!")
        if area < post_filter['plane_area']['min']:
            # logging.info("Skipping Plane")
            continue
        z_value = shell_coords[0][2]
        if config_pp['simplify']:
            poly_shape = poly_shape.simplify(
                tolerance=config_pp['simplify'], preserve_topology=True)
        # Perform 2D geometric operations
        if config_pp['positive_buffer']:
            poly_shape = poly_shape.buffer(
                config_pp['positive_buffer'], join_style=JOIN_STYLE.mitre, resolution=4)
        if config_pp['negative_buffer']:
            poly_shape = poly_shape.buffer(
                distance=-config_pp['negative_buffer'], resolution=4)
            if poly_shape.geom_type == 'MultiPolygon':
                all_poly_shapes = list(poly_shape.geoms)
                poly_shape = sorted(
                    all_poly_shapes, key=lambda geom: geom.area, reverse=True)[0]
            # poly_shape = poly_shape.buffer(distance=config_pp['negative_buffer'], resolution=4)
        if config_pp['simplify']:
            poly_shape = poly_shape.simplify(
                tolerance=config_pp['simplify'], preserve_topology=False)
        if poly_shape.geom_type == 'MultiPolygon':
            all_poly_shapes = list(poly_shape.geoms)
            poly_shape = sorted(
                all_poly_shapes, key=lambda geom: geom.area, reverse=True)[0]

        # Its possible that our polygon has no broken into a multipolygon
        # Check for this situation and handle it
        all_poly_shapes = [poly_shape]

        # iterate through every polygons and check for plane extraction
        for poly_shape in all_poly_shapes:
            area = poly_shape.area
            # logging.info("Plane is big enough still")
            if area >= post_filter['plane_area']['min']:
                # logging.info("Plane is big enough still")
                if config_pp['negative_buffer'] or config_pp['simplify'] or config_pp['positive_buffer']:
                    # convert back to 3D coordinates
                    # create kd tree for vertex lookup after buffering operations
                    kd_tree = create_kd_tree(shell_coords, hole_coords)
                    poly_shape = recover_3d(poly_shape, kd_tree, z_value)
                # Capture the polygon as well as its z height
                new_plane_polygon = Polygon(shell=poly_shape.exterior)
                planes.append((new_plane_polygon, z_value))

                for hole_lr in poly_shape.interiors:
                    # Filter by number of obstacle vertices, removes noisy holes
                    if len(hole_lr.coords) > post_filter['hole_vertices']['min']:
                        hole_poly = Polygon(shell=hole_lr)
                        area = hole_poly.area
                        # filter by area
                        if area >= post_filter['hole_area']['min'] and area < post_filter['hole_area']['max']:
                            z_value = hole_lr.coords[0][2]
                            obstacles.append((hole_poly, z_value))
    return planes, obstacles

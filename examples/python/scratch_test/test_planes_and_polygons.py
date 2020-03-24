import numpy as np
from polylidar import Delaunator, MatrixDouble, Polylidar3D
import matplotlib.pyplot as plt

np.random.seed(1)

def get_np_buffer_ptr(a):
    pointer, read_only_flag = a.__array_interface__['data']
    return pointer

def main():
    points = np.random.randn(1000, 2)
    polylidar_kwargs = dict(alpha=0.0, lmax=0.1)
    # Use Delaunay Triangulation
    points_mat = MatrixDouble(points)
    polylidar = Polylidar3D(**polylidar_kwargs)
    # delaunay = Delaunator(points_mat)
    # delaunay.triangulate()

    # triangles_ = np.asarray(delaunay.triangles)
    # print(triangles_)
    # print("")
    mesh, planes, polygons = polylidar.extract_planes_and_polygons(points_mat)
    triangles1 = np.asarray(mesh.triangles)
    
    # print(triangles1)
    # print(np.max(triangles1))
    # print(triangles1.flags)
    # print(triangles1.dtype)
    # print(get_np_buffer_ptr(triangles1))
    # print("")

    # triangles2 = np.copy(triangles1)
    # print(triangles2)
    # print(np.max(triangles2))
    # print(triangles2.flags)
    # print(triangles2.dtype)
    # print(get_np_buffer_ptr(triangles2))
    # print("")

    # print(triangles1)
    # print(np.max(triangles1))
    # print(triangles1.flags)
    # print(triangles1.dtype)
    # print(get_np_buffer_ptr(triangles1))
    # print("")
    
    plt.triplot(points[:,0], points[:,1], triangles1)

    plt.scatter(points[:, 0], points[: ,1])
    plt.show()

if __name__ == "__main__":
    main()
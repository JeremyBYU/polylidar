"""Used to just test Delaunay triangulation
"""
import numpy as np
from polylidar import Delaunator, MatrixDouble
import matplotlib.pyplot as plt

def main():
    points = np.random.randn(1000, 2)

    # Use Delaunay Triangulation
    points_mat = MatrixDouble(points)
    delaunay = Delaunator(points_mat)
    delaunay.triangulate()
    triangles = np.copy(np.asarray(delaunay.triangles))

    plt.triplot(points[:,0], points[:,1], triangles)

    plt.scatter(points[:, 0], points[: ,1])
    plt.show()

if __name__ == "__main__":
    main()
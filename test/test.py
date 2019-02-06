import numpy as np
from polylidar import Delaunator

a = np.random.rand(100, 2)
delaunay = Delaunator(a)
delaunay.triangulate()

print(type(delaunay.coords))


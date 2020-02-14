import time
import numpy as np
from polylidar import extract_point_cloud_from_float_depth


im = np.ones((424, 240), dtype=np.float32)
intrin = np.array([[307, 0, 0], 
                [0, 307, 0],
                [204, 121, 1]])

for i in range(100):
    t0 = time.perf_counter()
    pc = extract_point_cloud_from_float_depth(im, intrin, 1)
    t1 = time.perf_counter()
    elapsed = (t1-t0)*1000
    # print("Took: {:.2f}".format(elapsed))

print("Took: {:.2f}".format(elapsed))

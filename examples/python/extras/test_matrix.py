"""Used to test Polyidars Matrix Class
This test ensures that a no copy operation actually works
"""
import numpy as np
from polylidar import Polylidar3D, MatrixDouble

def get_np_buffer_ptr(a):
    pointer, read_only_flag = a.__array_interface__['data']
    return pointer

def main():
    a = np.arange(4, dtype=np.float64).reshape((2,2))
    print("Numpy Array and Buffer Ptr")
    print(a)
    print(get_np_buffer_ptr(a))
    b = MatrixDouble(a, False)
    c = np.asarray(b)
    print("")
    print("Matrix Array and Buffer Ptr")
    print(c)
    print(get_np_buffer_ptr(c))
    try:

        d = a.reshape((4,))
        print("")
        print("Reshape to 1 Dimension")
        print(d)
        print("Should throw runtime error because not 2 Dim")
        e = MatrixDouble(d, False)
    except Exception:
        print("Successfully threw an exception")



if __name__ == "__main__":
    main()
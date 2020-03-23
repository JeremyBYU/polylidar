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

    d = a.reshape((4,))
    print("Should throw runtime error because not 2 Dim")
    e = MatrixDouble(d, False)



if __name__ == "__main__":
    main()
import time
import pickle
import numpy as np
from polylidar import Delaunator, MatrixDouble, Polylidar3D
from polylidar.polylidarutil import (generate_test_points, plot_points, plot_triangles, get_estimated_lmax,
                                     plot_triangle_meshes, get_triangles_from_list, get_colored_planar_segments, plot_polygons)
import matplotlib.pyplot as plt

np.random.seed(1)


def get_np_buffer_ptr(a):
    pointer, read_only_flag = a.__array_interface__['data']
    return pointer


def stitch_letters(letters_map, letters=['P', 'L']):
    # letters = ['P', 'O', 'L', 'Y', 'L', 'I', 'D', 'A', 'R']
    x_offset = np.array([0.0, 0.0])
    x_spacing = np.array([-30.0, 0.0])
    all_points = []
    for letter in letters:
        letter_points = np.copy(letters_map[letter])
        letter_points = letter_points + x_offset + x_spacing
        x_offset[0] = letter_points.max(axis=0)[0]
        all_points.append(letter_points)

    all_points = np.concatenate(all_points, axis=0)

    return all_points

def read_alphabet(file_name='fixtures/pointsets/alphabet_2000.pkl'):
    with open(file_name, 'rb') as file:
        data = pickle.load(file)
    letters = ['P', 'O', 'L', 'Y', 'I', 'D', 'A', 'R']
    letter_map = dict()
    for letter in letters:
        letter_map[letter] = [item for item in data if item['poly_param']['name'] == letter][0]['points']
    
    return letter_map


def main():

    letter_map = read_alphabet()
    # letters = ['P', 'O', 'L', 'Y', 'L', 'I', 'D', 'A', 'R']
    points = stitch_letters(letter_map)

    # plt.scatter(points[:, 0], points[:, 1], s=0.2)
    # plt.show()
    print(int(points.shape[0] / 2))
    idx = np.random.randint(0, points.shape[0], size=int(points.shape[0] / 2))
    points = np.ascontiguousarray(points[idx, :])
    lmax = 10.0
    polylidar_kwargs = dict(alpha=0.0, lmax=lmax, min_triangles=5)
    # print(polylidar_kwargs)
    # Convert Points and make Polylidar
    points_mat = MatrixDouble(points)
    polylidar = Polylidar3D(**polylidar_kwargs)
    # Extracts planes and polygons, time
    t1 = time.perf_counter()
    mesh, planes, polygons = polylidar.extract_planes_and_polygons(points_mat)
    t2 = time.perf_counter()
    triangles = np.asarray(mesh.triangles)
    # print(triangles, triangles.shape)
    triangles = triangles.flatten()
    # print(triangles, triangles.shape)
    planes_np = planes
    # print(planes_np)
    print("Took {:.2f} milliseconds".format((t2 - t1) * 1000))

    # Plot Data
    if points.shape[0] < 100000:
        fig, ax = plt.subplots(figsize=(10, 10), nrows=1, ncols=1)
        ax.set_xticks([])
        ax.set_yticks([])
        plt.axis('equal')
        plt.axis('off')
        # plot points
        plot_points(points, ax)
        fig.savefig('assets/scratch/pl_logo_points.png', bbox_inches='tight', transparent=True)
        # plot polygons
        plot_polygons(polygons, points, ax, linewidth=6.0)
        fig.savefig('assets/scratch/pl_logo_poly.png', bbox_inches='tight', transparent=True)
        # plot all triangles
        plot_triangles(get_triangles_from_list(triangles, points), ax)
        # plot mesh triangles
        triangle_meshes = get_colored_planar_segments(planes_np, triangles, points)
        plot_triangle_meshes(triangle_meshes, ax)
        # plot polygons
        plot_polygons(polygons, points, ax, linewidth=4.0)

        plt.subplots_adjust(wspace=0.185, hspace=0.185,left=0.110,top=0.535,right=0.750,bottom=0.110) 
        fig.savefig('assets/scratch/pl_logo.png', bbox_inches='tight', transparent=True)
        plt.show()


if __name__ == "__main__":
    main()

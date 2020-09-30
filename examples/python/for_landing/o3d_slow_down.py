"""Demo of Open3D 0.10.0 Slowdown
Please modify DIRECTORY to point to the folder of meshes attached to this issue reply
"""
import os
import open3d as o3d
DIRECTORY = 'o3d_slow_down'
def main():
    all_meshes = []
    all_files = sorted(list(os.listdir(DIRECTORY)))
    for filename in all_files:
        print(filename)
        mesh = o3d.io.read_triangle_mesh(os.path.join(DIRECTORY, filename))
        all_meshes.append(mesh)
    o3d.visualization.draw_geometries(all_meshes)

if __name__ == "__main__":
    main()
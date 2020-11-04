"""Demo of Open3D 0.10.0 Slowdown
Please modify DIRECTORY to point to the folder of meshes attached to this issue reply
"""
import os
import open3d as o3d
import copy
DIRECTORY = 'fixtures/o3d_slow_down'
# o3d 0.10.0 - 9 Seconds to load meshes (time to being user interation), 1 FPS (with draw edges enabled 'w')
# o3d 0.11.0+f1d478c4 - 0.5 seconds, 45-60 FPS (with draw edges)

duplicate = 50
def main():
    all_meshes = []
    all_files = sorted(list(os.listdir(DIRECTORY)))
    for filename in all_files:
        print(filename)
        mesh = o3d.io.read_triangle_mesh(os.path.join(DIRECTORY, filename))
        
        all_meshes.extend([ copy.deepcopy(mesh) for i in range(duplicate)])
    print(len(all_meshes))
    o3d.visualization.draw_geometries(all_meshes)

if __name__ == "__main__":
    main()
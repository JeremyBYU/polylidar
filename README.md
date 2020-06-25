<h1 align="center">
  Polylidar3D
  <br>
</h1>

<h4 align="center">Polygon Extraction from 2D Point Sets, Unorganized/Organized 3D Point Clouds, and Triangular Meshes</h4>

<p align="center">
  <a href="#key-features">Key Features</a> •
  <a href="#install">Install</a> •
  <a href="#documentation">Documentation</a> •
  <a href="#polylidar-use-cases">Use Cases</a> •
  <a href="#credits">Credits</a> •
  <a href="#related-methods">Related</a> •
  <a href="#license">License</a>
</p>

<p align="middle">
  <img src="https://raw.githubusercontent.com/JeremyBYU/polylidar/major_changes_refactor/assets/polylidar_3D_architecture.jpg" height="100%" />
  <!-- <img src="https://raw.githubusercontent.com/JeremyBYU/polylidarv2/master/assets/polylidar_3D_architecture.png" height="100%" />  -->
</p>

## Key Features

* Fast (Multi)Polygon Extraction from multiple sources of 3D Data
  * Written in C++ for portability
  * Extremely fast single-threaded but includes CPU multi-threading using data and task-based parallelism
  * Polygons with holes may be returned
* Python3 bindings using PyBind11
  * Low overhead for calling python/cpp interface (no copying of point cloud data)
* Python and C++ Examples
  * Examples from 2D Point Sets, Unorganized 3D point clouds, Organized 3D point clouds (i.e., range images), and user provided meshes
* Cross platform
  * Windows and Linux ready.

Polylidar3D is a non-convex polygon extraction algorithm which takes as input either unorganized 2D point sets, unorganized 3D point clouds (e.g., airborne LiDAR point clouds), organized point clouds (e.g., range images), or user provided meshes. The non-convex polygons extracted represent flat surfaces in an environment, while interior holes represent obstacles on said surfaces. Polylidar3D is an extension to Polylidar (note the lack of 3D), the authors 2D algorithm which transforms 2D point sets into polygons. The picture above provides an overview of Polylidar3D's data input, frontend, backend, and output. The frontend transforms input data into a *half-edge* triangular mesh.  This representation provides a common level of abstraction such that the the back-end core algorithms may efficiently operate on. The back-end is composed of four core algorithms: mesh smoothing, dominant plane normal estimation, planar segment extraction, and finally polygon extraction.  Polylidar3D outputs *planar triangular segments*, sets of flat connected triangles, and their polygonal representations. Polylidar3D is extremely fast, taking as little as a few milliseconds and makes use of CPU multi-threading and GPU acceleration when available.

Currently this repo (named Polylidar3D) has all the front-end modules and the plane/polygon extraction of the back-end core algorithms. The GPU accelerated mesh smoothing procedures for organized points clouds are found in a separate repo titled [OrganizedPointFilters](https://github.com/JeremyBYU/OrganizedPointFilters). This must be installed if you desire fast mesh smoothing for organized point clouds (i.e., denoising). The dominant plane normal estimation procedure is general and implemented in a separate repo titled [Fast Gaussian Accumulator (FastGA)](https://github.com/JeremyBYU/FastGaussianAccumulator). This must be installed if you don't know the dominant plane normals in your data input (very likely for organized point clouds and meshes). These modules themselves are written in C++ as well with Python bindings; see the respective repos for installation instructions. One day I will try to ease installation burden and automatically pull these dependencies into the build process.

## Install

Installation is entirely through CMake now. You must have CMake 3.14 or higher installed and a C++ compiler with C++ 14 or higher. No built binaries are included currently.

<!-- ### Build Project Library

There are several config options which can be specified here during step (2):

```text
PL_BUILD_BENCHMARKS:BOOL=ON // PL - Build Benchmarks
PL_BUILD_EXAMPLES:BOOL=ON // PL - Build Examples
PL_BUILD_PYMODULE:BOOL=ON // PL -Build Python Module
PL_BUILD_TESTS:BOOL=ON // PL - Build Tests
PL_BUILD_WERROR:BOOL=OFF // PL - Add Werror flag to build (turns warnings into errors)
PL_USE_ROBUST_PREDICATES:BOOL=OFF // PL - Use Robust Geometric Predicates
PL_WITH_OPENMP:BOOL=ON // PL - Build with OpenMP Support
```

1. `mkdir cmake-build && cd cmake-build` - create build folder directory
2. `cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF` - This will take 10-20 minutes while dependencies are being downloaded from Github. Sorry, take a break!
3. `cmake --build . -j4` - Build Polylidar3D, change `-j4` to how many processors/threads you have. 
3. `./build/polylidar-simple` - Simple test program.

### Build and Install Python Extension

The basic setup here is that CMake will build the python extension (.so or .dll) into a standalone folder. Then you use `pip` to install from that folder.

1. Install [conda](https://conda.io/projects/conda/en/latest/) or create a python virtual environment ([Why?](https://medium.freecodecamp.org/why-you-need-python-environments-and-how-to-manage-them-with-conda-85f155f4353c)). I recommend `conda` for Windows users. Activate your environment.
2. `conda install shapely` - Only for Windows users because `conda` handles windows binary dependency correctly.
3. `cd cmake-build` - Ensure you are in cmake build directory.
4. `cmake --build . --target python-package --config Release -j4` - Build python extension, will create folder `cmake-build/lib/python_package`
5. `cd lib/python_package` - Change to standalone python package. 
6. `pip install -e .` - Install the python package in `develop/edit` mode into python virtual environment.
7. `cd ../../../ && pip install -r dev-requirements.txt` - Move back to main folder and install optional dependencies to run python examples.

### C++ Projects

#### CMake Integration

To integrate with a *different* CMake Project do the following:

1. Add as a submodule into your repo: `git submodule add https://github.com/JeremyBYU/polylidar thirdparty/polylidar` 
2. Add the following to your CMakeLists.txt file:

```text
add_subdirectory("thirdparty/polylidar")
... .
target_link_libraries(MY_BINARY polylidar)

``` 

### Robust Geometric Predicates

Delaunator (the 2D triangulation library used) does not use [robust geometric predicates](https://github.com/mikolalysenko/robust-arithmetic-notes) for its orientation and incircle tests; [reference](https://github.com/mapbox/delaunator/issues/43).  This means that the triangulation can be incorrect when points are nearly colinear or cocircular. A library developed by Jonathan Richard Shewchuk provides very fast adaptive precision floating point arithmetic for [geometric predicates](https://www.cs.cmu.edu/~quake/robust.html).  This library is released in the public domain and an updated version of it is maintained at this [repository](https://github.com/danshapero/predicates). I have included this source code in the folder `polylidar/predicates` .  

If you desire to have robust geometric predicates built into Delaunator you must build with the CMake option. For example `cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF -DPL_USE_ROBUST_PREDICATES=ONE`. -->

## Documentation

Please see documentation.

## Polylidar Use Cases

* [Polylidar-RealSense](https://github.com/JeremyBYU/polylidar-realsense) - Live ground floor detection with Intel RealSense camera using Polylidar
* [Polylidar-KITTI](https://github.com/JeremyBYU/polylidar-kitti) - Street surface and obstacle detection from autonomous driving platform.
* [PolylidarWeb](https://github.com/JeremyBYU/polylidarweb). An very old Typescript (javascript) version with live demos of Polylidar2D.
* [Concave-Evaluation](https://github.com/JeremyBYU/concavehull-evaluation) - Evaluates and benchmarks several competing concavehull algorithms.

## Credits

This software is only possible because of the great work from the following open source packages:

* [Delaunator](https://github.com/mapbox/delaunator) - Original triangulation library 
* [DelaunatorCPP](https://github.com/delfrrr/delaunator-cpp) - Delaunator ported to C++ (used)
* [parallel-hashmap](https://github.com/greg7mdp/parallel-hashmap) - Very fast hashmap library (used)
* [PyBind11](https://github.com/pybind/pybind11) - Python C++ Binding (used)
* [Robust Geometric Predicates](https://www.cs.cmu.edu/~quake/robust.html) - Original Robust Geometric predicates
* [Updated Predicates](https://github.com/danshapero/predicates) -Updated geometric predicate library (used)

## Related Methods

### 2D ConcaveHull Extraction

* [CGAL Alpha Shapes](https://doc.cgal.org/latest/Alpha_shapes_2/index.html) - MultiPolygon with holes.
* [PostGIS ConcaveHull](http://postgis.net/docs/ST_ConcaveHull.html) - Single Polygon with holes.
* [Spatialite ConcaveHull](https://www.gaia-gis.it/fossil/libspatialite/wiki?name=tesselations-4.0) - MultiPolygon with holes.
* [Concaveman](https://github.com/mapbox/concaveman) - A 2D concave hull extraction algorithm for 2D point sets.


## Contributing

Any help or suggestions would be appreciated!


## License

MIT

---

> GitHub [@jeremybyu](https://github.com/JeremyBYU)


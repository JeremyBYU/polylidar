# Polylidar V2

Polylidar allows one to extract planar meshes from a point cloud **and** their 2D projected polygons. The point cloud can be in 2, 3, or 4 dimensions (XY, XYZ, XYZC=Class). This module is written in C++ and is built as a python plugin.  A Typescript (javascript) version is also created as well.

The image below is polylidar (Typescript/Website version) with a classified point cloud data set.  A point cloud of a building roof (purple dots) and a tarp (green) dots are seen on the left.  The extracted polygon of the roof is seen on the right. The green line represents the concave hull of the extracted mesh, and the orange lines represent *holes* in the polygon.

![Polylidar Example](assets/polylidar-example.png)


Much of this work is possible because of the amazing library called [delaunator](https://github.com/delfrrr/delaunator-cpp).  It provides a very fast 2D delaunay triangulation that outputs a data structure that is perfectly suitable for this work. All the triangles are indexed by [half-edges](https://mapbox.github.io/delaunator/) allowing the quick extraction and polygon generation of any planar meshes.

## Installing

1. Install [conda](https://conda.io/projects/conda/en/latest/) - [Why?](https://medium.freecodecamp.org/why-you-need-python-environments-and-how-to-manage-them-with-conda-85f155f4353c)
2. `conda install -c conda-forge pybind11`
3. `conda install shapely` - Only needed for windows binary dependency
3. `python setup.py build install`

Please run the tests to ensure everything is working. Pytest should be installed as it is listed as a dependency.

1. Simply type `pytest` to run tests

## Demo

You can see a demo in action py running `python tests/visualize.py`. Requires `matplotlib descartes seaborn`.

## API

What are the inputs to the code?  The input arguments are a **contiguous** numpy array with length N and 2,3,or 4 columns depending on your data.  There are also configuration options as well that you can pass as keyword arguments.

What are the outputs?

* Delaunay - This is a C++ class data structure that has information about your triangles, half edges, and point indices. Read more [here](https://mapbox.github.io/delaunator/).
* planes - This is a *list* of C++ *vectors* holding `ints`. Each vector is an extracted plane.  The `ints` correspond to triangle indices.
* polygons - This is a *list* of C++ `polygon` data structure.
* polygon - This is a struct that has two fields: shell and holes. Shell is a *vector* of `ints`, where each int represents a *point* index. Holes is a list of a vector of `ints`. Each vector represents a hole in the polygon.

Example calls
```python
from polylidar import extractPlanesAndPolygons, extractPolygons, Delaunator

# You want everything!
delaunay, planes, polygons = extractPlanesAndPolygons(point_cloud:ndarray)

# Show me JUST the polygons!
polygons = extractPolygons(point_cloud:ndarray)

# Also if you just want fast 2D delaunay triangulation, no polylidar
delaunay = Delaunator(point_cloud:ndarray)
```



## Benchmark

| # Points | Time (ms) |
|----------|-----------|
| 10,000   | 5         |
|          |           |

## Issues

* If there are coincident points then it seems the delaunator cpp library gets stuck in an infinite loop (sometimes?). Add a very small amount of noise to the data (jitter) to ensure this doesn't happen.




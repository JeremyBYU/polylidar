# Polylidar V2

Polylidar allows one to extract planar meshes from a point cloud **and** their 2D projected polygons. The point cloud can be in 2, 3, or 4 dimensions (XY, XYZ, XYZC=Class). This module is written in C++ and is built as a python plugin.  A Typescript (javascript) version is also created as well.

The image below is polylidar (Typescript/Website version) with a classified point cloud data set.  A point cloud of a building roof (purple dots) and a tarp (green) dots are seen on the left.  The extracted polygon of the roof is seen on the right. The green line represents the concave hull of the extracted mesh, and the orange lines represent *holes* in the polygon.

![Polylidar Example](assets/polylidar-example.png)


Much of this work is possible because of the amazing library called [delaunator](https://github.com/delfrrr/delaunator-cpp).  It provides a very fast 2D delaunay triangulation that outputs a data structure that is perfectly suitable for this work. All the triangles are indexed by [half-edges](https://mapbox.github.io/delaunator/) allowing the extraction and polygon generation of any planar meshes very quick.

## Installing

1. Install [conda](https://conda.io/projects/conda/en/latest/) - [Why?](https://medium.freecodecamp.org/why-you-need-python-environments-and-how-to-manage-them-with-conda-85f155f4353c)
2. `conda install -c conda-forge pybind11`
3. `python setup.py build install`

Please run the tests to ensure everything is working. Pytest should be installed as it is listed as a dependency.

1. Simply type `pytest` to run tests

## Demo

You can see a demo in action py running `python tests/visualize.py`. Requires `matplotlib descartes seaborn`.

## API

`delaunay, planes, polygons = extractPlanesAndPolygons(point_cloud:ndarray`)

## Benchmark

| # Points | Time (ms) |
|----------|-----------|
| 10,000   | 5         |
|          |           |




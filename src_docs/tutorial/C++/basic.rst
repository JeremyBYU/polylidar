.. _cplusplus_basic_tutorial:

C++ interface
------------------

Take a look at `examples/cpp/polylidar-full-example.cpp`.

First we load a numpy file of an organized point cloud that has been previosly captured. 

.. literalinclude:: ../../../examples/cpp/polylidar-full-example.cpp
    :language: cpp
    :lines: 42-64

Then we convert the vector to a matrix wrapper (no copy) that Polylidar uses.

.. literalinclude:: ../../../examples/cpp/polylidar-full-example.cpp
    :language: cpp
    :lines: 66-71

Then we visualize the point cloud:

.. literalinclude:: ../../../examples/cpp/polylidar-full-example.cpp
    :language: cpp
    :lines: 73-77

.. image:: /_static/cpp_tutorial/opc.png


Then we create and visualize the mesh:

.. literalinclude:: ../../../examples/cpp/polylidar-full-example.cpp
    :language: cpp
    :lines: 79-92

.. image:: /_static/cpp_tutorial/opc_mesh.png


Then we find the dominant plane normals using FastGA:

.. literalinclude:: ../../../examples/cpp/polylidar-full-example.cpp
    :language: cpp
    :lines: 94-99


Then we extract the Planes and Polygons:

.. literalinclude:: ../../../examples/cpp/polylidar-full-example.cpp
    :language: cpp
    :lines: 101-107


Then we colorize the Open3D mesh by plane segments and create the corresponding polygons:

.. literalinclude:: ../../../examples/cpp/polylidar-full-example.cpp
    :language: cpp
    :lines: 109-133

.. image:: /_static/cpp_tutorial/opc_polygons.png


Full output:

.. code-block:: bash

    $ ./cmake-build/bin/Release/polylidar-full.exe
    A more complete example of using Polylidar3D with 3D Data. Needs Open3D and FastGA.
    Loading previously captured Organized Point Cloud from an L515 Camera.
    Attempting to load OPC file from ./fixtures/realsense/opc_example_one/L515_OPC.npy
    Shape of Point Cloud is: [180, 320, 3]
    Visualizing Point Cloud. Rotate View. Close Open3D window when satisfied...
    Creating Mesh and smoothing. NOT using optimized filters, see OrganizedPointFilters
    Visualizing mesh. Rotate View.
    Detected Peaks with FastGaussianAccumulator. Here are the detected peaks: [0.0641286, 0.525564, -0.848333], [0.00935028, -0.849775, -0.527063],

    Extracting Polygons from all detected peaks:
    Visualing Planes and Polygon. Each plane segment is color coded. Polgyons are shown as thin lines (OpenGL has no thickness):
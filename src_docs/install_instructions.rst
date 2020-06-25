.. _install_instructions:

Install Instructions
====================

Installation is entirely through CMake now. You must have CMake 3.14 or higher installed and a C++ compiler with C++ 14 or higher. No built binaries are included currently.

Build Project Library
------------------------------------

There are several config options which can be specified here during step (2):

.. code:: text

    PL_BUILD_BENCHMARKS:BOOL=OFF // PL - Build Benchmarks
    PL_BUILD_EXAMPLES:BOOL=ON // PL - Build Examples
    PL_BUILD_PYMODULE:BOOL=ON // PL -Build Python Module
    PL_BUILD_TESTS:BOOL=ON // PL - Build Tests
    PL_BUILD_WERROR:BOOL=OFF // PL - Add Werror flag to build (turns warnings into errors)
    PL_USE_ROBUST_PREDICATES:BOOL=OFF // PL - Use Robust Geometric Predicates
    PL_WITH_OPENMP:BOOL=ON // PL - Build with OpenMP Support


1. ``mkdir cmake-build && cd cmake-build`` - create build folder directory
2. ``cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF`` - This will take 10-20 minutes while dependencies are being downloaded from Github. Sorry, take a break!
3. ``cmake --build . -j4`` - Build Polylidar3D, change ``-j4`` to how many processors/threads you have. 
4. ``./build/polylidar-simple`` - Simple test program.

Build and Install Python Extension
------------------------------------

The basic setup here is that CMake will build the python extension (.so or .dll) into a standalone folder. Then you use `pip` to install from that folder.

1. Install `conda <https://conda.io/projects/conda/en/latest/>`_ or create a python virtual environment (`Why? <https://medium.freecodecamp.org/why-you-need-python-environments-and-how-to-manage-them-with-conda-85f155f4353c>`_). I recommend ``conda`` for Windows users. Activate your environment.
2. ``conda install shapely`` - Only for Windows users because ``conda`` handles windows binary dependency correctly.
3. ``cd cmake-build`` - Ensure you are in cmake build directory.
4. ``cmake --build . --target python-package --config Release -j4`` - Build python extension, will create folder ``cmake-build/lib/python_package``
5. ``cd lib/python_package`` - Change to standalone python package. 
6. ``pip install -e .`` - Install the python package in ``develop/edit`` mode into python virtual environment.
7. ``cd ../../../ && pip install -r dev-requirements.txt`` - Move back to main folder and install optional dependencies to run python examples.

C++ Projects
-------------

CMake Integration
^^^^^^^^^^^^^^^^^^

To integrate with a *different* CMake Project do the following:

1. Add as a submodule into your repo: ``git submodule add https://github.com/JeremyBYU/polylidar thirdparty/polylidar``
2. Add the following to your CMakeLists.txt file:

.. code:: text

    add_subdirectory("thirdparty/polylidar")
    ... .
    target_link_libraries(MY_BINARY polylidar)


Robust Geometric Predicates
---------------------------

Delaunator (the 2D triangulation library used for 2D Point Sets and Unorganized 3D Point Clouds) does not use `robust geometric predicates <https://github.com/mikolalysenko/robust-arithmetic-notes>`_ for its orientation and incircle tests; `reference <https://github.com/mapbox/delaunator/issues/43>`_. 
This means that the triangulation can be incorrect when points are nearly colinear or cocircular. A library developed by Jonathan Richard Shewchuk provides very fast adaptive precision floating point arithmetic for `geometric predicates <https://www.cs.cmu.edu/~quake/robust.html>`_.  
This library is released in the public domain and an updated version of it is maintained at this `repository <https://github.com/danshapero/predicates>`_. I have included this source code in the folder ``polylidar/predicates`` .  

If you desire to have robust geometric predicates built into Delaunator you must build with the CMake option. For example ``cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF -DPL_USE_ROBUST_PREDICATES=ON``.
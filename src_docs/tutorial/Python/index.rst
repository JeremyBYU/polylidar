Python
===================================================================

Python Tutorials

.. toctree::
    :maxdepth: 0

    basicdemo
    organizeddemo


.. warning::
    Polylidar3D uses Open3D to visualize 3D geometries (pointclouds, polygons, meshes, etc.). A recent version of Open3D has a serious performance regression which causes severe slowdown during visualization. 
    This regression is on versions 0.10 and 0.11 of Open3D. Regression details: `Link <https://github.com/intel-isl/Open3D/pull/2523>`_ , `Issue1 <https://github.com/intel-isl/Open3D/issues/2472>`_ , `Issue2 <https://github.com/intel-isl/Open3D/issues/2157>`_.
    I recommend that you stick with 0.9.0, build from master, or wait for 0.12.
try:
    from .polylidar_pybind import * # py2 py3 compatible
except Exception:
    # this was installed with as a python wheel
    from polylidar_pybind import *

try:
    import pkg_resources  # part of setuptools
    __version__ = pkg_resources.require("polylidar")[0].version
except Exception:
    __version__ = '@PROJECT_VERSION@'

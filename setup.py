import os
import sys
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import setuptools


USE_ROBUST_PREDICATES_NAME = 'USE_ROBUST_PREDICATES'
USE_ROBUST_PREDICATES = int(os.environ.get( USE_ROBUST_PREDICATES_NAME, 0 ))
if USE_ROBUST_PREDICATES:
    print("Building with robust geometric predicates.")
else:
    print("NOT building with robust predicates")


__version__ = '0.0.3'

class get_pybind_include(object):
    """Helper class to determine the pybind11 include path

    The purpose of this class is to postpone importing pybind11
    until it is actually installed, so that the ``get_include()``
    method can be invoked. """

    def __init__(self, user=False):
        self.user = user

    def __str__(self):
        import pybind11
        include_path = os.path.dirname(pybind11.get_include(self.user))
        return include_path

class get_numpy_include(object):
    """Helper class to determine the numpy include path
    The purpose of this class is to postpone importing numpy
    until it is actually installed, so that the ``get_include()``
    method can be invoked. """

    def __init__(self):
        pass

    def __str__(self):
        import numpy as np
        return np.get_include()

# Source files for polylidar
source_files = ['polylidar/module.cpp', 'polylidar/polylidar.cpp', 'polylidar/delaunator.cpp', 'polylidar/helper.cpp']
# Source files for robust geometric predicates
robust_files = ['polylidar/predicates/constants.c', 'polylidar/predicates/predicates.c', 'polylidar/predicates/printing.c', 'polylidar/predicates/random.c']
# Include directories for polylidar
include_dirs = [get_pybind_include(), get_pybind_include(user=True), get_numpy_include(), 'polylidar/']

# If compiling with robust predicates then add robust c and header files
if USE_ROBUST_PREDICATES:
    source_files.extend(robust_files)
    include_dirs.append('polylidar/predicates/')

ext_modules = [
    Extension(
        'polylidar',
        source_files,
        include_dirs=include_dirs,
        language='c++'
    ),
]


# As of Python 3.6, CCompiler has a `has_flag` method.
# cf http://bugs.python.org/issue26689
def has_flag(compiler, flagname):
    """Return a boolean indicating whether a flag name is supported on
    the specified compiler.
    """
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.cpp') as f:
        f.write('int main (int argc, char **argv) { return 0; }')
        try:
            compiler.compile([f.name], extra_postargs=[flagname])
        except setuptools.distutils.errors.CompileError:
            return False
    return True


def cpp_flag(compiler):
    """Return the -std=c++[11/14] compiler flag.

    The c++14 is prefered over c++11 (when it is available).
    """
    if has_flag(compiler, '-std=c++14'):
        return '-std=c++14'
    elif has_flag(compiler, '-std=c++11'):
        return '-std=c++11'
    else:
        raise RuntimeError('Unsupported compiler -- at least C++11 support '
                           'is needed!')


class BuildExt(build_ext):
    """A custom build extension for adding compiler-specific options."""
    c_opts = {
        'msvc': ['/EHsc'],
        'unix': [],
    }

    if sys.platform == 'darwin':
        c_opts['unix'] += ['-stdlib=libc++', '-mmacosx-version-min=10.7']

    def build_extensions(self):
        ct = self.compiler.compiler_type
        opts = self.c_opts.get(ct, [])
        opts.append('-DPY_EXTENSION') # inform compiler that we are compiling as a pyextension
        if ct == 'unix':
            opts.append('-DVERSION_INFO="%s"' % self.distribution.get_version())
            opts.append('-Wall')
            opts.append(cpp_flag(self.compiler))
            if has_flag(self.compiler, '-fvisibility=hidden'):
                opts.append('-fvisibility=hidden')
            if USE_ROBUST_PREDICATES:
                opts.append('-DUSE_ROBUST')
        elif ct == 'msvc':
            opts.append('/DVERSION_INFO=\\"%s\\"' % self.distribution.get_version())
            if USE_ROBUST_PREDICATES:
                opts.append('-DUSE_ROBUST')
        for ext in self.extensions:
            ext.extra_compile_args = opts
        build_ext.build_extensions(self)

setup(
    name='polylidar',
    version=__version__,
    author='Jeremy Castagno',
    author_email='jdcasta@umich.edu',
    packages=['polylidarutil', 'polylidar'],
    url='',
    description='Polygon extraction from Point Cloud data',
    long_description='',
    ext_modules=ext_modules,
    install_requires=['pybind11>=2.2', 'numpy', 'pytest', 'shapely', 'pytest-benchmark', 'descartes', 'matplotlib'],
    cmdclass={'build_ext': BuildExt},
    zip_safe=False,
)

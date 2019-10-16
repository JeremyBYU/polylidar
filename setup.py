import os
import sys
from os.path import join
from pathlib import Path
import shutil
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import setuptools

PL_USE_ROBUST_PREDICATES_NAME = 'PL_USE_ROBUST_PREDICATES'
PL_USE_ROBUST_PREDICATES = int(os.environ.get(PL_USE_ROBUST_PREDICATES_NAME, 0))
if PL_USE_ROBUST_PREDICATES:
    print("Building with robust geometric predicates.")
else:
    print("NOT building with robust predicates")

PL_USE_STD_UNORDERED_MAP_NAME = 'PL_USE_STD_UNORDERED_MAP'
PL_USE_STD_UNORDERED_MAP = int(os.environ.get(PL_USE_STD_UNORDERED_MAP_NAME, 0))
if PL_USE_STD_UNORDERED_MAP:
    print("Building with slower std::unordered_map.")
else:
    print("Building with fast flat_hash_map. 30% Speedup. Does not work with GCC7. See Wiki.")

def get_pybind_include_new():
        """Correct way to get pybinds include path.
        Lifted from https://github.com/matplotlib/mplcairo
        """
        try:
            import importlib.metadata as importlib_metadata
        except ImportError:
            import importlib_metadata
        # pybind11.get_include() is brittle (pybind #1425).
        pybind11_include_path = next(
            path for path in importlib_metadata.files("pybind11")
            if path.name == "pybind11.h").locate().parents[1]
        if not (pybind11_include_path / "pybind11/pybind11.h").exists():
            # egg-install from setup_requires:
            # importlib-metadata thinks the headers are at
            #   .eggs/pybind11-VER-TAG.egg/pybind11-VER.data/headers/pybind11.h
            # but they're actually at
            #   .eggs/pybind11-VER-TAG.egg/pybind11.h
            # pybind11_include_path is
            #   /<...>/.eggs/pybind11-VER-TAG.egg/pybind11-VER.data
            # so just create the proper structure there.
            try:
                is_egg = (pybind11_include_path.relative_to(
                    Path(__file__).resolve().parent).parts[0] == ".eggs")
            except ValueError:
                # Arch Linux ships completely wrong metadata, but the headers
                # are in the default include paths, so just leave things as is.
                is_egg = False
            if is_egg:
                shutil.rmtree(pybind11_include_path / "pybind11",
                              ignore_errors=True)
                for file in [*pybind11_include_path.parent.glob("**/*")]:
                    if file.is_dir():
                        continue
                    dest = (pybind11_include_path / "pybind11" /
                            file.relative_to(pybind11_include_path.parent))
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(file, dest)

        return pybind11_include_path

# Source files for polylidar
source_files = ['polylidar/module.cpp', 'polylidar/polylidar.cpp', 'polylidar/delaunator.cpp', 'polylidar/helper.cpp']
# Source files for robust geometric predicates
robust_files = ['polylidar/predicates/constants.c', 'polylidar/predicates/predicates.c', 'polylidar/predicates/printing.c', 'polylidar/predicates/random.c']
# Include directories for polylidar
include_dirs = ['polylidar/', 'polylidar/parallel_hashmap']

# If compiling with robust predicates then add robust c and header files
if PL_USE_ROBUST_PREDICATES:
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
        import numpy as np
        print(self.distribution.get_version())

        ext, = self.distribution.ext_modules
        ext.depends += [
            "setup.py",
            *map(str, Path("polylidar").glob("*.h")),
            *map(str, Path("polylidar").glob("*.cpp")),
        ]
        ext.include_dirs += [get_pybind_include_new(), np.get_include()]

        ct = self.compiler.compiler_type
        opts = self.c_opts.get(ct, [])
        lopts = []
        opts.append('-DPY_EXTENSION') # inform compiler that we are compiling as a pyextension
        if PL_USE_ROBUST_PREDICATES:
            opts.append('-DPL_USE_ROBUST_PREDICATES')
        if PL_USE_STD_UNORDERED_MAP:
            opts.append('-DPL_USE_STD_UNORDERED_MAP')
        if ct == 'unix':
            opts.append('-fopenmp')
            lopts.append('-fopenmp')
            opts.append('-DVERSION_INFO="%s"' % self.distribution.get_version())
            opts.append('-Wall')
            opts.append(cpp_flag(self.compiler))
            if has_flag(self.compiler, '-fvisibility=hidden'):
                opts.append('-fvisibility=hidden')
        elif ct == 'msvc':
            opts.append('/openmp')
            opts.append('/DVERSION_INFO=\\"%s\\"' % self.distribution.get_version())
        for ext in self.extensions:
            ext.extra_compile_args = opts
            ext.extra_link_args = lopts
        build_ext.build_extensions(self)


DEV = ['pytest', 'pytest-xdist', 'pytest-benchmark', 'pylint', 'twine', 'autopep8', 'nox']

this_directory = os.path.abspath(os.path.dirname(__file__))
with open(join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='polylidar',
    author='Jeremy Castagno',
    author_email='jdcasta@umich.edu',
    packages=['polylidarutil', 'polylidar'],
    url='',
    description='Polygon extraction from Point Cloud data',
    long_description=long_description,
    long_description_content_type='text/markdown',
    ext_modules=ext_modules,
    package_data={  # Optional
        'polylidar': ['*.hpp', 'parallel_hashmap/*'],
    },
    include_package_data=True,
    install_requires=['numpy>=1.15.0', 'pybind11>=2.2.4', 'shapely', 'matplotlib', 'descartes'],
    extras_require={
        'dev': DEV
    },
    setup_requires=[
        "importlib_metadata>=0.8",
        "setuptools_scm",
        'pybind11>=2.2.4',
        "numpy>=1.15.0",
    ],
    use_scm_version={  # xref __init__.py
        "version_scheme": "post-release",
        "local_scheme": "node-and-date"
    },
    cmdclass={'build_ext': BuildExt},
    zip_safe=False,
)

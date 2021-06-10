

## Building the Full Polylidar3D C++ Example

This is meant to show a more complete example of usiing Polyidar3D in C++. Note that it is using FastGA but currently is not using the OrganizedPointFilters.
I am using Open3D for visualization.

You need to activate this option in CMake: `option(PL_BUILD_FASTGA "PL - Build FastGA with Example" OFF)`. Example - `-DPL_BUILD_FASTGA=ON`.
Read below for how to link with Open3D

### Getting Open3D to Compile with Polylidar3D

It can be quite difficult to get Open3D to compile with this project (polylidar) on Windows. Open3D, by default, links with a STATIC Windows **runtime** (don't confuse this with static library). This is done with msvc with the flag `\MT`. However CMAKE by defualt will use a DYNAMIC windows **runtime** (see [here](https://cmake.org/cmake/help/latest/prop_tgt/MSVC_RUNTIME_LIBRARY.html)). This is the `\MD` flag when compiling. If you are going to compile Open3D into any external project they both have to use the *same* runtime. If you don't you get a bunch of errors like this: `error LNK2038: mismatch detected for 'RuntimeLibrary': value 'MT_StaticRelease' doesn't match value 'MD_DynamicRelease' in`

An oh my can it be really hard to make sure that CMake compiled all your code and all your possible dendencies with the same runtime. For this reason, I recommend that you compile Open3D by hand and switch it to use the dyamic runtime for Windows. I could never get it to work using Open3D 0.13.0, so I just stick with 0.12.0. These are the commands I used to build Open3D on my windows machine. I bet this will be easier on linux.


```bash
git clone --recursive --branch v0.12.0 https://github.com/intel-isl/Open3D
cd Open3D && mkdir build && cd build
cmake -G "Visual Studio 16 2019" -A x64 -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=./open3d_install -DSTATIC_WINDOWS_RUNTIME=OFF ..
cmake --build . --config Release --parallel 12 --target install
```

Then I build the fully integrated Polylidar3D cpp example by (cd to the `cmake-build` in polylidar repo)

```
cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF -DCMAKE_GENERATOR_PLATFORM=x64 -DPL_BUILD_FASTGA=ON -DOpen3D_ROOT=/c/Users/Jerem/Documents/UMICH/Research/Open3D/build/open3d_install
cmake --build . --config Release -j4
```


<!-- if(WIN32)
    # target_compile_options(${PROJECT_NAME} PRIVATE /MT)
endif() -->
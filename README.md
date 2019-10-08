# Polylidar Bug Isolation

This bug has been isolated to be just in the C++ portion of code. No python is necessary to demonstrate this bug.
This bug only manifests with GCC7 with -O3 optimizations enabled. THe issue has been isolated to being with the robin_hood::unordered_map.

So first things first: the code **compiles** and **runs perfectly fine** on linux with GCC5,GCC6,GCC8,GCC9, Clang6 and Windows on MSVC2017 with all optimizations enabled. It **does** compile with GCC7 with -O3 optimization enabled *but* runs into an infinite loop because of an issue with a robinhood hash map as I will explain in the section **Details**.

To reproduce: Go into the folder `examples/cpp`. The program `simple.cpp` will demonstrate the bug.

**Demonstrate bug with GCC7**

1. `export CXX=g++-7 && export CC=g++-7` - Just make sure you are compiling with GCC7
2. `make CC=$CC CXX=$CXX`
3. `./bin/simple` - Supposed to finish instantly *but* will go into infinite loop. memory will overfill. use ctrl-c after a few seconds to prevent.

**Remove bug with GCC9**

1. `make clean`
2. `export CXX=g++-9 && export CC=g++-9`
3. `make CC=$CC CXX=$CXX`
4. `./bin/simple` - Finishes

**Remove Bug with -O1 with GCC7**

If you change the optimization level to O1 in the Makefile the bug will go away with GCC7


**Remove Bug with one line change in GCC7 with -O3**

I can also make the bug go away with a single line change. Uncomment line 436 in `polylidar.cpp`:
```
    // std::cout << delaunay.triangles << std::endl; // commenting this line causes bug in GCC-7 with -O3
```
I don't think this line has anything truly related with the issue. I think somehow whatever bad optimization is being made by GCC is skipped when this std::cout occurs. 

**Cause bug another way with GCC7 -O3**

Go to ``helper.hpp`` and uncomment out line 6:
```
// #define NDEBUG // uncommenting this will cause an error in gcc7 with -O3 enabled
```


## Details:

I am using robinhood:unordered_map<size_t,size_t> with variable name `triHash`. It really is just meant as a set keeping track of integer ids which correspond to triangles.  The lines are 435 and 437 in `polylidar.cpp` that declare and insert these triangles inside.  In the example there will be the integers 0-10. I have verified that this works just fine.

On lines 465 I begin to do a loop, pulling integers out of the hash map named the `seedIdx` and calling `extractMeshHash` on line 474. In this function I have diagnosed the issue with several debugging statements.  The issue is two fold:

1. checking if a valid key (integer) exists in the hashmap returns false (incorrectly). The key really is there!
2. Erasing the seedIdx from the Hashmap has no effect.

I have verified these by printing out the content of the hashmap when the bug is being manifest. These issues lead to the infinite loop.


## Dumping optimization information

I have tried to dump optimization information from gcc and analyze the difference between nobug/bug case. I used this command:

`g++ -g -O3 -Wall -fopt-info-optimized-optall=opt_not_working.txt -c ../../polylidar/polylidar.cpp -I../../`

However, I have been unsuccessful at understanding what is causing the issues. I see differences in some of the optimizations but nothing really seems to stands out. I have spent hours changing some things (removing createTriHash2/4) and nothing removes the bug.

Note I learned that -O2 will also cause the optimization error in gcc7. However if you compile with O2 the code is always broken with gcc, you cant get some working states like I have shown above with -O3.

## Alternate hashmaps

I have tried to replace robin_hood with [parallel_hashmap](https://github.com/greg7mdp/parallel-hashmap). It has the same issue! They are both using the same technique that google has introduced with absail:flat_hash_map (flat hash map, dense array). 
This is making me think that GCC7 can run into issues with this technique possibly.










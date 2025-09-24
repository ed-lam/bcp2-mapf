BCP2-MAPF
=========

BCP2-MAPF is an implementation of a branch-and-cut-and-price algorithm for the multi-agent path finding problem. It is the second version of the earlier [BCP-MAPF](https://github.com/ed-lam/bcp-mapf) code. BCP2-MAPF is described in the following paper:

[Low-Level Search on Time Intervals in Branch-and-Cut-and-Price for Multi-Agent Path Finding](https://ed-lam.com/papers/bcp2mapf2025.pdf). Edward Lam and Peter J. Stuckey. International Symposium on Combinatorial Search. 2025.

Please cite this article if you use this code for the multi-agent path finding problem or as a template for other branch-and-cut-and-price codes.

License
-------

BCP2-MAPF is released under the PolyForm Noncommercial License 1.0.0. Source code is made available for academic use. Commercial use is not permitted. See [LICENSE.md](LICENSE.md) for details.

Dependencies
------------

BCP2-MAPF is implemented in C++23 and is built using CMake, so you will need a recent compiler and a recent version of CMake. It has been tested on Mac built using Clang 19 and Linux built using GCC 11. It has not been tested on Windows. If you're using Mac, you may need to download an updated version of Clang/LLVM that supports C++23 using [Homebrew](https://brew.sh/) because the built-in AppleClang is old.

BCP2-MAPF calls Gurobi for solving the linear relaxation. Gurobi is commercial software but provides free binaries under an academic license. BCP2-MAPF is tested with Gurobi 11 and 12.

Compiling
---------

1. Download and install the [binaries](https://www.gurobi.com/downloads/gurobi-software/) for Gurobi.

2. Obtain an [academic license](https://www.gurobi.com/features/academic-named-user-license/) for Gurobi.

3. Download the source code to BCP2-MAPF by cloning this repository:
```
git clone https://github.com/ed-lam/bcp2-mapf.git
cd bcp2-mapf
```

4. Compile BCP2-MAPF.

    1. Locate the directory of Gurobi, which should contain the `include` and `lib` subdirectories. Copy the directory path into the following command:
    ```
    cmake -DGUROBI_DIR={PATH TO GUROBI DIRECTORY} -B build/ .
    ```
    If you're using a custom compiler (e.g., an updated version of Clang on Mac), you will need to specify the path to the compiler:
    ```
    cmake -DGUROBI_DIR={PATH TO GUROBI DIRECTORY} -DCMAKE_C_COMPILER=/opt/homebrew/opt/llvm/bin/clang -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ -B build/ .
    ```

    2. Build the executable:
    ```
    cmake --build build/ --target bcp2-mapf --parallel
    ```

Usage
-----

After compiling, run it with:
```
./build/bcp2-mapf {PATH TO INSTANCE}
```

You can also set a time limit in seconds:
```
./build/bcp2-mapf —-time-limit={TIME LIMIT} {PATH TO INSTANCE}
```

The [Moving AI benchmarks](https://movingai.com/benchmarks/mapf.html) can be found in the `instances/movingai` directory. Most scenarios have a total of 1000 agents. You can specify how many of the first N agents to run. For example, you can run an instance with only the first 50 agents:
```
./build/bcp2-mapf --time-limit=30 --agent-limit=50 instances/movingai/Berlin_1_256-random-1.scen
```

Contributing
------------

Code contributions and scientific discussion are welcome. Please raise any issue with the author.

Authors
-------

BCP2-MAPF is developed by Edward Lam. Edward can be reached at [ed-lam.com](https://ed-lam.com).
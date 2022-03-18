# public_emoa

C++ Implementation of Enhanced Multi-Objective A\* (EMOA\*) Algorithm.

The code is distributed for academic and non-commercial use.

### Requirements

* We use CMake (3.16.3) and Make (4.2.1) to compile the code. Lower or higher version may also work.

## Project Structure

* `README.md` - This file
* `source/` - Contains the path planning source code
* `test/` - Contains example code for path planning algorithms
* `include/` - Contains header files
* `data/` - Contains sample graph files and sample result files

## Instructions:

### Run EMOA

* Clone this repo
* Compile this repo
  * `mkdir build`
  * `cd build`
  * `cmake ..` (You can specify the build type you like by adding additional args)
  * `make`
* Run example in `./test_emoa `
* Run example via command-line interface
  * `./run_emoa 2 data/graph1.txt data/graph2.txt data/result.txt`
* General usage of the command-line interface
  * `./run_emoa (arg1 M) (arg2 graph1_path) (arg3 graph2_path) ... ((arg(M+1) graphM_path)) (arg(M+2) result_path) `
  * arg1 M = the number of objectives for the input instance
  * arg2~arg(M+1) = the paths to M files that describe the graph, where each file contains the edge weights for one type of edge cost in the graph (details about file structure are specified below)
  * arg(M+2) = the path of the result file
* For help info `./run_emoa -h` or `./run_emoa --help`

### Graph file specification

Graphs are directed graphs labeled from 1~*n*, where *n* is the number of vertices.

If parallel edges are specified in the graph file, only the lexico-smallest edge is used.

Graph files must follow the same format described by [DIMACS](http://www.diag.uniroma1.it//~challenge9/format.shtml#graph).

Graph files used in the same run must correspond to each other line-by-line.

### Result file specifiction

Result file contains 7 lines of metadata (time, n_generated, n_expanded, n_domCheck, rt_initHeu, rt_search, N). N is the number of solutions found.

Each of the N solutions are then listed in sets of three lines:
1. The first line contains `Label: {label_id}`, where the label_id identifies the solution
2. The second line contains the (space-separated) cost vector of the solution
3. The third line contains the (space-separated) path of vertices for the solution

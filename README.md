# public_emoa

This work addresses a Multi-Objective Shortest Path Problem (MO-SPP) on a graph where the goal is to find a set of Pareto-optimal solutions from a start node to a destination in the graph. This repo provides a C++ implementation of Enhanced Multi-Objective A\* (EMOA\*) Algorithm, which is guaranteed to find all cost-unique Pareto-optimal solutions and runs faster than existing techniques by up to an order of magnitude. The code is distributed for academic and non-commercial use. More technical details can be found in [1](https://arxiv.org/pdf/2202.08992.pdf).

<img src="https://github.com/wonderren/wonderren.github.io/blob/master/images/fig_emoa_NY17.png" alt="" align="middle" hspace="20" style=" border: #FFFFFF 2px none;">

(Fig 1: Some Pareto-optimal solution paths in a New York City roadmap that optimize path length, arrival time and path risk.)

## Requirements

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
  * `./run_emoa 1 5 60 3 data/ex1-c1.txt data/ex1-c2.txt data/ex1-c3.txt data/result.txt`
  * Runs EMOA\* on 3-cost graph (edge weights detailed in `data/ex1-c1.txt`, `data/ex1-c2.txt`, `data/ex1-c3.txt`) to find solutions from node 1 to node 5 with a 60 second time limit, and saves results into `data/result.txt`
* General usage of the command-line interface
  * `./run_emoa (arg1 v_start) (arg2 v_dest) (arg3 time_limit) (arg3 M) (arg4 graph1_path) (arg5 graph2_path) ... ((arg(M+3) graphM_path)) (arg(M+4) result_path) `
  * arg1 v_start = the starting node
  * arg2 v_dest = the destination node
  * arg3 time_limit = the time limit for EMOA\*
  * arg4 M = the number of objectives for the input instance
  * arg5~arg(M+4) = the paths to M files that describe the graph, where each file contains the edge weights for one type of edge cost in the graph (details about file structure are specified below)
  * arg(M+5) = the path of the result file
* For help info `./run_emoa -h` or `./run_emoa --help`

### Graph file specification

Graphs are directed graphs labeled from 1~*n*, where *n* is the number of vertices.

If parallel edges are specified in the graph file, only the lexico-smallest edge is used.

Graph files must follow the same format described by [DIMACS](http://www.diag.uniroma1.it//~challenge9/format.shtml#graph).

Graph files used in the same run must correspond to each other line-by-line.

### Result file specifiction

Result file contains 7 lines of metadata (graph_load_time, n_generated, n_expanded, n_domCheck, rt_initHeu, rt_search, N). N is the number of solutions found.

Each of the N solutions are then listed in sets of three lines:
1. The first line contains `Label: {label_id}`, where the label_id identifies the solution
2. The second line contains the (space-separated) cost vector of the solution
3. The third line contains the (space-separated) path of vertices for the solution


## Others

### References

* [1] [Enhanced Multi-Objective A* Using Balanced Binary Search Trees](https://arxiv.org/pdf/2202.08992.pdf).\
  Zhongqiang Ren, Richard Zhan, Sivakumar Rathinam, Maxim Likhachev and Howie Choset.

### Development Team

Contributors: [Zhongqiang (Richard) Ren](https://wonderren.github.io/), Richard Zhan.

Advisors: Prof. Sivakumar Rathinam (TAMU), Prof. Maxim Likhachev (CMU), Prof. Howie Choset (CMU).
# Enhanced Multi-Objective A* (EMOA*)

This work addresses a Multi-Objective Shortest Path Problem (MO-SPP) on a graph where the goal is to find a set of Pareto-optimal solutions from a start node to a destination in the graph. This repo provides a C++ implementation of Enhanced Multi-Objective A\* (EMOA\*) Algorithm, which is guaranteed to find all cost-unique Pareto-optimal solutions and runs faster than existing techniques by up to an order of magnitude. More technical details can be found in [[1](https://arxiv.org/pdf/2202.08992.pdf)].

The code is distributed for academic and non-commercial use.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

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

### Installation

* Clone this repo
* Compile this repo
  * `mkdir build`
  * `cd build`
  * `cmake ..` (You can specify the build type you like by adding additional args)
  * `make`
* Run example in `./test_emoa `

### Command-Line Interface (CLI)

* Run example via command-line interface (CLI)
  * `./run_emoa 1 5 60 3 ../data/ex1-c1.gr ../data/ex1-c2.gr ../data/ex1-c3.gr ../data/result.txt`
  * Runs EMOA\* on 3-cost graph (edge weights detailed in `../data/ex1-c1.gr`, `../data/ex1-c2.gr`, `../data/ex1-c3.gr`) to find solutions from node 1 to node 5 with a 60 second time limit, and saves results into `data/result.txt`
* General usage of the command-line interface
  * `./run_emoa (arg1 v_start) (arg2 v_dest) (arg3 time_limit) (arg4 M) (arg5 graph1_path) (arg6 graph2_path) ... ((arg(M+4) graphM_path)) (arg(M+5) result_path)`
  * arg1 v_start = the starting node
  * arg2 v_dest = the destination node
  * arg3 time_limit = the time limit for EMOA\*
  * arg4 M = the number of objectives for the input instance
  * arg5~arg(M+4) = the paths to M files that describe the graph, where each file contains the edge weights for one type of edge cost in the graph (details about file structure are specified below)
  * arg(M+5) = the path of the result file
* For help info `./run_emoa -h` or `./run_emoa --help`

### Preliminary Python API

* We have also developed a simple Python wrapper based on the aforementioned CLI (by writing and reading files and call the CLI), which can be found in python/emoa_py_api.py
* Run `cd python/` and then run `python3 emoa_py_api.py`, a toy example using the python wrapper can be executed.
* The current Python wrapper is only applicable to grid-like map. For general usage, please use the CLI.
* More APIs may be developed in the future.

### Graph file specification

* The input graph is directed.
* The node ID are within range 1~*n*, where *n* is the number of nodes in the graph. 
* Parallel edges are not allowed.
* Graph files must follow the same format described by [DIMACS](http://www.diag.uniroma1.it//~challenge9/format.shtml#graph).
* Graph files used in the same run must correspond to each other line-by-line.
 * In other words, it requires all cost files to have the same set of edges in the same order.
```
For example:
In c1 file, let’s say we have edges:
a 0 1 26
a 2 4 26
a 3 5 114
Then in c2 file, we also need to have edges:
a 0 1 x
a 2 4 y
a 3 5 z
Here, x,y,z are non-negative integer cost values. If, for example, edge (3,5) has cost zero, this edge also need to appear in the third place (i.e., the same place as in the c1 file) in c2 file, and has a value 0.
Same rule applies to all cost files.
```

### Result file specifiction

Result file contains 7 lines of metadata (graph_load_time, n_generated, n_expanded, n_domCheck, rt_initHeu, rt_search, N). N is the number of solutions found.

Each of the N solutions are then listed in sets of three lines:
1. The first line contains `Label: {label_id}`, where the label_id identifies the solution
2. The second line contains the (space-separated) cost vector of the solution
3. The third line contains the (space-separated) path of vertices for the solution


### Notes for Performance

* The implementation of EMOA* (as well as the baselines as mentioned in the paper [1]) relies heavily on std::unordered_map from C++ STL for the purpose of easy implementation. Using other data structure such as std::vector (or simply arrays) can lead to improvement in performance than using std::unordered_map.

### References

* [1] Enhanced Multi-Objective A* Using Balanced Binary Search Trees.\
  Zhongqiang Ren, Richard Zhan, Sivakumar Rathinam, Maxim Likhachev and Howie Choset.\
  [[Bibtex](https://wonderren.github.io/files/bibtex_ren22emoa.txt)][[Paper](https://wonderren.github.io/files/ren22_emoa_socs.pdf)]

### Development Team

Contributors: [Zhongqiang (Richard) Ren](https://wonderren.github.io/), Richard Zhan.

Advisors: Prof. Sivakumar Rathinam (TAMU), Prof. Maxim Likhachev (CMU), Prof. Howie Choset (CMU).

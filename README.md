# public_emoa
C++ Implementation of Enhanced Multi-Objective A\* (EMOA\*) Algorithm.

### Requirements
* We use CMake (3.16.3) and Make (4.2.1) to compile the code. Lower or higher version may also work.

## Project Structure
* `README.md` - This file
* `source/` - Contains the path planning source code
* `test/` - Contains example code for path planning algorithms
* `include/` - Contains header files

## Instructions:

### Run EMOA
* Clone this repo
* Compile this repo
  * `mkdir build`
  * `cd build`
  * `cmake ..` (You can specify the build type you like by adding additional args)
  * `make`
* Run toy example `./test_emoa `
* Run via command-line interface (TODO fill in or refine the text below)
  * `./run_emoa (arg1 M) (arg2 graph_1) (arg3 graph_2) ... ((arg(M+1) graph_M)) (arg(M+2) result_file_name) `
  * arg1 M = the number of objectives for the input instance
  * arg2~arg(M+1) = totally M graphs, each specify one type of edge cost. The graph file structure is described below.
  * arg(M+2) = the name/path of the result file to be output.
* For help info `./run_emoa -h` or `./run_emoa --help`

### Graph file specification (TODO fill in or refine the text below)
The graph file follows a similar format as DIAMCS ...

### Result file specifiction (TODO fill in or refine the text below)
The result file ...
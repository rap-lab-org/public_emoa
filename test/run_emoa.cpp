
/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/

#include "api.hpp"
#include "graph_io.hpp"
#include "debug.hpp"
#include <iostream>
#include <string>

int TestNY3();

void print_help_message();

int main( int argc, char *argv[] ) {

  // help
  if (argc >= 2) {
    std::string arg1 = argv[1];
    if (arg1 == "-h" || arg1 == "--help") {
      print_help_message();
      return 0;
    }
  }

  // get args
  for (int i = 0; i < argc; i++){
    std::cout << "[INFO] input arg[" << i << "]=" << argv[i] << std::endl;
  }

  long vo = std::stol(argv[1]);
  long vd = std::stol(argv[2]);
  double time_limit = std::stod(argv[3]);
  int M = std::stoi(argv[4]);

  int expected_args = M + 6;
  if (argc != expected_args) {
    std::cout << "M=" << M << ", expected M+6 (" << expected_args << ") arguments, received " << argc << std::endl;
    return -1;
  }

  // get filenames
  std::string result_fname = argv[argc - 1];
  std::vector<std::string> input_fnames;
  for (int i = 0; i < M; i++) {
    input_fnames.push_back(argv[i + 5]);
  }


  // timer for loading graph files
  rzq::basic::SimpleTimer timer;
  timer.Start();

  rzq::basic::Roadmap g;
  {
    int status = rzq::basic::ReadRoadmapFromFile(input_fnames, false, &g);
    if (status < 0) {
      // if return value is less than 0, then error
      std::cout << "Error: ReadRoadmapFromFile()" << std::endl;
      return -1;
    }
  }

  double load_graph_time = timer.GetDurationSecond();

  // do some print and verification to make sure the generated graph is correct.
  std::cout << "num_nodes: " << g.GetNumberOfNodes() << std::endl;
  std::cout << "num_edges: " << g.GetNumberOfEdges() << std::endl;
  std::cout << "cdims: " << g.GetCostDim() << std::endl;

  // ######################################### //
  // ####### Test 2 - run planner ######### //
  // ######################################### //

  rzq::search::EMOAResult res;
  rzq::search::RunEMOA(&g, vo, vd, time_limit, &res);

  rzq::basic::SaveResultToFile(result_fname, load_graph_time, &res);

  return 1;
};

void print_help_message () {
  std::cout << "./run_emoa (arg1 v_start) (arg2 v_dest) (arg3 time_limit) (arg4 M) (arg5 graph1_path) (arg6 graph2_path) ... ((arg(M+4) graphM_path)) (arg(M+5) result_path)" << std::endl;
}

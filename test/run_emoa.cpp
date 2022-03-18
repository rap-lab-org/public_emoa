
/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/

#include "api.hpp"
#include "graph_io.hpp"
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

  int M = std::stoi(argv[1]);

  if (argc != M + 3) {
    std::cout << "M=" << M << ", expected M+3 (" << M + 3 << ") arguments, received " << argc << std::endl;
    return -1;
  }

  // get filenames
  std::string result_fname = argv[argc - 1];
  std::vector<std::string> input_fnames;
  for (int i = 0; i < M; i++) {
    input_fnames.push_back(argv[i + 2]);
  }


  rzq::basic::Roadmap g;
  {
    int status = rzq::basic::ReadRoadmapFromFile(input_fnames, false, &g);
    if (status < 0) {
      // if return value is less than 0, then error
      std::cout << "Error: ReadRoadmapFromFile()" << std::endl;
      return -1;
    }
  }

  // do some print and verification to make sure the generated graph is correct.
  std::cout << "num_nodes: " << g.GetNumberOfNodes() << std::endl;
  std::cout << "num_edges: " << g.GetNumberOfEdges() << std::endl;
  std::cout << "cdims: " << g.GetCostDim() << std::endl;

  // ######################################### //
  // ####### Test 2 - run planner ######### //
  // ######################################### //

  long vo = 1; // starting node in the graph.
  long vd = 5; // destination node id in the graph.
  double time_limit = 60; // e.g. one minute search time limit.

  // rzq::basic::SimpleTimer timer;
  // timer.Start();

  rzq::search::EMOAResult res;
  rzq::search::RunEMOA(&g, vo, vd, time_limit, &res);

  rzq::basic::SaveResultToFile(result_fname, -1, &res);

  return 1;
};

void print_help_message () {
  std::cout << "./run_emoa (arg1 M) (arg2 graph1_path) (arg3 graph2_path) ... ((arg(M+1) graphM_path)) (arg(M+2) result_path)" << std::endl;
}

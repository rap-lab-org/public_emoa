
/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/

#include "api.hpp"
#include "graph_io.hpp"
#include <iostream>
#include <string>

int TestNY3();

int main( int argc, char *argv[] ) {

  for (int i = 0; i < argc; i++){
    std::cout << "[INFO] input arg[" << i << "]=" << argv[i] << std::endl;
  }

  rzq::basic::Roadmap g;
  std::string fname_suffix = "NY";
  std::string dist_data_fname = "../data/dimacs/USA-road-d." + fname_suffix + ".gr";
  std::string time_data_fname = "../data/dimacs/USA-road-t." + fname_suffix + ".gr";

  {
    int status = rzq::basic::ReadRoadmapFromFile(dist_data_fname, time_data_fname, 3, &g);
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
  long vd = 200000; // destination node id in the graph.
  double time_limit = 60; // e.g. one minute search time limit.

  rzq::search::EMOAResult res;
  rzq::search::RunEMOA(&g, vo, vd, time_limit, &res);

  // print paths, times and costs
  std::cout << "---- reprint solutions for more clarity:" << std::endl;
  for (auto iter : res.paths) {
    long k = iter.first; // id of a Pareto-optipmal solution
    // path nodes
    std::cout << " path nodes = ";
    for (auto xx : res.paths[k]) {
      std::cout << xx << ", ";
    }
    std::cout << std::endl;
    // cost
    std::cout << " cost = " << res.costs[k] << std::endl;
    std::cout << " nDomCheck = " << res.n_domCheck << std::endl;
    std::cout << " rtSearch = " << res.rt_search << std::endl;
  }
  
  return 1;

};

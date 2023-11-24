
/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/

#include "search_emoa.hpp"
#include "graph_io.hpp"
#include <iostream>
#include <string>


int TestRoadmapToy();

int main(){
  TestRoadmapToy();
  return 0;
};

int TestRoadmapToy() {
  rzq::basic::SparseGraph g;
  // g.Init(4, 2);

  std::vector<double> cv;
  cv.resize(3);
  cv[0] = 1;
  cv[1] = 10;
  cv[2] = 100;
  g.AddEdge(1, 2, cv);
  g.AddEdge(2, 1, cv);

  cv[0] = 9;
  cv[1] = 4;
  g.AddEdge(1, 3, cv);
  g.AddEdge(3, 1, cv);

  cv[0] = 5;
  cv[1] = 5;
  g.AddEdge(2, 3, cv);
  g.AddEdge(3, 2, cv);

  cv[0] = 10;
  cv[1] = 3;
  g.AddEdge(2, 4, cv);
  g.AddEdge(4, 2, cv);

  cv[0] = 4;
  cv[1] = 10;
  g.AddEdge(3, 4, cv);
  g.AddEdge(4, 3, cv);

  // do some print and verification to make sure the generated graph is correct.

  std::cout << "num_nodes: " << g.NumVertex() << std::endl;
  std::cout << "num_edges: " << g.NumEdge() << std::endl;
  std::cout << "cdims: " << g.CostDim() << std::endl;
  std::cout << "g.GetCost(1, 2): " << g.GetCost(1, 2) << std::endl;
  std::cout << "g.GetCost(1, 3): " << g.GetCost(1, 3) << std::endl;

  // std::vector<long> tempPath({1,2,4});
  // std::cout << "--get path cost :" << GetPathCostInGraph(&g, tempPath) ;

  // ######################################### //
  // ####### Test 2 - run planner ######### //
  // ######################################### //

  long vo = 1; // starting node in the graph.
  long vd = 4; // destination node id in the graph.
  double time_limit = 60; // e.g. one minute search time limit.

  rzq::search::EMOAResult res;
  RunEMOA(&g, vo, vd, time_limit, &res);

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
  }
  return 1;

};

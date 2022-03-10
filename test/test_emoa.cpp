
/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/

#include "api.hpp"
#include "graph_io.hpp"
#include <iostream>
#include <string>

int TestGridToy();
int TestRoadmapToy();

int main(){
  TestGridToy();
  TestRoadmapToy();
  return 0;
};


int TestGridToy() {

  // static environment
  rzq::basic::Grid static_world; // static obstacles appears in this 2d grid.
  int r = 3; // rows (y)
  int c = 3; // columns (x)
  static_world.Resize(r,c);
  static_world.Set(1,1,1); // set grid[y=1,x=1] = 1, a static obstacle.

  // declare cost structure.
  rzq::basic::Grid time_cost;
  time_cost.Resize(r,c,1); // each action takes unit time.
  time_cost.Set(2,1,3);
  rzq::basic::Grid comm_cost; // communication quality
  comm_cost.Resize(r,c,1); // the communication quality at each location is by default 2.
  comm_cost.Set(1,2,3);
  std::vector<rzq::basic::Grid> cost_grids; // cost vectors (implemented at nodes rather than edges), arrival cost at a node.
  cost_grids.push_back(time_cost); // cost_grids[0] = traversal time cost,
  cost_grids.push_back(comm_cost); // cost_grids[1] = communication quality cost, ...

  // workspace graph (static)
  rzq::basic::GridkConn g; // this is the graph (impl as a grid) that represents the workspace
  g.Init(static_world, cost_grids);

  long vo = 0; // start node id.
  long vd = 8; // goal node id.
  double time_limit = 60; // seconds

  rzq::search::EMOAResult res;
  rzq::search::RunEMOA(&g, vo, vd, time_limit, &res);

  // print paths, times and costs
  std::cout << "----EMOA* reprint solutions for more clarity:" << std::endl;
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
  }

  return 1;

};

int TestRoadmapToy() {
  rzq::basic::Roadmap g;
  g.Init(4, 2);

  rzq::basic::CostVector cv = rzq::basic::CostVector(0, 2);
  cv[0] = 1;
  cv[1] = 10;
  g.AddEdge(1, 2, cv);
  g.AddEdge(2, 1, cv);

  cv = rzq::basic::CostVector(0, 2);
  cv[0] = 9;
  cv[1] = 4;
  g.AddEdge(1, 3, cv);
  g.AddEdge(3, 3, cv);

  cv = rzq::basic::CostVector(0, 2);
  cv[0] = 5;
  cv[1] = 5;
  g.AddEdge(2, 3, cv);
  g.AddEdge(3, 2, cv);

  cv = rzq::basic::CostVector(0, 2);
  cv[0] = 10;
  cv[1] = 3;
  g.AddEdge(2, 4, cv);
  g.AddEdge(4, 2, cv);

  cv = rzq::basic::CostVector(0, 2);
  cv[0] = 4;
  cv[1] = 10;
  g.AddEdge(3, 4, cv);
  g.AddEdge(4, 3, cv);

  // do some print and verification to make sure the generated graph is correct.

  std::cout << "num_nodes: " << g.GetNumberOfNodes() << std::endl;
  std::cout << "num_edges: " << g.GetNumberOfEdges() << std::endl;
  std::cout << "cdims: " << g.GetCostDim() << std::endl;
  std::cout << "g.GetCost(1, 2): " << g.GetCost(1, 2) << std::endl;
  std::cout << "g.GetCost(1, 3): " << g.GetCost(1, 3) << std::endl;

  // ######################################### //
  // ####### Test 2 - run planner ######### //
  // ######################################### //

  long vo = 1; // starting node in the graph.
  long vd = 4; // destination node id in the graph.
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
  }
  return 1;

};

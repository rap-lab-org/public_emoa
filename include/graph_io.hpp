
#ifndef ZHONGQIANGREN_BASIC_GRAPH_IO_H_
#define ZHONGQIANGREN_BASIC_GRAPH_IO_H_

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <iostream>

#include "graph.hpp"
#include "emoa.hpp"

namespace rzq{
namespace basic{


int ReadRoadmapFromFile(std::string dist_data_fname, std::string time_data_fname, int cost_dim, Roadmap* out) ;

int SaveResultToFile(std::string fname, double time, search::EMOAResult* res);


} // end namespace basic
} // end namespace rzq


#endif  // ZHONGQIANGREN_BASIC_GRAPH_IO_H_

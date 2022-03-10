
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>

#include "graph.hpp"
#include "emoa.hpp"

namespace rzq{
namespace basic{


int ReadRoadmapFromFile(std::string dist_data_fname, std::string time_data_fname, int cost_dim, Roadmap* out) {
	if ((cost_dim != 2) && (cost_dim != 3)) {
		std::cout << "[ERROR] ReadRoadmapFromFile input cost_dim = " << cost_dim << std::endl;
	  throw std::runtime_error("[ERROR] ReadRoadmapFromFile, cost_dim must be 2 or 3 for benchmark maps.");
	}

  std::cout << "Running read_graph_file(): " << std::endl;
	std::cout << "dist_data_path: " << dist_data_fname << std::endl;
	std::cout << "time_data_path: " << time_data_fname << std::endl;

	// open dist/time files with error checking
	std::ifstream f_dist(dist_data_fname);
  if (!f_dist) {
    std::cerr << "Error: file '" << dist_data_fname << "' could not be opened" << std::endl;
		return -1;
  }
	std::ifstream f_time(time_data_fname);
  if (!f_time) {
    std::cerr << "Error: file '" << time_data_fname << "' could not be opened" << std::endl;
		return -1;
  }

	std::string line_dist;
	std::string line_time;
	std::vector<std::string> words;
	int count = 0;
  int num_nodes = -1;
  int num_edges = -1;
	while (std::getline(f_dist, line_dist)) {
		std::getline(f_time, line_time);
		if (line_dist[0] == 'p') {
			int index = line_dist.find_first_of(" ", 5);
			num_nodes = stoi(line_dist.substr(5, index));
			num_edges = stoi(line_dist.substr(index + 1));
      std::cout << "p:num_nodes: " << num_nodes << std::endl;
      std::cout << "p:num_edges: " << num_edges << std::endl;
      out->Init(num_nodes, cost_dim);
		} else if (line_dist[0] == 'a') {
			int index1 = line_dist.find_first_of(" ", 2);
			int index2 = line_dist.find_first_of(" ", index1 + 1);
			int u = stoi(line_dist.substr(2, index1));
			int v = stoi(line_dist.substr(index1 + 1, index2));
			int w_dist = stoi(line_dist.substr(index2 + 1));
			int w_time = stoi(line_time.substr(line_time.find_last_of(" ") + 1));

			if ((w_time <= 0) || (w_dist <= 0)) {
			  throw std::runtime_error("[ERROR] ReadRoadmapFromFile, input graph has negative cost !?");
			}

      CostVector cv = basic::CostVector(0, cost_dim);
      cv[0] = w_dist;
      cv[1] = w_time;
			if (cost_dim == 3) {
				cv[2] = 0; // set afterwards
			}
			// todo: assert it is positive
			// if (!check_positive_vec(cost_vec)) {
			// 	std::cout << "Negative cost vec:" << endl;
			// 	print_cost_vec(cost_vec);
			// 	assert(false);
			// }

      out->AddEdge(u, v, cv);
		}
	}

	if (cost_dim == 3) {
		out->AddDegreeCost(2);
	}

  return 1; // true, succeed.
};

int SaveResultToFile(std::string fname, double time, search::EMOAResult* res) {
  std::ofstream fout;

  fout.open(fname);
  if (!fout) {
    std::cerr << "Error: file '" << fname << "' could not be opened" << std::endl;
		return -1;
  }

  fout << "EMOAResults: " << std::endl;
	fout << "time: " << time << std::endl;
  fout << "n_generated: " << res->n_generated << std::endl;
  fout << "n_expanded: " << res->n_expanded << std::endl;
  fout << "n_domCheck: " << res->n_domCheck << std::endl;
  fout << "rt_initHeu: " << res->rt_initHeu << std::endl;
  fout << "rt_search: " << res->rt_search << std::endl;
	fout << "N: " << res->costs.size() << std::endl;
	int index = 0;
  for (const auto& kv: res->costs) {
		fout << "Label: " << kv.first << std::endl;

    fout << kv.second.ToStr() << std::endl;

		std::vector<long> path = res->paths[kv.first];
    for (int i = 0; i < path.size(); i++) {
      fout << path[i] << " ";
    }
    fout << std::endl;

		index += 1;
  }

  return 1;
}


} // end namespace basic
} // end namespace rzq

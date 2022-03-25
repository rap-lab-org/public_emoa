
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


int ReadRoadmapFromFile(std::vector<std::string> edge_cost_fnames, bool add_degree_cost, Roadmap* out) {
	// if ((cost_dim != 2) && (cost_dim != 3)) {
	// 	std::cout << "[ERROR] ReadRoadmapFromFile input cost_dim = " << cost_dim << std::endl;
	//   throw std::runtime_error("[ERROR] ReadRoadmapFromFile, cost_dim must be 2 or 3 for benchmark maps.");
	// }

  std::cout << "Running read_graph_file(): " << std::endl;
	for (int i = 0; i < edge_cost_fnames.size(); i++) {
		std::cout << "cost_fname " << i << ": " << edge_cost_fnames[i] << std::endl;
	}

	std::ifstream f_costs[100]; // todo: not very elegant
	for (int i = 0; i < edge_cost_fnames.size(); i++) {
		f_costs[i].open(edge_cost_fnames[i]);
		if (!f_costs[i]) {
			std::cerr << "Error: file '" << edge_cost_fnames[i] << "' could not be opened" << std::endl;
			return -1;
		}
	}

	int cost_dim = edge_cost_fnames.size();
  if (add_degree_cost) {
    cost_dim++;
  }

	std::string line;
  int num_nodes = -1;
  int num_edges = -1;
	while (std::getline(f_costs[0], line)) {
		std::vector<std::string> lines;
		lines.push_back(line);
		for (int i = 1; i < edge_cost_fnames.size(); i++) {
			std::getline(f_costs[i], line);
			lines.push_back(line);
		}
		line = lines[0];
		if (line[0] == 'p') {
			int index = line.find_first_of(" ", 5);
			num_nodes = stoi(line.substr(5, index));
			num_edges = stoi(line.substr(index + 1));
      std::cout << "p:num_nodes: " << num_nodes << std::endl;
      std::cout << "p:num_edges: " << num_edges << std::endl;
      out->Init(num_nodes, cost_dim);
		} else if (line[0] == 'a') {
			// find u, v
			int index1 = line.find_first_of(" ", 2);
			int index2 = line.find_first_of(" ", index1 + 1);
			int u = stoi(line.substr(2, index1));
			int v = stoi(line.substr(index1 + 1, index2));
			// find costs
			CostVector cv = basic::CostVector(0, cost_dim);
			for (int i = 0; i < edge_cost_fnames.size(); i++) {
				cv[i] = stoi(lines[i].substr(lines[i].find_last_of(" ") + 1));
				if (cv[i] < 0) {
					throw std::runtime_error("[ERROR] ReadRoadmapFromFile, input graph has negative cost !?");
				}
			}
			// todo: extra cost dim
			if (add_degree_cost) {
				cv[cost_dim - 1] = 0; // set afterwards
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

	if (add_degree_cost) {
		out->AddDegreeCost(cost_dim - 1);
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

	fout << "graph_load_time: " << time << std::endl;
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

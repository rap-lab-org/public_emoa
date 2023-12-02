
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>

#include "graph.hpp"
#include "vec_type.hpp"
// #include "emoa.hpp"

namespace rzq{
namespace basic{

int LoadSparseGraphDIMAC(std::vector<std::string> edge_cost_fnames, SparseGraph* out) {

	std::cout << "[INFO] LoadSparseGraphDIMAC starts " << std::endl;
	for (int i = 0; i < edge_cost_fnames.size(); i++) {
		std::cout << "[INFO] --- cost_fname " << i << ": " << edge_cost_fnames[i] << std::endl;
	}

	std::ifstream f_costs[100]; // todo: not very elegant, exceeding 100?
	for (int i = 0; i < edge_cost_fnames.size(); i++) {
		f_costs[i].open(edge_cost_fnames[i]);
		if (!f_costs[i]) {
			std::cerr << "[Error] file '" << edge_cost_fnames[i] << "' could not be opened" << std::endl;
			return -1;
		}
	}

	int cost_dim = edge_cost_fnames.size();

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
			std::cout << "[INFO] num_nodes: " << num_nodes << std::endl;
			std::cout << "[INFO] num_edges: " << num_edges << std::endl;
			// out->Init(num_nodes, cost_dim);
		} else if (line[0] == 'a') {
			// find u, v
			int index1 = line.find_first_of(" ", 2);
			int index2 = line.find_first_of(" ", index1 + 1);
			long u = stol(line.substr(2, index1));
			long v = stol(line.substr(index1 + 1, index2));
			// find costs
			auto cv = InitVecType(cost_dim, 0.0);
			// CostVector cv = basic::CostVector(0, cost_dim);
			for (int i = 0; i < edge_cost_fnames.size(); i++) {
				cv[i] = stod(lines[i].substr(lines[i].find_last_of(" ") + 1));
				if (cv[i] < 0) {
					throw std::runtime_error("[ERROR] LoadSparseGraphDIMAC, input graph has negative cost !?");
				}
			}

			// todo: assert it is positive
			// if (!check_positive_vec(cost_vec)) {
			// 	std::cout << "Negative cost vec:" << endl;
			// 	print_cost_vec(cost_vec);
			// 	assert(false);
			// }

			out->AddArc(u, v, cv);
		}
	}
	
	std::cout << "[INFO] LoadSparseGraphDIMAC ends " << std::endl;

	return 1; // true, succeed.
};

int LoadStartGoal(std::string fname, std::vector<long>* sources, std::vector<long>* goals) {
	std::ifstream fin;
	fin.open(fname);
	if (!fin) {
		std::cerr << "[Error] file '" << fname << "' could not be opened" << std::endl;
		return -1;
	}

	int N;
	fin >> N;
	long a, b;
	for (int i = 0; i < N; i++) {
		fin >> a >> b;
		sources->push_back(a);
		goals->push_back(b);
	}

	return 1;
}


int LoadCoordDIMAC(std::string coord_file, std::vector< std::vector<double> >* output) {
	
	std::cout << "[INFO] LoadCoordDIMAC starts " << std::endl;
	std::cout << "[INFO] --- coord_fname " << coord_file << std::endl;
	std::ifstream fcoord;
	fcoord.open(coord_file);
	if (!fcoord) {
		std::cerr << "[Error] file '" << coord_file << "' could not be opened" << std::endl;
		return -1;
	}

	std::string line;
	long n_nodes = 0;
	while (std::getline(fcoord, line)) {
		if (line[0]=='p') {
			int idx = line.find_first_of(" ", 10);
			n_nodes = stol(line.substr(idx+1, line.size()-1));
			break;
		}
	}

	output->resize(n_nodes + 1);
	(*output)[0] = {-1, -1};
	while (std::getline(fcoord, line)) {
		if (line[0]=='v') {
			int index1 = line.find_first_of(" ", 0);
			int index2 = line.find_first_of(" ", index1+1);
			int index3 = line.find_first_of(" ", index2+1);
			// find index of coordinates
			int idv = stol(line.substr(index1+1, index2));
			// find coordinates
			double x = stol(line.substr(index2+1, index3));
			double y = stol(line.substr(index3+1, line.size()-1));
			// std::cout << idv << " " << x << " " << y << std::endl;
			
			(*output)[idv] = {x, y};
		}
	}

	return 1;
};


} // end namespace basic
} // end namespace rzq

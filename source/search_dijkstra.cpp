
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "search_dijkstra.hpp"
#include <set>
#include <limits>

namespace rzq{
namespace search{

using namespace basic;

Dijkstra::Dijkstra() {

};

Dijkstra::~Dijkstra() {

};

std::vector<long> Dijkstra::PathFinding(long vs, long vg, double time_limit, short cdim) {
	_mode = 0; // start-goal path finding
	_cdim = cdim;
	_vs = vs;
	_vg = vg;
  _time_limit = time_limit;
	_search();
	return GetPath(vg);
};

int Dijkstra::ExhaustiveBackwards(long vg, double time_limit, short cdim) {
	_mode = 1; // 1 = exhaustive backwards, 
	_cdim = cdim;
	_vs = vg; // search from vs but use get predecessor for expansion.
  _time_limit = time_limit;
	_search();
	return 1;
};

int Dijkstra::ExhaustiveForwards(long vs, double time_limit, short cdim) {
	_mode = 2; // 2 = exhaustive forwards.
	_cdim = cdim;
	_vs = vs;
  _time_limit = time_limit;
	_search();
	return 1;
};

int Dijkstra::_search() {
  if (DEBUG_DIJKSTRA){ std::cout << "[DEBUG] Dijkstra::_search, _vs = " << _vs << " _vg = " << _vg << " _mode = " << _mode << " _time_limit = " << _time_limit << std::endl; }
  if ( !(_graph->HasVertex(_vs)) ) {
    // failed, input src does not exists in the graph.
    std::cout << "[ERROR] Dijkstra, input v_start " << _vs << " does not exist !!" << std::endl;
    throw std::runtime_error( "[ERROR] Dijkstra, input v_start does not exist !!" );
    return -1;
  } 
  if ( (_mode == 0 ) && !(_graph->HasVertex(_vg)) ) {
    // failed, input goal does not exists in the graph.
    std::cout << "[ERROR] Dijkstra, input v_goal " << _vg << " does not exist !!" << std::endl;
    throw std::runtime_error( "[ERROR] Dijkstra, input v_goal does not exist !!" );
    return -2;
  } 
  auto tstart = std::chrono::steady_clock::now();

  // DataType is defined in cost_vector.hpp, which is by default double.
  std::set< std::pair<double, long> > open; 
    // open list, leverage property of set, which is sorted by the first value in pair.

  // init search
  size_t nV = _graph->NumVertex();
  _v2d.clear();
  _v2d.resize(nV, std::numeric_limits<double>::infinity() );
  _parent.clear();
  _parent.resize(nV, -1);
  _cvec.clear();
  _cvec.resize(nV);
  open.insert(std::make_pair(0.0, _vs));
  _v2d[_vs] = 0.0;
  _cvec[_vs].resize(_graph->CostDim(), 0); // the corresponding cost vector of the path.

  // main search loop
  while(!open.empty()){

    // check time limit
    auto tnow = std::chrono::steady_clock::now();
    if ( std::chrono::duration<double>(tnow-tstart).count() > _time_limit) {
      break; // fails. timeout.
    }

    // extract from OPEN
    std::pair<double, long> curr_pair = *(open.begin());
    open.erase(open.begin());
    auto v = curr_pair.second;
    if (DEBUG_DIJKSTRA){ std::cout << "[DEBUG] Dijkstra::_search, popped v = " << v << " g = " << curr_pair.first << std::endl; }
    if (curr_pair.first > _v2d[v]) { // this candidate is outdated.
    	// std::cout << "[INFO] Dijkstra::_search, outdated vertex exists" << std::endl;
    	continue;
    }

    // check goal
    if (_mode == 0 && v == _vg) {
    	break; // termination
    }

    // expansion
    std::vector<long> nghs;
    std::vector< std::vector<double> > ngh_costs;
    if (_mode == 1) { 
      nghs = _graph->GetPreds(v);
      ngh_costs = _graph->GetPredCosts(v);
    }else if (_mode == 0 || _mode == 2){
      nghs = _graph->GetSuccs(v);
      ngh_costs = _graph->GetSuccCosts(v);
    }else{
      std::cout << "[ERROR] Dijkstra::_search, _mode = " << _mode << std::endl;
      throw std::runtime_error( "[ERROR] Dijkstra::_search, unknown _mode !!" );
    }
    for (size_t j = 0; j < nghs.size(); j++) {
      // generation
      long u = nghs[j];
      auto cvec = ngh_costs[j];
      auto c = cvec[_cdim];
      if (c < 0){
        // negative edge !! Input graph has ERROR!!
        std::cout << "[ERROR] v = " << v << " u = " << u << " cost_vec(u,v) = " << cvec << " c = " << c << std::endl;
        throw std::runtime_error( "[ERROR] Dijkstra::_search, encounter negative edge costs !!" );
        _v2d.clear();
        return -3; // failed
      } // end if
      auto dist_u = curr_pair.first + c;
      auto cvec_u = _cvec[v] + cvec; // the corresponding cost vector of the path to vd.
      if (DEBUG_DIJKSTRA){ std::cout << "[DEBUG] Dijkstra::_search, generate u = " << u << " g = " << dist_u << std::endl; }

      // pruning and add open for non-pruned successors
      if ( dist_u < _v2d[u] ){
        _v2d[u] = dist_u;
        _cvec[u] = cvec_u;
        auto temp_pair = std::make_pair(dist_u,u);
        _parent[u] = v;
        open.insert(temp_pair); // re-insert
      } // end if 
    }// end for
  }// end while
  
  return 1;
};

std::vector<long> Dijkstra::GetPath(long v, bool do_reverse) {
  std::vector<long> out;
  out.push_back(v);
  if (_parent.size() == 0){
    return std::vector<long>() ;
  }
  while( _parent[v] != -1 ) {
    out.push_back(_parent[v]);
    v = _parent[v];
  }

  if (do_reverse) {
    std::vector<long> path;
    for (size_t i = 0; i < out.size(); i++) {
      path.push_back(out[out.size()-1-i]);
    }
    return path;
  }else{
    return out;
  }
};

std::vector<double> Dijkstra::GetDistAll() {
  return _v2d;
};

double Dijkstra::GetDistValue(long v) {
  return _v2d[v];
};

std::vector<double> Dijkstra::GetSolutionCost() {
  return _cvec[_vg];
} ;

std::vector<double> Dijkstra::GetPathCost(long v) {
  return _cvec[v];
};



} // end namespace basic
} // end namespace zr

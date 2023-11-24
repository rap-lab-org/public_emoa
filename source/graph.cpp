  
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "graph.hpp"
#include "vec_type.hpp"

namespace rzq{
namespace basic{


// ############################################################
// ############################################################
// ############################################################

SparseGraph::SparseGraph() {};

SparseGraph::~SparseGraph() {};

bool SparseGraph::HasVertex(long v) {
  return v < _to.size();
};

bool SparseGraph::HasArc(long v, long u) {
  for (auto k : _to[v]){
    if (k == u) {return true;}
  }
  return false;
};

std::vector<long> SparseGraph::GetSuccs(long v) {
  if (!HasVertex(v)) {return std::vector<long>(); }
  return _to[v];
};

std::vector<long> SparseGraph::GetPreds(long v) {
  if (!HasVertex(v)) {return std::vector<long>(); }
  return _from[v];
};

std::vector<double> SparseGraph::GetCost(long u, long v) {
  for (size_t idx = 0; idx < _to_cost[u].size(); idx++){
    if (_to[u][idx] == v) {
      return _to_cost[u][idx];
    }
  }
  return std::vector<double>();
};

std::vector<std::vector<double>> SparseGraph::GetSuccCosts(long u) {
  if (!HasVertex(u)) {return std::vector<std::vector<double>>(); }
  return _to_cost[u];
};

std::vector<std::vector<double>> SparseGraph::GetPredCosts(long u) {
  if (!HasVertex(u)) {return std::vector<std::vector<double>>(); }
  return _from_cost[u];
};

size_t SparseGraph::NumVertex() {
  return _to.size();
};

size_t SparseGraph::NumArc() {
  return _n_arc;
};

size_t SparseGraph::NumEdge() {
  if (_n_arc % 2 != 0) {
    std::cout << "[ERROR] SparseGraph::NumEdge is not an integer but a fraction" << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::NumEdge is not an integer but a fraction");
  }
  return size_t(_n_arc / 2);
};

size_t SparseGraph::CostDim() {
  return _cdim ;
};

std::vector<long> SparseGraph::AllVertex() 
{
  std::vector<long> out;
  for (int i = 0; i < _to.size(); i++){
    out.push_back(i);
  }
  return out;
};

void SparseGraph::AddVertex(long v) {
  if (!HasVertex(v)) {
    _to.resize(v+1);
    _to_cost.resize(v+1);
    _from.resize(v+1);
    _from_cost.resize(v+1);
  }
  return;
};

void SparseGraph::AddEdge(long u, long v, std::vector<double> c) {
  AddArc(u,v,c);
  AddArc(v,u,c);
  return ;
};

void SparseGraph::AddArc(long u, long v, std::vector<double> c) {
  if (!HasVertex(u)) {
    AddVertex(u);
  }
  if (!HasVertex(v)) {
    AddVertex(v);
  }

  if (_cdim == 0) {
    _cdim = c.size();
  }else{
    if (_cdim != c.size()) {
      std::cout << "[ERROR] SparseGraph::AddArc cdim does not match: " << _cdim << " != " << c.size() << std::endl;
      throw std::runtime_error("[ERROR] SparseGraph::AddArc cdim does not match");
    }
  }

  bool updated = false;
  for (size_t idx = 0; idx < _to[u].size(); idx++){
    if (_to[u][idx] == v) {
      _to_cost[u][idx] = c;
      updated = true;
      break;
    }
  }
  if (!updated) {
    _to[u].push_back(v);
    _to_cost[u].push_back(c);
    _n_arc++;
  }

  updated = false;
  for (size_t idx = 0; idx < _from[v].size(); idx++){
    if (_from[v][idx] == u) {
      _from_cost[v][idx] = c;
      updated = true;
      break;
    }
  }
  if (!updated) {
    _from[v].push_back(u);
    _from_cost[v].push_back(c);
    _n_arc++;
  }
  return ;
};

void SparseGraph::CreateFromEdges(std::vector<long> sources, 
    std::vector<long> targets, std::vector<std::vector<double>> costs)
{
  _to.clear();
  _to_cost.clear();
  _from.clear();
  _from_cost.clear();
  long max_id = 0;
  for (size_t i = 0; i < sources.size(); i++){
    if (sources[i] > max_id) { max_id = sources[i]; }
    if (targets[i] > max_id) { max_id = targets[i]; }
  }
  _to.resize(max_id+1);
  _to_cost.resize(max_id+1);
  _from.resize(max_id+1);
  _from_cost.resize(max_id+1);
  for (size_t i = 0; i < sources.size(); i++) {
    _to[sources[i]].push_back(targets[i]);
    _to_cost[sources[i]].push_back(costs[i]);
    _from[targets[i]].push_back(sources[i]);
    _from_cost[targets[i]].push_back(costs[i]);
    _to[targets[i]].push_back(sources[i]);
    _to_cost[targets[i]].push_back(costs[i]);
    _from[sources[i]].push_back(targets[i]);
    _from_cost[sources[i]].push_back(costs[i]);
  }
  return ;
};

void SparseGraph::CreateFromArcs(std::vector<long> sources, 
    std::vector<long> targets, std::vector<std::vector<double>> costs)
{
  _to.clear();
  _to_cost.clear();
  _from.clear();
  _from_cost.clear();
  long max_id = 0;
  for (size_t i = 0; i < sources.size(); i++){
    if (sources[i] > max_id) { max_id = sources[i]; }
    if (targets[i] > max_id) { max_id = targets[i]; }
  }
  _to.resize(max_id+1);
  _to_cost.resize(max_id+1);
  _from.resize(max_id+1);
  _from_cost.resize(max_id+1);
  for (size_t i = 0; i < sources.size(); i++) {
    _to[sources[i]].push_back(targets[i]);
    _to_cost[sources[i]].push_back(costs[i]);
    _from[targets[i]].push_back(sources[i]);
    _from_cost[targets[i]].push_back(costs[i]);
  }
  return ;
};

void SparseGraph::ChangeCostDim(size_t new_cdim, double default_value) {
  for (size_t i = 0; i < _to.size(); i++) {
    for (size_t j = 0; j < _to[i].size(); j++) {
      _to_cost[i][j].resize(new_cdim, default_value);
    }
  }
  for (size_t i = 0; i < _from.size(); i++) {
    for (size_t j = 0; j < _from[i].size(); j++) {
      _from_cost[i][j].resize(new_cdim, default_value);
    }
  }
  _cdim = new_cdim;
  return ;
};

bool SparseGraph::SetArcCost(long u, long v, const std::vector<double>& new_cost) {
  if (new_cost.size() != _cdim){
    std::cout << "[ERROR] SparseGraph::SetArcCost new edge cost dimension does not match! Maybe call ChangeCostDim() at first." << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::SetArcCost new edge cost dimension does not match! Maybe call ChangeCostDim() at first.") ;
    return false;    
  }
  if ( (!HasVertex(u)) || (!HasVertex(v)) ) {
    std::cout << "[ERROR] SparseGraph::SetArcCost vertex does not exist!" << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::SetArcCost vertex does not exist!") ;
    return false;
  }
  if ( new_cost.size() != _cdim ) {
    std::cout << "[ERROR] SparseGraph::SetArcCost input vector size mismatch : " << new_cost.size() << " vs " << _cdim << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::SetArcCost input vector size mismatch!") ;
    return false;
  }

  bool found = false;
  for (int i = 0; i < _to[u].size(); i++){
    if (_to[u][i] == v) {
      _to_cost[u][i] = new_cost;
      found = true;
      break;
    }
  }
  if (!found) {
    std::cout << "[ERROR] SparseGraph::SetArcCost arc does not exist!" << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::SetArcCost arc does not exist!") ;
  }

  found = false;
  for (int i = 0; i < _from[v].size(); i++){
    if (_from[v][i] == u) {
      _from_cost[v][i] = new_cost;
      found = true;
      break;
    }
  }
  if (!found) {
    std::cout << "[ERROR] SparseGraph::SetArcCost arc does not exist!" << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::SetArcCost arc does not exist!") ;
  }
  return true;
};

std::string SparseGraph::ToStr() const {
  std::string out;
  out += "=== SparseGraph Begin ===\n |V| = " + std::to_string(_to.size()) + " outgoing edges \n";
  for (long v = 0; v < _to.size(); v++) {
    out += " -- " + std::to_string(v) + ":[";
    for (size_t idy = 0; idy < _to[v].size(); idy++){
      out += std::to_string(_to[v][idy]) + "(" + ToString(_to_cost[v][idy]) + "),";
    }
    out += "]\n";
  }
  out += " incoming edges \n";
  for (long v = 0; v < _to.size(); v++) {
    out += " -- " + std::to_string(v) + ":[";
    for (size_t idy = 0; idy < _from[v].size(); idy++){
      out += std::to_string(_from[v][idy]) + "(" + ToString(_from_cost[v][idy]) + "),";
    }
    out += "]\n";
  }
  out += "=== SparseGraph End ===";
  return out;
};

std::ostream& operator<<(std::ostream& os, const SparseGraph& c) {
  os << c.ToStr();
  return os;
};

// ############################################################
// ############################################################
// ############################################################

Grid2d::Grid2d() {};

Grid2d::~Grid2d() {};

bool Grid2d::HasVertex(long v)
{
  if ( (_k2r(v) < _occu_grid_ptr->size()) && (_k2c(v) < _occu_grid_ptr->at(0).size()) ){
    return true;
  }
  return false;
};

bool Grid2d::HasArc(long v, long u) {
  // newly added on @2023-11
  // TODO, run some test.
  long r1 = _k2r(v);
  long c1 = _k2c(v);
  long r2 = _k2r(u);
  long c2 = _k2c(u);
  for (int i = 0; i < _act_r.size(); i++){
    bool b1 = (r2 == (r1+_act_r[i]));
    bool b2 = (c2 == (c1+_act_c[i]));
    if (b1 && b2) {return true;}
  }
  return false;
};

std::vector<long> Grid2d::GetSuccs(long v)
{
  std::vector<long> out;
  long r = _k2r(v);
  long c = _k2c(v);
  for (size_t idx = 0; idx < _kngh; idx++) {
    long nr = r+_act_r[idx];
    if (nr >= _occu_grid_ptr->size()) {continue;}
    long nc = c+_act_c[idx];
    if (nc >= _occu_grid_ptr->at(0).size()) {continue;}
    if (_occu_grid_ptr->at(nr).at(nc) > 0) {continue;} // obstacle
    out.push_back(_rc2k(nr,nc));
  }
  return out;
};

std::vector<long> Grid2d::GetPreds(long v)
{
  return GetSuccs(v);
};
  

std::vector<double> Grid2d::GetCost(long u, long v)
{
  // v is the target vertex of arc (u,v)
  std::vector<double> out;
  long r = _k2r(v);
  long c = _k2c(v);
  long r2 = _k2r(v);
  long c2 = _k2c(v);
  if ((r != r2) && (c != c2)) {
    out.push_back(1.4);
    return out;
  }else{
    out.push_back(1.0);
    return out;
  }
};

std::vector<std::vector<double>> Grid2d::GetSuccCosts(long u)
{
  std::vector<std::vector<double>> out;
  long r = _k2r(u);
  long c = _k2c(u);
  for (size_t idx = 0; idx < _kngh; idx++) {
    long nr = r+_act_r[idx];
    if (nr >= _occu_grid_ptr->size()) {continue;}
    long nc = c+_act_c[idx];
    if (nc >= _occu_grid_ptr->at(0).size()) {continue;}
    if (_occu_grid_ptr->at(nr).at(nc) > 0) {continue;} // obstacle
    std::vector<double> c;
    if (idx <= 3) { 
      c.push_back(1);
    }
    else if (idx <= 7) {
      c.push_back(1.4);
    }
    else { throw std::runtime_error( "[ERROR], Grid2d _kngh > 8, not supported!" ); }
    out.push_back(c);
  }
  return out;
};

std::vector<std::vector<double>> Grid2d::GetPredCosts(long u) {
  return GetSuccCosts(u);
};

size_t Grid2d::NumVertex() {
  if (_occu_grid_ptr->size() == 0) {return 0;}
  return _occu_grid_ptr->size() * _occu_grid_ptr->at(0).size() ;
} ;

size_t Grid2d::NumArc() {
  std::cout << "[ERROR], Grid2d::NumArc not implemented, TODO." << std::endl;
  throw std::runtime_error( "[ERROR], Grid2d::NumArc not implemented, TODO." );
  return 0;
};

size_t Grid2d::NumEdge() {
  std::cout << "[ERROR], Grid2d::NumArc not implemented, TODO." << std::endl;
  throw std::runtime_error( "[ERROR], Grid2d::NumArc not implemented, TODO." );
  return 0;
}

size_t Grid2d::CostDim() {
  return 1;
};

std::vector<long> Grid2d::AllVertex() 
{
  std::cout << "[ERROR], Grid2d::AllVertex not implemented, TODO." << std::endl;
  throw std::runtime_error( "[ERROR], Grid2d::AllVertex not implemented, TODO." );

  std::vector<long> out;
  return out;
};

////////////

void Grid2d::SetOccuGridPtr(std::vector< std::vector<double> >* in)
{
  _occu_grid_ptr = in;
  return ;
};

void Grid2d::SetOccuGridObject(std::vector< std::vector<double> >& in)
{
  _mat_from_py = in; // make a local copy. To avoid pybind issue.
  SetOccuGridPtr(&_mat_from_py);
};

bool Grid2d::SetKNeighbor(int kngh) {
  if (kngh == 4 || kngh == 8) {
    _kngh = kngh;
    if (_kngh == 4) {
      _act_r = std::vector<long>({0,0,-1,1});
      _act_c = std::vector<long>({-1,1,0,0});
    }else if (_kngh == 8) {
      _act_r = std::vector<long>({ 0, 0,-1, 1, -1,-1, 1, 1});
      _act_c = std::vector<long>({-1, 1, 0, 0, -1, 1,-1, 1});
    }
    return true;
  }
  return false;
};

long Grid2d::_rc2k(const long r, const long c) const 
{
  return r * _occu_grid_ptr->at(0).size() + c;
};

long Grid2d::_k2r(const long k) const
{
  return long(k / _occu_grid_ptr->at(0).size());
};

long Grid2d::_k2c(const long k) const 
{
  return k % _occu_grid_ptr->at(0).size();
};


} // end namespace basic
} // end namespace zr


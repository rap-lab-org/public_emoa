
/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/


#include "graph.hpp"

namespace rzq{
namespace basic{


CostVector::CostVector(){};

CostVector::CostVector(long val, size_t dim) {
  this->resize(dim);
  for (size_t i = 0; i < dim; i++){
    this->at(i) = val;
  }
  return;
};

CostVector::CostVector(const std::vector<long>& in) {
  this->resize(in.size());
  for (size_t i = 0; i < in.size(); i++){
    this->at(i) = in.at(i);
  }
  return;
};

CostVector CostVector::operator+(const CostVector& v) {
  CostVector out = *this;
  for (size_t i = 0; i < v.size(); i++) {
    out[i] += v[i];
  }
  return out;
};

CostVector& CostVector::operator+=(const CostVector& rhs) {
  for (size_t i = 0; i < rhs.size(); i++) {
    this->at(i) += rhs[i];
  }
  return *this;
}

bool CostVector::operator==(const CostVector& rhs) {
  for (size_t i = 0; i < rhs.size(); i++) {
    if (this->at(i) != rhs[i]) {
      return false;
    }
  }
  return true;
};

CostVector CostVector::operator-(const CostVector& v) {
  CostVector out = *this;
  for (size_t i = 0; i < v.size(); i++) {
    out[i] -= v[i];
  }
  return out;
};

CostVector CostVector::operator*(const long& k) {
  CostVector out = *this;
  for (size_t i = 0; i < this->size(); i++) {
    out[i] *= k;
  }
  return out;
};

CostVector CostVector::ElemWiseMin(const CostVector& rhs) {
  CostVector out = *this;
  for (size_t i = 0; i < this->size(); i++) {
    if (rhs[i] < out[i]) {
      out[i] = rhs[i];
    }
  }
  return out;
};

std::string CostVector::ToStr() const {
  std::string s = "[";
  for (auto a : (*this) ) {
    s += std::to_string(a) + ",";
  }
  s += "]";
  return s;
};

std::ostream& operator<<(std::ostream& os, const CostVector& c) {
  os << c.ToStr();
  return os;
};

/*
Return -1 if v1 < v2
Return 0 if v1 = v2
Return 1 if v1 > v2
*/
int CostVector::CompareLexico(const CostVector& v2) {
  CostVector v1 = *this;
  for (size_t i = 0; i < v1.size(); i++) {
		if (v1[i] < v2[i]) {
			return -1;
		} else if (v1[i] > v2[i]) {
			return 1;
		}
  }
	return 0;
}

// ##########

Grid::Grid() {};

Grid::~Grid() {
  // std::cout << "Grid destructor done" << std::endl;
};

void Grid::Resize(size_t r, size_t c, int val) {
  this->resize(r);
  for (auto iter = this->begin(); iter != this->end(); iter++){
    iter->resize(c, val);
  }
  return;
};

size_t Grid::GetRowNum() const {
  return this->size();
};

size_t Grid::GetColNum() const {
  if (this->size() == 0) {return 0;}
  return this->at(0).size();
};

void Grid::Set(size_t r, size_t c, int val) {
  this->at(r).at(c) = val;
};

int Grid::Get(size_t r, size_t c) {
  return this->at(r).at(c);
};

// ##########

// CvecGrid::CvecGrid() {};

// CvecGrid::~CvecGrid() {};

// ##########

GridkConn::GridkConn() {};

GridkConn::~GridkConn() {
  // std::cout << "GridkConn destructor done" << std::endl;
};

void GridkConn::Init(Grid grid, std::vector<Grid> cvecs) {
  // grid and dimensions
  _grid = grid;
  _nc = _grid.GetColNum();
  _nr = _grid.GetRowNum();
  // costs
  _cvecs = cvecs;
  _cdim = 0;
  if (cvecs.size() != 0){
    _cdim = _cvecs.size();
    // re-organize cvecs.
    _cvecs2.resize(_nr);
    for (size_t y = 0; y < _nr; y++){
      _cvecs2[y].resize(_nc);
      for (size_t x = 0; x < _nc; x++){
        _cvecs2[y][x].resize(_cdim);
      }
    }
    for (size_t y = 0; y < cvecs.begin()->size(); y++) {
      for (size_t x = 0; x < cvecs.begin()->begin()->size(); x++) {
        CostVector c;
        for (size_t k = 0; k < _cvecs.size(); k++) {
          c.push_back(_cvecs[k][x][y]);
        }
        _cvecs2[y][x] = c;
        // std::cout << " _cvecs2[" << y << "][" << x << "]=" << c[0] << "," << c[1] << std::endl;
      }
    }
  }
  // actions
  _actions.clear();
  _actions.push_back(std::vector<int>({-1,0}));
  _actions.push_back(std::vector<int>({1,0}));
  _actions.push_back(std::vector<int>({0,1}));
  _actions.push_back(std::vector<int>({0,-1}));
  return;
};

void GridkConn::SetActionSet(std::vector< std::vector<int> > actions) {
  _actions = actions;
};

bool GridkConn::HasNode(long v) {
  auto ny = y(v);
  auto nx = x(v);
  if ((ny >= 0) && (ny < _nr) && (nx >= 0) && (nx < _nc)) { // within border
    return true;
  }
  return false;
};

std::unordered_set<long> GridkConn::GetSuccs(long v) {
  std::unordered_set<long> out;
  for (auto iter = _actions.begin(); iter != _actions.end(); iter++){
    int ny = y(v)+iter->at(0);
    int nx = x(v)+iter->at(1);
    if ((ny >= 0) && (ny < _nr) && (nx >= 0) && (nx < _nc)) { // within border
      if ( _grid[ny][nx] == 0 ){ // non-obstacle
        out.insert( this->v(ny,nx) );
      }
    }
  }
  return out;
};

std::unordered_set<long> GridkConn::GetPreds(long v) {
  return GetSuccs(v); // due to symmetry.
};

CostVector GridkConn::GetCost(long u, long v) {
  return _cvecs2[y(v)][x(v)];
};

size_t GridkConn::GetCostDim() {
  return _cdim;
};

long GridkConn::v(int y, int x) {
  return x + y * _nc;
};
int GridkConn::y(long v) {
  return v / _nc;
};
int GridkConn::x(long v) {
  return v % _nc;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

/*
Details:
Nodes created by Init are 1-indexed
*/
Roadmap::Roadmap() {};

Roadmap::~Roadmap() {
  // std::cout << "Roadmap destructor done" << std::endl;
};

/*
Initializes nodes 1 to nv
*/
void Roadmap::Init(long nv, int cdim) {
  // grid and dimensions
  _adjlist = AdjList();
  _adjlist_rev = AdjList();
  _num_nodes = 0;
  _num_edges = 0;
  _cdim = cdim;

  // init nodes
  for (long i = 1; i <= nv; i++) {
    AddNode(i);
  }
  return;
};

bool Roadmap::HasNode(long v) {
  return _nodes.find(v) != _nodes.end();
};

bool Roadmap::HasEdge(long u, long v) {
  return HasNode(u) && HasNode(v) && _adjlist[u].find(v) != _adjlist[u].end();
};

std::unordered_set<long> Roadmap::GetSuccs(long v) {
  std::unordered_set<long> succs;
  for (const auto& kv: _adjlist[v]) {
    succs.insert(kv.first);
  }
  return succs;
};

std::unordered_set<long> Roadmap::GetPreds(long v) {
  std::unordered_set<long> preds;
  for (const auto& kv: _adjlist_rev[v]) {
    preds.insert(kv.first);
  }
  return preds;
};

/*

*/
CostVector Roadmap::GetCost(long u, long v) {
  return _adjlist[u][v];
};

size_t Roadmap::GetCostDim() {
  return _cdim;
};

std::unordered_set<long> Roadmap::GetNodes() {
  return _nodes;
}


void Roadmap::AddNode(long v) {
  if (!HasNode(v)) { // add new node, with empty edge list
    _num_nodes++;
    _adjlist[v] = std::unordered_map<long, CostVector >();
    _adjlist_rev[v] = std::unordered_map<long, CostVector >();
    _nodes.insert(v);
  } else {
    // bad
    std::cout << "RoadMap::AddNode() added duplicate node " << v << std::endl;
  }
  return;
}

/*
Requires: nodes v1, v2 exist in the graph
--
Adds new edge to graph
If an edge already exists between start/dest nodes,
keep the lexico-smaller CostVector
*/
void Roadmap::AddEdge(long v1, long v2, CostVector& c) {
  if (_adjlist[v1].find(v2) == _adjlist[v1].end()) {
    // new edge
    _num_edges++;
    _adjlist[v1][v2] = c;
    _adjlist_rev[v2][v1] = c;
  } else { // has existing edge
    if (c.CompareLexico(_adjlist[v1][v2]) < 0) {
      _adjlist[v1][v2] = c;
      _adjlist_rev[v2][v1] = c;
    }
  }
  return;
}

long Roadmap::GetNumberOfNodes() {
  return _num_nodes;
}

long Roadmap::GetNumberOfEdges() {
  return _num_edges;
}

/*
Adds degree cost as the 'cost_index'th element of the cost vector
Degree cost is set to 2 if degree is >= 4, 1 otherwise (used to reduce number of Pareto optimal solutions)
*/
void Roadmap::AddDegreeCost(int cost_index) {
  for (const auto& kv1: _adjlist) {
    long u = kv1.first;
    for (const auto& kv2: _adjlist[u]) {
      long v = kv2.first;
      long cost = _adjlist[u].size() + _adjlist[v].size();

      cost = cost/2; // average degree.
      if (cost < 0) {
        throw std::runtime_error("[ERROR] AddDegreeCost, negative cost !?");
      }
      long cost_value = 1;
      if (cost >= 4) {
        cost_value = 2;
      }else{
        cost_value = 1;
      }
      _adjlist[u][v][cost_index] = cost_value;
      _adjlist_rev[v][u][cost_index] = cost_value;
    }
  }
}

} // end namespace basic
} // end namespace rzq

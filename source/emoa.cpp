
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#include "emoa.hpp"
#include <set>
#include <memory>
#include <chrono>

namespace rzq{
namespace search{

#define NO_FILTER_FRONTIER false // just for experiment

long G_DOM_CHECK_COUNT = 0; // global variable, make impl easier...

std::ostream& operator<<(std::ostream& os, Label& l)
{
  std::string s;
  s = "{id:" + std::to_string(l.id) + ",v:" + std::to_string(l.v) + ",g:" 
    + l.g.ToStr() + ",f:" + l.f.ToStr() + "}";
  os << s;
  return os;
};

//////////////////////////////////////////////////////////

//################################################
// TOA* related //
//################################################

TOATree::TOATree() {
  
};

TOATree::~TOATree() {
  
};

void TOATree::Filter(basic::CostVector v) {
  int outFlag = 0;
  std::unordered_set<long> filtered_node;
  _root = _filter(_root, v, &filtered_node);
  if (filtered_node.size()>0) {
    _rebuildTree(&filtered_node);
  }
  return;
};

basic::AVLNode* TOATree::_filter(
  basic::AVLNode* n, const basic::CostVector& k, std::unordered_set<long> *filtered_node)
{
  if (n == NULL) {return n;}
  if ( _key[n->id] < k ) { // only need to look at the right sub-tree.
    n->right = _filter(n->right, k, filtered_node); 
  }else{
    n->left = _filter(n->left, k, filtered_node);
    n->right = _filter(n->right, k, filtered_node);
  }
  // no return above, will reach here if n != NULL.
  if (EpsDominance(k, _key[n->id])) {
    if (filtered_node) {
      filtered_node->insert(n->id);
    }
  }
  return n;
};

void TOATree::_rebuildTree(std::unordered_set<long> *skip_node) {
  std::vector<long> tree_node_ids;
  size_t current_size = _size;
  ToSortedVector(NULL, &tree_node_ids, skip_node);

  _size = 0;
  basic::AVLNode* new_root = _rebuildTreeMethod(tree_node_ids, 0, tree_node_ids.size()-1);
  _deleteAll(_root);
  _root = new_root;

  return ;
};

basic::AVLNode* TOATree::_rebuildTreeMethod(std::vector<long> &tree_node_ids, long start, long end) {
  if (start > end) {
    return NULL;
  }
  long mid = (start+end)/2;
  basic::AVLNode* n = basic::NewAVLNode(tree_node_ids[mid]) ;
  _size++;
  n->left  = _rebuildTreeMethod(tree_node_ids, start, mid-1);
  n->right = _rebuildTreeMethod(tree_node_ids, mid+1, end);
  n->h = 1 + basic::Max(basic::H(n->left), basic::H(n->right));
  if (DEBUG_AVLTREE) {_verifyTree(n);}
  return n;
};

void TOATree::_verifyNonDom(std::vector<long>& tree_node_ids){
  for (size_t i = 0; i < tree_node_ids.size(); i++) {
    for (size_t j = i+1; j < tree_node_ids.size(); j++) {
      if (EpsDominance(_key[tree_node_ids[i]], _key[tree_node_ids[j]]) || 
          EpsDominance(_key[tree_node_ids[j]], _key[tree_node_ids[i]]) )
      {
        std::cout << " vec1 = " << _key[tree_node_ids[i]] << " vec2 = " << _key[tree_node_ids[j]] << std::endl;
        throw std::runtime_error("[ERROR] TOATree::_verifyNonDom fails.");
      }
    }
  }
  return;
};

///////////////////////////////////////////////////

Frontier3d::Frontier3d() {
  return;
};

Frontier3d::~Frontier3d() {
  return;
};

bool Frontier3d::Check(basic::CostVector g) {
  basic::CostVector ref_vec;
  auto pg = _p(g);
  auto flag = _tree.FindMaxLess(pg, &ref_vec, true); // true means find the max CostVector that is <= l.g.
  if (flag == 0) { // fail to find a smaller vec than l.g.
    return false; // non-dominated.
  }
  return ref_vec[1] <= pg[1]; // these are projected vectors (2-d).
};

void Frontier3d::Update(Label l) {
  auto pg = _p(l.g);
  if (_tree.Size() == 0){
    _tree.Add( pg );
    label_ids.insert(l.id);
    return;
  }
  // non-empty tree.

  label_ids.insert(l.id);
  _tree.Filter(pg);
  _tree.Add(pg); // add at first

  return;
};

basic::CostVector Frontier3d::_p(basic::CostVector v) {
  // TODO, remove this, use _proj().
  basic::CostVector out;
  for (size_t i = 1; i < v.size(); i++){
    out.push_back(v[i]);
  }
  return out;
};

//////////////////////////////////////////////////

//################################################
// KOA* related //
//################################################

KOATree::KOATree() {
  return;
};

KOATree::~KOATree() {
  return;
};

bool KOATree::Check(basic::CostVector v) {
  return _check(_root, v);
};

bool KOATree::_check(basic::AVLNode* n, const basic::CostVector& k) {
  if (n == NULL) {return false;}

  if (EpsDominance(_key[n->id], k)) {
    return true;
  }

  if (_key[n->id] > k) {
    return _check(n->left, k);
  }else{
    if (_check(n->left, k)) {return true;}
    return _check(n->right, k);
  }
};

FrontierKd::FrontierKd() {
  return;
};

FrontierKd::~FrontierKd() {
  return;
};

bool FrontierKd::Check(basic::CostVector g) {
  auto pg = _p(g);
  auto temp = _tree.Check(pg);
  return temp;
};

void FrontierKd::Update(Label l) {
  auto pg = _p(l.g);
  if (_tree.Size() == 0){
    _tree.Add( pg );
    label_ids.insert(l.id);
    return;
  }
  // non-empty tree.

  label_ids.insert(l.id);
  _tree.Filter(pg);
  _tree.Add(pg);

  // NOTE: no need to remove label_ids, Note that Kung's algo has a notation error.

  return;
};

//////////////////////////////////////////////////////////

EMOA::EMOA() {};

EMOA::~EMOA() {
  // for (auto k :_alpha) { // free all frontier that are newed.
  //   delete k.second; // all destructors are virtual
  // }
  // std::cout << "EMOA destructor done" << std::endl;
};

void EMOA::SetGraphPtr(basic::Graph* g) {
  _graph = g;
};

void EMOA::InitHeu(long vd) {
  auto tstart = std::chrono::steady_clock::now();
  _dijks.resize(_graph->GetCostDim());
  for (size_t i = 0; i<_graph->GetCostDim(); i++) {
    _dijks[i].SetGraphPtr(_graph);
    _dijks[i].Search(vd, i);
  }
  auto tend = std::chrono::steady_clock::now();
  _res.rt_initHeu = std::chrono::duration<double>(tend-tstart).count();

  G_DOM_CHECK_COUNT = 0; // reset dom check counter.
  return ;
};

int EMOA::Search(long vo, long vd, double time_limit) {
  // ### init heu ###
  InitHeu(vd);

  // ### init ###
  auto tstart = std::chrono::steady_clock::now();
  _vo = vo;
  _vd = vd;
  basic::CostVector zero_vec;
  zero_vec.resize(_graph->GetCostDim(), 0);
  Label lo(_GenLabelId(), vo, zero_vec, _Heuristic(_vo));
  _label[lo.id] = lo;
  _res.n_generated++;
  // _UpdateFrontier(lo);
  _open.insert( std::make_pair(lo.f, lo.id) );

  if (DEBUG_EMOA > 0) {
    std::cout << "[DEBUG] Init, lo = " << lo << std::endl;
  }

  // ### main search loop ###
  while ( !_open.empty() ) {

    // check timeout
    auto tnow = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(tnow-tstart).count() > time_limit) {
      std::cout << "[INFO] EMOA::Search timeout !" << std::endl;
      _res.timeout = true;
      break;
    }

    // ## select label l, lexicographic order ##
    Label l = _label[ _open.begin()->second ];
    _open.erase(_open.begin());

    if (DEBUG_EMOA > 0) {
      std::cout << "[DEBUG] ### Pop l = " << l << std::endl;
    }

    // ## lazy dominance check ##
    if ( _FrontierCheck(l) || _SolutionCheck(l) ) {
      if (DEBUG_EMOA > 1) {
        std::cout << "[DEBUG] F- AND S-check lazy, dom, cont..." << std::endl;
      }
      continue;
    }
    _UpdateFrontier(l);
    if (l.v == vd) {
      continue;
    }
    if (DEBUG_EMOA > 1) {
      std::cout << "[DEBUG] ### Exp. " << l << std::endl;
    }

    // ## expand label l ##
    _res.n_expanded++;
    auto succs = _graph->GetSuccs(l.v);
    for (auto u : succs) {
      basic::CostVector gu = l.g + _graph->GetCost(l.v, u);
      Label l2(_GenLabelId(), u, gu, gu + _Heuristic(u));
      _label[l2.id] = l2;
      _parent[l2.id] = l.id;
      if (DEBUG_EMOA > 0) {
        std::cout << "[DEBUG] >>>> Loop v= " << u << " gen l' = " << l2 << std::endl;
      }
      if (_FrontierCheck(l2) || _SolutionCheck(l2) ) {
        if (DEBUG_EMOA > 1) {
          std::cout << "[DEBUG] ----- F- AND S-check gene, dom, cont..." << std::endl;
        }
        continue;
      }
      if (DEBUG_EMOA > 0) {
        std::cout << "[DEBUG] ----- Add to open..." << std::endl;
      }
      _res.n_generated++;
      _open.insert( std::make_pair(l2.f, l2.id) );
    }
  };

  // ### post-process the results ###
  std::cout << "[INFO] EMOA::_PostProcRes..." << std::endl;
  _PostProcRes();

  auto tend = std::chrono::steady_clock::now();
  _res.n_domCheck = G_DOM_CHECK_COUNT;
  _res.rt_search = std::chrono::duration<double>(tend-tstart).count();

  std::cout << "[INFO] EMOA::Search exit." << std::endl;
  return 1;
};


basic::CostVector EMOA::_Heuristic(long v) {
  auto out = basic::CostVector(0, _graph->GetCostDim());
  for (size_t cdim = 0; cdim < out.size(); cdim++) {
    out[cdim] = _dijks[cdim].GetCost(v);
    // out[cdim] = 0;
    if (out[cdim] < 0) {
      throw std::runtime_error( "[ERROR], unavailable heuristic !?" );
    }
  }
  // std::cout << " h(" << v << ") = " << out << std::endl;
  return out;
};

EMOAResult EMOA::GetResult() const {
  return _res;
};

long EMOA::_GenLabelId() {
  return _label_id_gen++;
};

void EMOA::_PostProcRes() {
  if (_alpha.find(_vd) != _alpha.end()) {
    for (auto lid : _alpha[_vd].label_ids) {
      _res.paths[lid] = _BuildPath(lid);
      _res.costs[lid] = _label[lid].g;
    }
  }
  return ;
};

bool EMOA::_FrontierCheck(Label l) {
  if (_alpha.find(l.v) == _alpha.end()) {return false;}
  auto res = _alpha[l.v].Check(l.g);
  return res;
};

bool EMOA::_SolutionCheck(Label l) {

  if (_alpha.find(_vd) == _alpha.end()) {return false;}
  auto temp = _alpha[_vd].Check(l.f);

  return temp;
};

std::vector<long> EMOA::_BuildPath(long lid) {
  std::vector<long> out, out2;
  out.push_back(_label[lid].v);
  while( _parent.find(lid) != _parent.end() ) {
    out.push_back(_label[_parent[lid]].v);
    lid = _parent[lid];
  }
  for (size_t i = 0; i < out.size(); i++) {
    out2.push_back(out[out.size()-1-i]);
  }
  return out2;
};

void EMOA::SetGrid(basic::GridkConn& g) {
  // std::cout << " @ set graph, get cost = " << g.GetCost(0,1).ToStr() << std::endl;
  temp_g = g; // to get pybind11 work. otherwise, not properly copied and the pointer is usable.
  _graph = &temp_g;
  // *_graph = g;
  // std::cout << " @ set g_ptr, get cost = " << _graph->GetCost(0,1).ToStr() << std::endl;
};


void EMOA::_UpdateFrontier(Label l) {
  if (_alpha.find(l.v) == _alpha.end()) {
    if (DEBUG_EMOA > 2) {
      std::cout << "[DEBUG] new frontier3d for label " << l << std::endl;
    }
    // _alpha[l.v] = new Frontier3d;
    _alpha[l.v] = Frontier3d();
  }
  // Frontier3d *ptr = dynamic_cast<Frontier3d *>(_alpha[l.v]);
  // ptr->Update(l);
  _alpha[l.v].Update(l);

  // debug info below
  if (DEBUG_EMOA > 0) {
    std::cout << "[DEBUG] ----->> UpdateF. tree(" << l.v << ") : " << std::endl;
    // ptr->_tree.Print() ;
    _alpha[l.v]._tree.Print();
  }
  if (DEBUG_EMOA > 1) {
    std::cout << "[DEBUG] ----->> UpdateF. label ids = {";
    for (auto i : _alpha[l.v].label_ids) {
      std::cout << i << ",";
    }
    std::cout << "} " << std::endl;
  }
};

//////////////////////////////////////////////////

EMOAKd::EMOAKd() {};

EMOAKd::~EMOAKd() {};

void EMOAKd::_PostProcRes() {
  if (_alpha.find(_vd) != _alpha.end()) {
    for (auto lid : _alpha[_vd].label_ids) {
      _res.paths[lid] = _BuildPath(lid);
      _res.costs[lid] = _label[lid].g;
    }
  }
  return ;
};

bool EMOAKd::_FrontierCheck(Label l) {
  if (_alpha.find(l.v) == _alpha.end()) {return false;}
  auto res = _alpha[l.v].Check(l.g);
  return res;
};

bool EMOAKd::_SolutionCheck(Label l) {
  if (_alpha.find(_vd) == _alpha.end()) {return false;}
  return _alpha[_vd].Check(l.f);
};
void EMOAKd::_UpdateFrontier(Label l) {
  if (_alpha.find(l.v) == _alpha.end()) {
    if (DEBUG_EMOA > 2) {
      std::cout << "[DEBUG] new frontierKd for label " << l << std::endl;
    }
    // _alpha[l.v] = new FrontierKd;
    _alpha[l.v] = FrontierKd();
  }
  // FrontierKd *ptr = dynamic_cast<FrontierKd *>(_alpha[l.v]);  
  // if (_mode != 0) {
  //   ptr->_mode = _mode;
  // }
  // ptr->Update(l);
  _alpha[l.v].Update(l);

  // debug info below
  if (DEBUG_EMOA > 0) {
    std::cout << "[DEBUG] ----->> UpdateF. tree(" << l.v << ") : " << std::endl;
    _alpha[l.v]._tree.Print() ;
  }
  if (DEBUG_EMOA > 1) {
    std::cout << "[DEBUG] ----->> UpdateF. label ids = {";
    for (auto i : _alpha[l.v].label_ids) {
      std::cout << i << ",";
    }
    std::cout << "} " << std::endl;
  }
};



} // end namespace search
} // end namespace rzq


/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#include "search_emoa.hpp"
#include "debug.hpp"
#include <set>
#include <memory>
// #include <chrono>

#include <fstream>

namespace rzq{
namespace search{

#define NO_FILTER_FRONTIER false // just for experiment

long G_DOM_CHECK_COUNT = 0; // global variable, make impl easier...

std::ostream& operator<<(std::ostream& os, Label& l)
{
  std::string s;
  s = "{id:" + std::to_string(l.id) + ",v:" + std::to_string(l.v) + ",g:" 
    + ToString(l.g) + ",f:" + ToString(l.f) + "}";
  os << s;
  return os;
};

//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////

EMOA::EMOA() {};

EMOA::~EMOA() {
  // for (auto k :_alpha) { // free all frontier that are newed.
  //   delete k.second; // all destructors are virtual
  // }
  // std::cout << "EMOA destructor done" << std::endl;

  for (int idx = 0; idx < _alpha.size(); idx++){
    delete _alpha[idx];
  }
  return;
};

void EMOA::SetGraphPtr(basic::PlannerGraph* g) {
  _graph = g;
};

void EMOA::InitHeu(long vd) {
  SimpleTimer timer;
  timer.Start();
  _dijks.resize(_graph->CostDim());
  for (size_t i = 0; i<_graph->CostDim(); i++) {
    _dijks[i].SetGraphPtr(_graph);
    _dijks[i].ExhaustiveBackwards(vd, std::numeric_limits<double>::infinity(), i) ;
  }
  _res.rt_initHeu = timer.GetDurationSecond();

  G_DOM_CHECK_COUNT = 0; // reset dom check counter.
  return ;
};

int EMOA::Search(long vo, long vd, double time_limit) {
  
  // NOTE: InitHeu(vd) should be called outside

  _InitFrontiers();
  // _alpha.resize(_graph->NumVertex()); // this requires the graph vertex are numbered from 0 to |V|.

  // ### init ###
  SimpleTimer timer;
  timer.Start();
  _vo = vo;
  _vd = vd;
  auto zero_vec = InitVecType(_graph->CostDim(), 0.0);
  Label lo(_GenLabelId(), vo, zero_vec, _Heuristic(_vo));
  _label[lo.id] = lo;
  _parent[lo.id] = -1; 
  _res.n_generated++;
  // _UpdateFrontier(lo);
  _open.insert( std::make_pair(lo.f, lo.id) );

  if (DEBUG_EMOA > 0) {
    std::cout << "[DEBUG] Init, lo = " << lo << std::endl;
  }

  // ### main search loop ###
  while ( !_open.empty() ) {

    // check timeout
    if (timer.GetDurationSecond() > time_limit) {
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
    auto cvecs = _graph->GetSuccCosts(l.v);
    for (int idx = 0; idx < succs.size(); idx++) {
      auto u = succs[idx];
      CostVec gu = l.g + cvecs[idx];
      Label l2(_GenLabelId(), u, gu, gu + _Heuristic(u));
      _label[l2.id] = l2;
      _parent[l2.id] = l.id;
      if (DEBUG_EMOA > 0) {
        std::cout << "[DEBUG] >>>> Loop v= " << u << " gen l' = " << l2 << std::endl;
      }
      if (_FrontierCheck(l2) || _SolutionCheck(l2) ) {
        if (DEBUG_EMOA > 1) {
          std::cout << "[DEBUG] ----- F- AND S-check eager, dom, cont..." << std::endl;
        }
        continue;
      }
      if (DEBUG_EMOA > 0) {
        std::cout << "[DEBUG] ----- Add to open..." << std::endl;
      }
      _res.n_generated++;
      _open.insert( std::make_pair(l2.f, l2.id) );
    } // end for
  } // end while

  // ### post-process the results ###
  std::cout << "[INFO] EMOA::_PostProcRes..." << std::endl;
  _PostProcRes();

  _res.n_domCheck = G_DOM_CHECK_COUNT;
  _res.rt_search = timer.GetDurationSecond();

  std::cout << "[INFO] EMOA::Search exit." << std::endl;
  return 1;
};


CostVec EMOA::_Heuristic(long v) {
  auto out = CostVec(_graph->CostDim(), 0);
  for (size_t cdim = 0; cdim < out.size(); cdim++) {
    out[cdim] = _dijks[cdim].GetDistValue(v);
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

  long out = _label_id_gen++;

  if (_label_id_gen > __vec_alloc_total){
    __vec_alloc_total += __vec_alloc_batch;
    _label.resize(__vec_alloc_total);
    _parent.resize(__vec_alloc_total);
    if (__vec_alloc_batch < __vec_alloc_batch_max){
      __vec_alloc_batch += __vec_alloc_batch;
    }
  }

  return out;
};

void EMOA::_PostProcRes() {
  // if (_alpha.find(_vd) != _alpha.end()) {
    for (auto lid : _alpha[_vd]->label_ids) {
      _res.paths[lid] = _BuildPath(lid);
      _res.costs[lid] = _label[lid].g;
    }
  // }
  return ;
};

bool EMOA::_FrontierCheck(Label l) {
  // if (_alpha.find(l.v) == _alpha.end()) {return false;}
  auto res = _alpha[l.v]->Check(l.g);
  return res;
};

bool EMOA::_SolutionCheck(Label l) {

  // if (_alpha.find(_vd) == _alpha.end()) {return false;}
  auto temp = _alpha[_vd]->Check(l.f);

  return temp;
};

std::vector<long> EMOA::_BuildPath(long lid) {
  std::vector<long> out, out2;
  while( lid >= 0 ) {
    out.push_back(_label[lid].v);
    lid = _parent[lid];
  }
  for (size_t i = 0; i < out.size(); i++) {
    out2.push_back(out[out.size()-1-i]);
  }
  return out2;
};

// void EMOA::SetGrid(basic::GridkConn& g) {
//   // std::cout << " @ set graph, get cost = " << g.GetCost(0,1).ToStr() << std::endl;
//   temp_g = g; // to get pybind11 work. otherwise, not properly copied and the pointer is usable.
//   _graph = &temp_g;
//   // *_graph = g;
//   // std::cout << " @ set g_ptr, get cost = " << _graph->GetCost(0,1).ToStr() << std::endl;
// };


void EMOA::_InitFrontiers() {
  _alpha.resize(_graph->NumVertex()); // this requires the graph vertex are numbered from 0 to |V|.
  for (int idx = 0; idx < _alpha.size(); idx++){
    _alpha[idx] = new Frontier;
  }
  return;
};

void EMOA::_UpdateFrontier(Label l) {

  // if (_alpha.find(l.v) == _alpha.end()) {
  //   if (DEBUG_EMOA > 2) {
  //     std::cout << "[DEBUG] new frontier3d for label " << l << std::endl;
  //   }
  //   // _alpha[l.v] = new Frontier3d;
  //   _alpha[l.v] = Frontier();
  // }

  // Frontier3d *ptr = dynamic_cast<Frontier3d *>(_alpha[l.v]);
  // ptr->Update(l);
  _alpha[l.v]->Update(l);

  // debug info below
  if (DEBUG_EMOA > 0) {
    std::cout << "[DEBUG] ----->> UpdateF. tree(" << l.v << ") : " << std::endl;
    // ptr->_tree.Print() ;
    // _alpha[l.v].Print();
  }
  if (DEBUG_EMOA > 1) {
    std::cout << "[DEBUG] ----->> UpdateF. label ids = {";
    for (auto i : _alpha[l.v]->label_ids) {
      std::cout << i << ",";
    }
    std::cout << "} " << std::endl;
  }
};


//################################################
// TOA* related //
//################################################

// EMOATree::EMOATree() {
  
// };

// EMOATree::~EMOATree() {
  
// };

void Frontier::Filter(CostVec v) {
  int outFlag = 0;
  std::unordered_set<long> filtered_node;
  _root = _filter(_root, v, &filtered_node);
  if (filtered_node.size()>0) {
    _rebuildTree(&filtered_node);
  }
  return;
};

basic::AVLNode* Frontier::_filter(
  basic::AVLNode* n, const CostVec& k, std::unordered_set<long> *filtered_node)
{
  if (n == NULL) {return n;}
  if ( _key[n->id] < k ) { // only need to look at the right sub-tree.
    n->right = _filter(n->right, k, filtered_node); 
  }else{
    n->left = _filter(n->left, k, filtered_node);
    n->right = _filter(n->right, k, filtered_node);
  }
  // no return above, will reach here if n != NULL.
  if (EpsDom(k, _key[n->id])) {
    if (filtered_node) {
      filtered_node->insert(n->id);
    }
  }
  return n;
};

void Frontier::_rebuildTree(std::unordered_set<long> *skip_node) {
  std::vector<long> tree_node_ids;
  size_t current_size = _size;
  ToSortedVector(NULL, &tree_node_ids, skip_node);

  _size = 0;
  basic::AVLNode* new_root = _rebuildTreeMethod(tree_node_ids, 0, tree_node_ids.size()-1);
  _deleteAll(_root);
  _root = new_root;

  return ;
};

basic::AVLNode* Frontier::_rebuildTreeMethod(std::vector<long> &tree_node_ids, long start, long end) {
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

void Frontier::_verifyNonDom(std::vector<long>& tree_node_ids){
  for (size_t i = 0; i < tree_node_ids.size(); i++) {
    for (size_t j = i+1; j < tree_node_ids.size(); j++) {
      if (EpsDom(_key[tree_node_ids[i]], _key[tree_node_ids[j]]) || 
          EpsDom(_key[tree_node_ids[j]], _key[tree_node_ids[i]]) )
      {
        std::cout << " vec1 = " << _key[tree_node_ids[i]] << " vec2 = " << _key[tree_node_ids[j]] << std::endl;
        throw std::runtime_error("[ERROR] Frontier::_verifyNonDom fails.");
      }
    }
  }
  return;
};


// bool Frontier::Check2(CostVec v) {
//   return _check(_root, v);
// };

bool Frontier::_check(basic::AVLNode* n, const CostVec& k) {
  if (n == NULL) {return false;}

  if (EpsDom(_key[n->id], k)) {
    return true;
  }

  if (_key[n->id] > k) {
    return _check(n->left, k);
  }else{
    if (_check(n->left, k)) {return true;}
    return _check(n->right, k);
  }
};


///////////////////////////////////////////////////

Frontier::Frontier() {
  return;
};

Frontier::~Frontier() {
  return;
};

bool Frontier::Check(CostVec g) {
  auto pg = g;
  pg = _p(g);
  // std::cout << " FrontierKd::Check pg = " << pg << std::endl;
  // auto temp = Check2(pg);
  return _check(_root, pg);
  // return temp;
};

void Frontier::Update(Label l) {
  auto pg = _p(l.g);
  if (this->Size() == 0){
    Add( pg );
    label_ids.push_back(l.id);
    return;
  }
  // non-empty tree.

  label_ids.push_back(l.id);
  Filter(pg);
  Add(pg); // add at first

  return;
};

CostVec Frontier::_p(CostVec v) {
  CostVec out;
  for (size_t i = 1; i < v.size(); i++){
    out.push_back(v[i]);
  }
  return out;
};

//////////////////////////////////////////////////

/**
 *
 */
void GraphExpandCostDim(rzq::basic::SparseGraph* g, bool add_deg_cost, bool add_len_cost) {
  if (add_deg_cost){
    int cdim = g->CostDim();
    g->ChangeCostDim(cdim+1);
    auto v_all = g->AllVertex();
    for (auto v : v_all){
      auto succs = g->GetSuccs(v);
      auto costs = g->GetSuccCosts(v);
      for (int j = 0; j < succs.size(); j++){
        long u = succs[j];
        auto cvec = costs[j];
        auto temp1 = g->GetPreds(v); // to get in-degree
        auto temp2 = g->GetPreds(u);
        
        // cvec.back() = 1.0 * ( (temp1.size() + temp2.size()) / 2 ) ;
        
        double cost = 1.0 * (temp1.size() + temp2.size()) / 2 ;
        if (cost >= 4) {
          cvec.back() = 2;
        }else{
          cvec.back() = 1;
        }

        g->SetArcCost(v,u, cvec);
      }
    }
  }
  if (add_len_cost){
    int cdim = g->CostDim();
    g->ChangeCostDim(cdim+1);
    auto v_all = g->AllVertex();
    for (auto v : v_all){
      auto succs = g->GetSuccs(v);
      auto costs = g->GetSuccCosts(v);
      for (int j = 0; j < succs.size(); j++){
        long u = succs[j];
        auto cvec = costs[j];
        cvec.back() = 1;
        g->SetArcCost(v,u, cvec);
      }
    }
  }
};

int RunEMOA(rzq::basic::PlannerGraph* g, long vo, long vd, double time_limit, rzq::search::EMOAResult* res)
{
  size_t cdim = g->CostDim();
  std::cout << "[INFO] RunEMOA, M=" << cdim << " time_limit = " << time_limit << std::endl;
  int ret_flag = 0;

  // The following if-else is actually minor. Just use EMOAKd for all cases is totally fine...
  auto planner = rzq::search::EMOA();
  planner.SetGraphPtr(g) ; // set graph to the planner
  planner.InitHeu(vd); // this vd must be the same as the vd in Search().
  ret_flag = planner.Search(vo, vd, time_limit) ;
  *res = planner.GetResult(); // get result

  std::cout << "[INFO] RunEMOA, exit with flag " << ret_flag << " with " << res->paths.size() 
            << " solutions in " << res->rt_initHeu << "(for heu) + " << res->rt_search << "(for search) seconds." 
            << std::endl;

  return ret_flag; // TODO, add more return flags.
};


int SaveEMOAResult(std::string fname, const search::EMOAResult& res) {
  std::ofstream fout;

  fout.open(fname);
  if (!fout) {
    std::cerr << "Error: file '" << fname << "' could not be opened" << std::endl;
    return -1;
  }

  fout << "EMOAResults: " << std::endl;
  fout << "n_generated: " << res.n_generated << std::endl;
  fout << "n_expanded: " << res.n_expanded << std::endl;
  fout << "n_domCheck: " << res.n_domCheck << std::endl;
  fout << "rt_initHeu: " << res.rt_initHeu << std::endl;
  fout << "rt_search: " << res.rt_search << std::endl;
  fout << "timeout: " << res.timeout << std::endl;
  fout << "num_nondom_labels_max: " << res.num_nondom_labels_max << std::endl;
  fout << "num_nondom_labels_avg: " << res.num_nondom_labels_avg << std::endl;
  fout << "N: " << res.costs.size() << std::endl;

  int index = 0;
  for (const auto& kv: res.costs) {
    fout << "Label: " << kv.first << std::endl;
    fout << ToString(kv.second) << std::endl;
    std::vector<long> path = res.paths.at(kv.first);
    for (int i = 0; i < path.size(); i++) {
      fout << path[i] << " ";
    }
    fout << std::endl;
    index += 1;
  }

  return 1;
}


// //################################################
// // FrontierNaive related //
// //################################################

// FrontierNaive::FrontierNaive(){};

// FrontierNaive::~FrontierNaive(){};

// bool FrontierNaive::Check(CostVec g){
//   for (auto k : labels){
//     auto l = k.second;
//     if (EpsDom(l.g, g)) {
//       return true;
//     }
//   }
//   return false;
// };

// void FrontierNaive::Update(Label l){
//   // std::vector<long> tbd;
//   // for (auto k : labels){
//   //   auto lk = k.second;
//   //   if (EpsDom(l.g, lk.g)) {
//   //     tbd.push_back(lk.id);
//   //   }
//   // }
//   // for (auto k: tbd){
//   //   labels.erase(k);
//   // }
//   labels[l.id] = l;
//   label_ids.push_back(l.id);
//   return;
// };


} // end namespace search
} // end namespace rzq


/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_SEARCH_EMOA_H_
#define ZHONGQIANGREN_SEARCH_EMOA_H_

#include "graph.hpp"
#include "dijkstra.hpp"
#include "avltree.hpp"

#include <set>
#include <limits>
#include <list>

namespace rzq{
namespace search{

#define DEBUG_EMOA 0

extern long G_DOM_CHECK_COUNT; // global variable, make impl easier...

/**
 * @brief Epsilon-dominance.
 */
template<typename IterData>
bool EpsDominance(IterData v1, IterData v2, double eps=0.0, bool less=true) {
  G_DOM_CHECK_COUNT++; // counter increase.
  auto i2 = v2.begin();
  for (auto i1 = v1.begin(); i1 != v1.end(); i1++){
    if (less) {
      if (*i1 > (1.0+eps)*(*i2)) { return false; }
    }else{
      if (*i1 < (1.0+eps)*(*i2)) { return false; }
    }
    i2++;
  }
  return true;
};

/**
 * @brief A search label, used by EMOA*.
 */
struct Label {
  Label() {};
  Label(long id0, long v0, basic::CostVector g0, basic::CostVector f0) {
    id = id0; v = v0; g = g0; f = f0;
  };
  long id; // label's id, make it easy to look up.
  long v;
  basic::CostVector g;
  basic::CostVector f;
};

std::ostream& operator<<(std::ostream& os, Label& l) ;

/**
 * @brief result data structure.
 */
struct EMOAResult {
  std::unordered_map< long, std::vector<long> > paths;
  std::unordered_map< long, basic::CostVector > costs;
  long n_generated = 0;
  long n_expanded = 0;
  long n_domCheck = 0;
  double rt_initHeu = 0.0;
  double rt_search = 0.0;
  bool timeout = false;
};

//////////////////////////////////////////////////////////////


basic::CostVector _proj(basic::CostVector v) ;


/**
 * @brief an interface, depends on the impl of BOA*, TOA*, etc.
 */
class Frontier {
public:
  Frontier(){};
  virtual ~Frontier(){};
  virtual bool Check(basic::CostVector g) = 0; // check if l is dominated or not.
  virtual void Update(Label l) = 0; // update the frontier using l.
  std::unordered_set<long> label_ids;
};

/**
 * @brief
 */
class TOATree : public basic::AVLTree<basic::CostVector> 
{
public:
  TOATree() ;
  ~TOATree() ;
  virtual void Filter(basic::CostVector v) ;
  virtual void _rebuildTree(std::unordered_set<long> *skip_node);
protected:
  // virtual basic::AVLNode* _filter(basic::AVLNode* n, const basic::CostVector& k, int* outFlag=NULL);
  virtual basic::AVLNode* _filter(basic::AVLNode* n, const basic::CostVector& k, std::unordered_set<long> *a = NULL);
  virtual basic::AVLNode* _rebuildTreeMethod(std::vector<long> &tree_node_ids, long start, long end);
  virtual void _verifyNonDom(std::vector<long>&);
};

/**
 * @brief The frontier set at each node, for TOA* (Tri-objective A*).
 */
class Frontier3d : public Frontier
{
public:
  Frontier3d();
  virtual ~Frontier3d();
  virtual bool Check(basic::CostVector g) override ;
  virtual void Update(Label l) override ;
// protected:
  // project the 3d vector to 2d vector by removing the first component.
  virtual basic::CostVector _p(basic::CostVector v);
  TOATree _tree;
};


//////////////////////////////////////////////////////////////////////

/**
 * @brief an interface / base class, use its pointer
 */
class EMOA {
public:
  EMOA() ;
  virtual ~EMOA() ;
  // virtual void SetMode(const std::string in) ;

  // set graph as pointer, note to leverage polymorphism here.
  virtual void SetGraphPtr(basic::Graph* g) ;

  // this vd must be the same as the vd in Search().
  virtual void InitHeu(long vd);
  
  // heuristic computation are included in each search call.
  virtual int Search(long vo, long vd, double time_limit) ;

  virtual EMOAResult GetResult() const ;

  /**
   * @brief a new API for python wrapper. only for grid like world. 
   * Add to make it compatible with pybind11.
   */
  virtual void SetGrid(basic::GridkConn& g) ;
protected:
  // return the heuristic vector from v to vd.
  virtual basic::CostVector _Heuristic(long v) ;

  // this method needs to new frontiers, which depend on the specific #obj.
  virtual void _UpdateFrontier(Label l) ;

  virtual long _GenLabelId() ;

  virtual bool _FrontierCheck(Label l) ;
  
  virtual bool _SolutionCheck(Label l) ;

  virtual void _PostProcRes();

  virtual std::vector<long> _BuildPath(long lid) ;

  basic::Graph* _graph;
  std::unordered_map< long, Frontier3d > _alpha; // map a vertex id (v) to alpha(v).
  long _label_id_gen = 0;
  long _vo = -1, _vd = -1;
  std::set< std::pair< basic::CostVector, long> > _open;
  std::unordered_map<long, Label> _label;
  std::unordered_map<long, long> _parent;
  EMOAResult _res;
  std::vector<DijkstraScan> _dijks;
  // std::string _mode = ""; // for some special usage

  basic::GridkConn temp_g; // temp, to be removed.
};

//////////////////////////////////////////////////////////////////

/**
 * @brief
 */
class KOATree : public TOATree
{
public:
  KOATree() ;
  virtual ~KOATree() ;
  virtual bool Check(basic::CostVector v) ;
  // void Filter(basic::CostVector v) ;
protected:
  virtual bool _check(basic::AVLNode* n, const basic::CostVector& k) ; // new impl @2021-08-31
};

/**
 * @brief The frontier set at each node, for TOA* (Tri-objective A*).
 */
class FrontierKd : public Frontier3d
{
public:
  FrontierKd();
  virtual ~FrontierKd();
  virtual bool Check(basic::CostVector g) override ;
  virtual void Update(Label l) override ;
// protected:
  // project the 3d vector to 2d vector by removing the first component.
  // virtual basic::CostVector _p(basic::CostVector v);
  KOATree _tree; // TODO use polymorphism, change to pointer...
  int _mode = 0;
};

/**
 * 
 */
class EMOAKd : public EMOA
{
public:
  EMOAKd();
  ~EMOAKd();
protected:
  // virtual basic::CostVector _Heuristic(long v) override;
  /**
   * @brief This method needs to new frontiers, which depend on the specific #obj.
   */
  virtual void _UpdateFrontier(Label l) override;
  virtual bool _FrontierCheck(Label l) override ;
  virtual bool _SolutionCheck(Label l) override;
  virtual void _PostProcRes() override;
  std::unordered_map< long, FrontierKd > _alpha; // map a vertex id (v) to alpha(v).
};

//////////////////////////////////////////////////////////////////


} // end namespace search
} // end namespace rzq


#endif  // ZHONGQIANGREN_SEARCH_EMOA_H_

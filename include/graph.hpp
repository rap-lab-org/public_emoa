
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_BASIC_GRAPH_H_
#define ZHONGQIANGREN_BASIC_GRAPH_H_

#include <string>
#include <vector>
#include <iostream>
// #include "cost_vector.hpp"
#include "vec_type.hpp"

namespace rzq{
namespace basic{

// #define GraphCostType CostVector // always vector-cost, more general

/**
 * @brief This class is an interface (for planners) to access all directed graphs G=(V,E,C), where
 * V is a vertex set with a long integer ID.
 * E is an arc set (u,v) with u in V, v in V.
 * C is a cost map that maps an edge (u,v) to a cost of type VecType (vector-cost).
 * CAVEAT: graph vertex ID should be within range [0,N], since std::vector is used as the underlying storage.
 */
class PlannerGraph {
public:
  /**
   * @brief
   */
  PlannerGraph() {};
  /**
   * @brief
   */
  virtual ~PlannerGraph() {};
  /**
   * @brief
   */
  virtual bool HasVertex(long v) = 0;
  /**
   * @brief
   */
  virtual bool HasArc(long v, long u) = 0;
  /**
   * @brief return successors of node v
   */
  virtual std::vector<long> GetSuccs(long v) = 0;
  /**
   * @brief return predecessors of node v
   */
  virtual std::vector<long> GetPreds(long v) = 0;
  /**
   * @brief
   */
  virtual std::vector<double> GetCost(long u, long v) = 0;
  /**
   * @brief 
   */
  virtual std::vector< std::vector<double> > GetSuccCosts(long u) = 0;
  /**
   * @brief 
   */
  virtual std::vector< std::vector<double> > GetPredCosts(long u) = 0;
  /**
   * @brief
   */
  virtual size_t NumVertex() = 0;
  /**
   * @brief 
   */  
  virtual size_t NumArc() = 0;
  /**
   * @brief only meaningful for undirected graph. it is the same as NumArc()/2. 
   */  
  virtual size_t NumEdge() = 0;
  /**
   * @brief 
   */  
  virtual size_t CostDim() = 0;
  /**
   * @brief 
   */  
  virtual std::vector<long> AllVertex() = 0;
};

/**
 *
 */
class SparseGraph: public PlannerGraph
{
public:
  /**
   * @brief
   */
  SparseGraph() ;
  /**
   * @brief
   */
  virtual ~SparseGraph() ;
  /**
   * @brief
   */
  virtual bool HasVertex(long v) override ;
  /**
   * @brief
   */
  virtual bool HasArc(long v, long u) override ;
  /**
   * @brief return successors of node v.
   */
  virtual std::vector<long> GetSuccs(long v) override ;
  /**
   * @brief return predecessors of node v
   */
  virtual std::vector<long> GetPreds(long v) override ;
  /**
   * @brief Not recommended. For better performance, use GetSuccCosts and GetPredCosts instead.
   */
  virtual std::vector<double> GetCost(long u, long v) override ;
  /**
   * @brief a vector of all successor costs
   */
  virtual std::vector< std::vector<double> > GetSuccCosts(long u) override ;
  /**
   * @brief a vector of all predecessor costs
   */
  virtual std::vector< std::vector<double> > GetPredCosts(long u) override ;
  /**
   * @brief 
   */  
  virtual size_t NumVertex() override ;
  /**
   * @brief 
   */  
  virtual size_t NumArc() override ;
  /**
   * @brief only meaningful for undirected graph. it is the same as NumArc()/2. 
   */  
  virtual size_t NumEdge() override ;
  /**
   * @brief 
   */  
  virtual size_t CostDim() override ;
  /**
   * @brief 
   */  
  virtual std::vector<long> AllVertex() override ;

  //--------------------------------------
  //--------------------------------------
  //#### Non-Inherited Methods Below ####
  //--------------------------------------
  //--------------------------------------

  /**
   * @brief Add a new vertex v, return the ID of the vertex. 
   */
  virtual void AddVertex(long v);
  /**
   *
   */
  virtual void AddEdge(long u, long v, std::vector<double> c);
  /**
   *
   */
  virtual void AddArc(long u, long v, std::vector<double> c);
  /**
   * @brief Create an undirected graph from the given set of edges.
   */
  virtual void CreateFromEdges(std::vector<long> sources, 
    std::vector<long> targets, std::vector< std::vector<double> > costs);
  /**
   * @brief Create an undirected graph from the given set of arcs.
   */
  virtual void CreateFromArcs(std::vector<long> sources, 
    std::vector<long> targets, std::vector< std::vector<double> > costs);
  /**
   *
   */
  virtual void ChangeCostDim(size_t new_cdim, double default_value=0.0);
  /**
   *
   */
  virtual bool SetArcCost(long u, long v, const std::vector<double>& new_cost);
  /**
   *
   */
  virtual std::string ToStr() const ;
protected:
  std::vector< std::vector<long> > _to;
  std::vector< std::vector< std::vector<double> > > _to_cost;
  std::vector< std::vector<long> > _from;
  std::vector< std::vector< std::vector<double> > > _from_cost;
  size_t _n_arc = 0;
  size_t _cdim = 0;
};

/**
 *
 */
std::ostream& operator<<(std::ostream& os, const SparseGraph& c) ;

/**
 * TODO, dense graph, matrix representation...
 */
// class DenseGraph : public Graph
// {
// public:
//   DenseGraph();
//   virtual ~DenseGraph();
// };


/**
 * @brief
 */
class Grid2d : public PlannerGraph
{
public:
  /**
   *
   */
  Grid2d();
  /**
   *
   */
  virtual ~Grid2d();
  /**
   * @brief
   */
  virtual bool HasVertex(long v) override ;
  /**
   * @brief
   */
  virtual bool HasArc(long v, long u) override ;
  /**
   * @brief return successors of node v
   */
  virtual std::vector<long> GetSuccs(long v) override ;
  /**
   * @brief return predecessors of node v
   */
  virtual std::vector<long> GetPreds(long v) override ;
  /**
   * @brief Not recommended. For better performance, use GetSuccCosts and GetPredCosts instead.
   */
  virtual std::vector<double> GetCost(long u, long v) override ;
  /**
   * @brief
   */
  virtual std::vector< std::vector<double> > GetSuccCosts(long u) override ;
  /**
   * @brief 
   */
  virtual std::vector< std::vector<double> > GetPredCosts(long u) override ;
  /**
   * @brief 
   */  
  virtual size_t NumVertex() override ;
  /**
   * @brief 
   */  
  virtual size_t NumArc() override ;
  /**
   * @brief only meaningful for undirected graph. it is the same as NumArc()/2. 
   */  
  virtual size_t NumEdge() override ;
  /**
   * @brief 
   */  
  virtual size_t CostDim() override ;
  /**
   * @brief 
   */  
  virtual std::vector<long> AllVertex() override ;

  //#### Non-Inherited Methods Below ####

  /** 
   * @brief Treat the input as a binary matrix where: cell value <=0 means free and cell value >0 means obstacles.
   * And let each edge cost to be a vector of length one with value one.
   */
  virtual void SetOccuGridPtr(std::vector< std::vector<double> >*) ;
  /** 
   * @brief For pybind11.
   */
  virtual void SetOccuGridObject(std::vector< std::vector<double> >&) ;

  /**
   * @brief 
   */
  virtual bool SetKNeighbor(int kngh) ;
  /**
   * @brief 
   */
  virtual long _rc2k(const long r, const long c) const ;
  /**
   * @brief 
   */
  virtual long _k2r(const long k) const ;
  /**
   * @brief 
   */
  virtual long _k2c(const long k) const ;

protected:
  std::vector< std::vector< double > > _mat_from_py;
  std::vector< std::vector< double > >* _occu_grid_ptr;
    // row (y) first, column (x) next, the value in a cell indicates if that cell is an obstacle.
  int _kngh;
  std::vector<long> _act_r;
  std::vector<long> _act_c;
};

// /**
//  *
//  */
// struct Grid : std::vector< std::vector<long> >
// {
//   /**
//    *
//    */
//   Grid() ;
//   /**
//    *
//    */
//   virtual ~Grid() ;
//   /**
//    *
//    */
//   void Resize(size_t r, size_t c, int val=0) ;
//   /**
//    *
//    */
//   size_t GetColNum() const ;
//   /**
//    *
//    */
//   size_t GetRowNum() const ;
//   /**
//    *
//    */
//   void Set(size_t r, size_t c, int val) ; // no boundary check
//   /**
//    *
//    */
//   int Get(size_t r, size_t c) ; // no boundary check
// };

// std::ostream& operator<<(std::ostream& os, const Grid& g) ;

// // /**
// //  *
// //  */
// // struct CvecGrid : std::vector< std::vector<CostVector> >
// // {
// //   CvecGrid();
// //   ~CvecGrid();
// // };

// /**
//  * @brief This class is a 4-connected grid implementation of Graph
//  */
// class GridkConn : public Graph
// {
// public:
//   GridkConn() ;
//   virtual ~GridkConn() ;

//   // input an occupancy grid (with values 0 or 1), and a matrix of cost vectors cvecs.
//   // Here, cvecs(i,j) indicates the cost to arrive at that cell (i,j).
//   virtual void Init(Grid grid, std::vector<Grid> cvecs) ;

//   virtual void SetActionSet(std::vector< std::vector<int> > actions) ;

//   virtual bool HasNode(long v) ;

//   // return successors of node v
//   virtual std::unordered_set<long> GetSuccs(long v) ;

//   // return predecessors of node v
//   virtual std::unordered_set<long> GetPreds(long v) ;

//   // M-dimensional cost vector
//   virtual CostVector GetCost(long u, long v) ;

//   virtual size_t GetCostDim();

//   virtual long GetGridValue(long u) ;

//   // image coord, y is row, x is col, i.e. grid[y,x]
//   // v := x + y * num_cols.
//   long v(int y, int x); 
//   int y(long v); // get row index
//   int x(long v); // get col index

// protected:
//   Grid _grid;
//   std::vector<Grid> _cvecs;
//   std::vector< std::vector< CostVector > > _cvecs2; // internally re-arrange to improve cache miss.
//   int _nc=0, _nr=0, _cdim=0;
//   std::vector< std::vector<int> > _actions;
// };

} // end namespace basic
} // end namespace zr

#endif  // ZHONGQIANGREN_BASIC_GRAPH_H_

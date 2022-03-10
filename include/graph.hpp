
/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/


#ifndef ZHONGQIANGREN_BASIC_GRAPH_H_
#define ZHONGQIANGREN_BASIC_GRAPH_H_

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <iostream>

namespace rzq{
namespace basic{

struct CostVector : std::vector<long> {
  CostVector();
  CostVector(long val, size_t dim) ;
  CostVector(const std::vector<long>&) ;
  CostVector operator+(const CostVector& v) ;
  int CompareLexico(const CostVector& v1) ;
  CostVector& operator+=(const CostVector& rhs) ;
  bool operator==(const CostVector& rhs) ;
  CostVector operator-(const CostVector& v) ;
  CostVector operator*(const long& k) ;
  CostVector ElemWiseMin(const CostVector& rhs) ;
  std::string ToStr() const ;
};
std::ostream& operator<<(std::ostream& os, const CostVector& c) ;


/**
 * @brief This class is an interface.
 */
class Graph
{
public:
  Graph() {};
  virtual ~Graph() {};

  virtual bool HasNode(long v) = 0;

  // return successors of node v
  virtual std::unordered_set<long> GetSuccs(long v) = 0;

  // return predecessors of node v
  virtual std::unordered_set<long> GetPreds(long v) = 0;

  // M-dimensional cost vector
  virtual CostVector GetCost(long u, long v) = 0;

  virtual size_t GetCostDim() = 0;
};

/**
 *
 */
struct Grid : std::vector< std::vector<long> >
{
  Grid() ;
  virtual ~Grid() ;
  void Resize(size_t r, size_t c, int val=0) ;
  size_t GetColNum() const ;
  size_t GetRowNum() const ;
  void Set(size_t r, size_t c, int val) ; // no boundary check
  int Get(size_t r, size_t c) ; // no boundary check
};

// /**
//  *
//  */
// struct CvecGrid : std::vector< std::vector<CostVector> >
// {
//   CvecGrid();
//   ~CvecGrid();
// };

/**
 * @brief This class is a 4-connected grid implementation of Graph
 */
class GridkConn : public Graph
{
public:
  GridkConn() ;
  virtual ~GridkConn() ;

  // input an occupancy grid (with values 0 or 1), and a matrix of cost vectors cvecs.
  // Here, cvecs(i,j) indicates the cost to arrive at that cell (i,j).
  virtual void Init(Grid grid, std::vector<Grid> cvecs) ;

  virtual void SetActionSet(std::vector< std::vector<int> > actions) ;

  virtual bool HasNode(long v) ;

  // return successors of node v
  virtual std::unordered_set<long> GetSuccs(long v) ;

  // return predecessors of node v
  virtual std::unordered_set<long> GetPreds(long v) ;

  // M-dimensional cost vector
  virtual CostVector GetCost(long u, long v) ;

  virtual size_t GetCostDim();

  // image coord, y is row, x is col, i.e. grid[y,x]
  // v := x + y * num_cols.
  long v(int y, int x);
  int y(long v); // get row index
  int x(long v); // get col index

protected:
  Grid _grid;
  std::vector<Grid> _cvecs;
  std::vector< std::vector< CostVector > > _cvecs2; // internally re-arrange to improve cache miss.
  int _nc=0, _nr=0, _cdim=0;
  std::vector< std::vector<int> > _actions;
};


typedef std::unordered_map<long, std::unordered_map<long, CostVector > > AdjList;

class Roadmap : public Graph {
public:
  Roadmap() ;
  virtual ~Roadmap() ;

  virtual void Init(long nv, int cdim);

  virtual bool HasNode(long v) override;

  // return successors of node v
  virtual std::unordered_set<long> GetSuccs(long v) override;

  // return predecessors of node v
  virtual std::unordered_set<long> GetPreds(long v) override;

  // M-dimensional cost vector
  virtual CostVector GetCost(long u, long v) override;

  virtual size_t GetCostDim() override;

  // Additional methods

  virtual bool HasEdge(long u, long v);

  virtual std::unordered_set<long> GetNodes();

  virtual void AddNode(long v);
  virtual void AddEdge(long v1, long v2, CostVector& c);
  virtual long GetNumberOfNodes();
  virtual long GetNumberOfEdges();

  // for each edge, the value of its cost vector at index 'cost_index' is set to be the sum of the degrees of the endpoints of the edge
  virtual void AddDegreeCost(int cost_index);

public:
  AdjList _adjlist;
  AdjList _adjlist_rev;
  std::unordered_set<long> _nodes;
  long _num_nodes=0, _num_edges=0;
  size_t _cdim=0;
};



} // end namespace basic
} // end namespace rzq


#endif  // ZHONGQIANGREN_BASIC_GRAPH_H_

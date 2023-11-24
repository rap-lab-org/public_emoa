
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_BASIC_SEARCH_H_
#define ZHONGQIANGREN_BASIC_SEARCH_H_

#include "graph.hpp"
#include <chrono>

namespace rzq{
namespace search{
  
using namespace rzq::basic;

/**
 * @brief This class is an interface of search-based planners.
 */
class GraphSearch {
public:
  /**
   * @brief
   */
  GraphSearch() ;
  /**
   * @brief
   */
  virtual ~GraphSearch() ;
  /**
   *
   */
  virtual void SetGraphPtr(PlannerGraph* g) ;
  /**
   * @brief cdim specifies which cost dimension of the graph will be used for search.
   * time_limit is the runtime limit.
   */
  virtual std::vector<long> PathFinding(long vs, long vg, double time_limit, short cdim) = 0;
  /**
   * @brief return the solution path cost, must be called after calling PathFinding()
   */
  virtual std::vector<double> GetSolutionCost() = 0;


protected:
  PlannerGraph* _graph;
  int _mode = 0;
  long _vs = -1;
  long _vg = -1;
  double _time_limit = -1;
};


} // end namespace search
} // end namespace zr

#endif  // ZHONGQIANGREN_BASIC_SEARCH_H_

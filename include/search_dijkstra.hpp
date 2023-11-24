
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_BASIC_SEARCH_DIJKSTRA_H_
#define ZHONGQIANGREN_BASIC_SEARCH_DIJKSTRA_H_

#include "search.hpp"

#define DEBUG_DIJKSTRA 0

namespace rzq{
namespace search{

using namespace rzq::basic;

/**
 * @brief
 * 
 * CAVEAT: The graph vertex ID should be within range [0,N], since std::vector is used as the underlying storage.
 * NOTE: This implementation assumes the entire graph is available.
 */
class Dijkstra : public GraphSearch
{
public:
	/**
	 *
	 */
	Dijkstra() ;
	/**
	 *
	 */
	virtual ~Dijkstra() ;
	/**
	 * @brief cdim specifies which cost dimension of the graph will be used for search.
	 */
	virtual std::vector<long> PathFinding(long vs, long vg, double time_limit = std::numeric_limits<double>::infinity(), short cdim = 0) override ;	
	/**
	 * @brief This must be called after calling PathFinding().
	 */
	virtual std::vector<double> GetSolutionCost() override ;
	/**
	 *
	 */
	virtual int ExhaustiveBackwards(long vg, double time_limit = std::numeric_limits<double>::infinity(), short cdim = 0) ;
	/**
	 *
	 */
	virtual int ExhaustiveForwards(long vs, double time_limit = std::numeric_limits<double>::infinity(), short cdim = 0) ;
	/**
	 *
	 */
	virtual std::vector<long> GetPath(long v, bool do_reverse=true) ;
	/**
	 * @brief Return a vector that stores the cost-to-go from all other vertices to/from the 
	 *   given vg/vs, depending on whether ExhaustiveBackwards/ExhaustiveForwards is called.
	 */
	std::vector<double> GetDistAll() ;
	/**
	 * @brief Return the cost-to-go from v to/from the 
	 *   given vg/vs, depending on whether ExhaustiveBackwards/ExhaustiveForwards is called.
	 */
	double GetDistValue(long v) ;
	/**
	 * @brief Similar to GetSolutionCost(), but must be called after ExhaustiveBackwards/ExhaustiveForwards.
	 */
	std::vector<double> GetPathCost(long v) ;

protected:

	virtual int _search() ;

	// Graph* _graph; // inherited

	/////////

	// long _vs;
	// long _vg;
	
	short _cdim; // the selected cost dimenion to be searched.
	short _mode; // 0 = start-goal path finding, 1 = exhaustive backwards, 2 = exhaustive forwards.
	std::vector<long> _parent; // help reconstruct the path.
	std::vector<double> _v2d; // store the results.
	std::vector<std::vector<double>> _cvec; // the corresponding cost vector of the path to vd.
};

} // end namespace search
} // end namespace zr

#endif  // ZHONGQIANGREN_BASIC_SEARCH_DIJKSTRA_H_

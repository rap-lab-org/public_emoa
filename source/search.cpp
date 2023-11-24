
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#include "search.hpp"

namespace rzq{
namespace search{

using namespace basic;


GraphSearch::GraphSearch() {};

GraphSearch::~GraphSearch() {};

void GraphSearch::SetGraphPtr(PlannerGraph* g) {
  _graph = g;
  _vs = -1; // every time when the graph is changed, reset start and goals.
  _vg = -1;
  _mode = 0;
};

} // end namespace search
} // end namespace zr

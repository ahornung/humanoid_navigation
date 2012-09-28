// SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_navigation/footstep_planner/src/PlanningStateChangeQuery.cpp $
// SVN $Id: PlanningStateChangeQuery.cpp 2159 2011-11-09 20:22:25Z garimort.johannes $

#include <footstep_planner/PlanningStateChangeQuery.h>


namespace footstep_planner
{
PlanningStateChangeQuery::PlanningStateChangeQuery(
    const std::vector<int>& neighbors)
: ivNeighbors(neighbors)
{}


PlanningStateChangeQuery::~PlanningStateChangeQuery()
{}


const std::vector<int>*
PlanningStateChangeQuery::getPredecessors() const
{
  return &ivNeighbors;
}


const std::vector<int>*
PlanningStateChangeQuery::getSuccessors() const
{
  return &ivNeighbors;
}
}

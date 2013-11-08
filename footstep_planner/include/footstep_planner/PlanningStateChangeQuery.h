/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FOOTSTEP_PLANNER_PLANNINGSTATECHANGEQUERY_H_
#define FOOTSTEP_PLANNER_PLANNINGSTATECHANGEQUERY_H_


#include <vector>

#include <sbpl/headers.h>


namespace footstep_planner
{
class PlanningStateChangeQuery : public StateChangeQuery
{
public:
  PlanningStateChangeQuery(const std::vector<int>& neighbors);
  virtual ~PlanningStateChangeQuery();

  const std::vector<int>* getPredecessors() const;
  const std::vector<int>* getSuccessors() const;

private:
  const std::vector<int>& ivNeighbors;
};
}

#endif  // FOOTSTEP_PLANNER_PLANNINGSTATECHANGEQUERY_H_

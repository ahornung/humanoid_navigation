// SVN $HeadURL$
// SVN $Id$

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

#include <footstep_planner/Heuristic.h>

namespace footstep_planner
{
	Heuristic::Heuristic(HeuristicType type)
		: ivHeuristicType(type)
	{}


	Heuristic::~Heuristic()
	{}


	EuclideanHeuristic::EuclideanHeuristic()
		: Heuristic(EUCLIDEAN)
	{}


	EuclideanHeuristic::~EuclideanHeuristic()
	{}


	double
	EuclideanHeuristic::getHValue(const PlanningState& from, const PlanningState& to)
	const
	{
		if (from == to)
			return 0;

		return euclidean_distance(from.getX(), from.getY(), to.getX(), to.getY());
	}


	EuclStepCostHeuristic::EuclStepCostHeuristic(int step_cost,
	                                             int max_step_width)
		: Heuristic(EUCLIDEAN_STEPCOST),
		  ivStepCost(step_cost),
		  ivMaxStepWidth(max_step_width)
	{}


	EuclStepCostHeuristic::~EuclStepCostHeuristic()
	{}


	double
	EuclStepCostHeuristic::getHValue(const PlanningState& from, const PlanningState& to)
	const
	{
		if (from == to)
			return 0;

        double dist = euclidean_distance(from.getX(), from.getY(),
                                         to.getX(), to.getY());
		int expectedSteps = (int)(dist / ivMaxStepWidth);

		return dist + expectedSteps * ivStepCost;
	}
}

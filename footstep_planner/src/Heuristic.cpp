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

namespace footstep_planner{

	Heuristic::Heuristic(HeuristicType type)
		: ivHeuristicType(type)
	{}


	Heuristic::~Heuristic()
	{}


	EuclideanHeuristic::EuclideanHeuristic(HeuristicType type)
		: Heuristic(type)
	{}


	EuclideanHeuristic::~EuclideanHeuristic()
	{}


	float
	EuclideanHeuristic::getHValue(const State& from, const State& to)
	const
	{

		if (from == to)
			return 0;

		return euclideanDistance(from.getX(), from.getY(), to.getX(), to.getY());

	}


	EuclStepCostHeuristic::EuclStepCostHeuristic(HeuristicType type,
	                                             float stepCosts,
	                                             float maxStepWidth)
		: Heuristic(type),
		  ivStepCosts(stepCosts),
		  ivMaxStepWidth(maxStepWidth)
	{}


	EuclStepCostHeuristic::~EuclStepCostHeuristic()
	{}


	float
	EuclStepCostHeuristic::getHValue(const State& from, const State& to)
	const
	{

		if (from == to)
			return 0;

		float distance = euclideanDistance(from.getX(), from.getY(), to.getX(), to.getY());
		int expectedSteps = (int)(distance*1.5 / ivMaxStepWidth);

		return distance + expectedSteps * ivStepCosts;

	}

}

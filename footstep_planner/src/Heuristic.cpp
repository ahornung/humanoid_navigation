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
	Heuristic::Heuristic(double cellSize, HeuristicType type)
		: ivCellSize (cellSize),
		  ivHeuristicType(type)
	{}


	Heuristic::~Heuristic()
	{}


	EuclideanHeuristic::EuclideanHeuristic(double cellSize)
		: Heuristic(cellSize, EUCLIDEAN)
	{}


	EuclideanHeuristic::~EuclideanHeuristic()
	{}


	double
	EuclideanHeuristic::getHValue(const PlanningState& from, const PlanningState& to)
	const
	{
		if (from == to)
			return 0;

		// in meter
		return euclidean_distance(disc_2_cont(from.getX(), ivCellSize),
									disc_2_cont(from.getY(), ivCellSize),
									disc_2_cont(to.getX(), ivCellSize),
									disc_2_cont(to.getY(), ivCellSize));

	}


	EuclStepCostHeuristic::EuclStepCostHeuristic(double cellSize,
												 double step_cost,
	                                             double max_step_width)
		: Heuristic(cellSize, EUCLIDEAN_STEPCOST),
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

		// in meter
        double dist = euclidean_distance(disc_2_cont(from.getX(), ivCellSize),
        									disc_2_cont(from.getY(), ivCellSize),
        									disc_2_cont(to.getX(), ivCellSize),
        									disc_2_cont(to.getY(), ivCellSize));
		double expectedSteps = (dist / ivMaxStepWidth);

		return (dist + expectedSteps * ivStepCost);
	}
}

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
	Heuristic::Heuristic(double cell_size, int num_angle_bins,
	                     HeuristicType type)
		: ivCellSize(cell_size),
		  ivNumAngleBins(num_angle_bins),
		  ivHeuristicType(type)
	{}


	Heuristic::~Heuristic()
	{}


	EuclideanHeuristic::EuclideanHeuristic(double cell_size, int num_angle_bins)
		: Heuristic(cell_size, num_angle_bins, EUCLIDEAN)
	{}


	EuclideanHeuristic::~EuclideanHeuristic()
	{}


	double
	EuclideanHeuristic::getHValue(const PlanningState& from,
	                              const PlanningState& to)
	const
	{
		if (from == to)
			return 0;

		// in meter
		return euclidean_distance(cell_2_state(from.getX(), ivCellSize),
		                          cell_2_state(from.getY(), ivCellSize),
		                          cell_2_state(to.getX(), ivCellSize),
		                          cell_2_state(to.getY(), ivCellSize));

	}


	EuclStepCostHeuristic::EuclStepCostHeuristic(double cell_size,
	                                             int    num_angle_bins,
												 double step_cost,
                                                 double diff_angle_cost,
	                                             double max_step_width)
		: Heuristic(cell_size, num_angle_bins, EUCLIDEAN_STEPCOST),
		  ivStepCost(step_cost),
		  ivDiffAngleCost(diff_angle_cost),
		  ivMaxStepWidth(max_step_width)
	{}


	EuclStepCostHeuristic::~EuclStepCostHeuristic()
	{}


	double
	EuclStepCostHeuristic::getHValue(const PlanningState& from,
	                                 const PlanningState& to)
	const
	{
		if (from == to)
			return 0;

		// in meter
        double dist = euclidean_distance(cell_2_state(from.getX(), ivCellSize),
                                         cell_2_state(from.getY(), ivCellSize),
                                         cell_2_state(to.getX(), ivCellSize),
                                         cell_2_state(to.getY(), ivCellSize));
		double expected_steps = dist / ivMaxStepWidth;
		// we could replace this by working on ints (w. all normalization)
		// int disc_diff_angle = abs(from.getTheta() - to.getTheta());
		// double diff_angle = angle_disc_2_cont(disc_diff_angle, ivNumAngleBins);
		double diff_angle = 0.0;
		if (ivDiffAngleCost > 0.0)
		{
		    diff_angle = std::abs(angles::shortest_angular_distance(
		            angle_cell_2_state(from.getTheta(), ivNumAngleBins),
		            angle_cell_2_state(to.getTheta(), ivNumAngleBins)));
		}

		return (dist + expected_steps*ivStepCost + diff_angle*ivDiffAngleCost);
	}
}

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

#ifndef HEURISTIC_H_
#define HEURISTIC_H_

#include <footstep_planner/helper.h>
#include <footstep_planner/State.h>

namespace footstep_planner
{

	/**
	 * @brief An abstract super class providing methods necessary to be used as
	 * heuristic function within Dstar.
	 */
	class Heuristic
	{

	public:

		enum HeuristicType { EUCLIDEAN=0, EUCLIDEAN_STEPCOST=1, ASTAR_PATH=2 };


		Heuristic(HeuristicType type);
		virtual ~Heuristic();

		/**
		 * @return The heuristically determined path costs to get from
		 * state 'from' to state 'to'.
		 */
		virtual float getHValue(const State& from, const State& to) const = 0;

		HeuristicType getHeuristicType() const { return ivHeuristicType; };


	private:

		const HeuristicType ivHeuristicType;

	};


	/**
	 * @brief Determining the heuristic value by the euclidean distance between
	 * two states.
	 */
	class EuclideanHeuristic : public Heuristic
	{

	public:

		EuclideanHeuristic(HeuristicType type, int roundingThreshold);
		virtual ~EuclideanHeuristic();

		virtual float getHValue(const State& from, const State& to) const;


	private:

		const int ivRoundingThreshold;

	};


	/**
	 * @brief Determining the heuristic value by the euclidean distance between
	 * two states and the costs of the expected step costs to get from one of the
	 * states to the other.
	 */
	class EuclStepCostHeuristic : public Heuristic
	{

	public:

		EuclStepCostHeuristic(HeuristicType type,
		                      int roundingThreshold,
		                      float stepCosts,
		                      float maxStepWidth);
		virtual ~EuclStepCostHeuristic();

		virtual float getHValue(const State& from, const State& to) const;


	private:

		const int   ivRoundingThreshold;
		const float ivStepCosts;
		const float ivMaxStepWidth;

	};
}


#endif /* HEURISTIC_H_ */

// SVN $HeadURL$
// SVN $Id$

/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 * D* Lite (Koenig et al. 2002) partly based on the implementation
 * by J. Neufeld (http://code.google.com/p/dstarlite/)
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

#ifndef DSTAR_H
#define DSTAR_H

#include <boost/tr1/unordered_map.hpp>
#include <footstep_planner/helper.h>
#include <footstep_planner/Heuristic.h>
#include <footstep_planner/State.h>
#include <gridmap_2d/GridMap2D.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <ros/ros.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <vector>

namespace footstep_planner{


	class Footstep
	{

	public:

		Footstep();
		Footstep(float x, float y, float theta, Leg leg = NOLEG);
		virtual ~Footstep();

		// performing a footstep means rotating and translating the step leg with
		// respect to the current robot's state
		void performMeOnThisState(const State& current, State* successor, float footSeparation) const;
		void revertMeOnThisState(const State& current, State* predecessor, float footSeparation) const;

		void setLeg(Leg leg) { ivLeg = leg; };


	private:

		// translation and rotation parameters
		float	ivX;
		float	ivY;
		float	ivTheta;
		Leg		ivLeg;

	};


	class Dstar
	{

	public:

		typedef std::priority_queue<State, std::vector<State>, std::greater<State> > openlist_type;
		typedef boost::unordered_map<State, State::state_info, State::state_hash, std::equal_to<State> > statehash_type;
		typedef boost::unordered_map<State, float, State::state_hash, std::equal_to<State> > openhash_type;

		typedef std::vector<State>::const_iterator stateIterator;

		Dstar(const std::vector<Footstep>& footstepSet,
				const float footSeparation,
				const float footOriginXShift,
				const float footOriginYShift,
				const float footWidth,
				const float footHeight,
				const float maxShiftX,
				const float maxShiftY,
				const float maxTurn,
				const float maxInverseShiftY,
				const float maxInnerTurn,
				const float maxStepWidth,
				const float stepCost,
				const int   collCheckAccuracy,
				const int   stateEqualityCutoff,
				const int   plannerMaxSteps,
				const boost::shared_ptr<const Heuristic>& heuristicConstPtr);
		virtual ~Dstar();

		bool closeSteps(Leg stepLeg, tf::Transform& supportFoot, tf::Transform& footPlacement);
		bool setUp(const State& startFootLeft,
				const State& startFootRight,
				const State& goal);
		bool isCloseToGoal(const State& s) const;
		void updateStart(const State& startFootLeft, const State& startFootRight);

		/**
		 * The main planning / replanning function.
		 * Updates the costs for all cells and computes the shortest path to
		 * goal. The path is computed by doing a greedy search over the cost+g values of each
		 * state.
		 *
		 * @return true if a path is found, false otherwise.
		 */
		bool replan();

		void setGoal(const State& goal);

		/// resets the Dstar planner, discarding all replanning information
		void reset();

		stateIterator getPathBegin() const { return ivPath.begin(); };
		stateIterator getPathEnd() const { return ivPath.end(); };

		stateIterator getExpandedBegin() const { return ivExpandedStates.begin(); };
		stateIterator getExpandedEnd() const { return ivExpandedStates.end(); };

		/// receive the successor state with the smallest g value
		bool  getMinSucc(const State u, State* succ);
		/// return true if the foot in state u would collide with an obstacle
		bool  occupied(const State& u) const;
		void  updateDistanceMap(boost::shared_ptr<GridMap2D> map);

		static int cvRoundingThreshold;


	private:

		std::vector<State> ivExpandedStates;
		std::vector<State> ivPath;

		float ivKM;
		State ivGoal;
		State ivLast;
		State ivStart;
		State ivStartStateLeft;
		State ivStartStateRight;

		const std::vector<Footstep> ivFootstepSet;
		const float ivFootSeparation;
		const float ivFootOriginShiftX, ivFootOriginShiftY;
		const float ivFootsizeX, ivFootsizeY;
		const float ivMaxShiftX, ivMaxShiftY, ivMaxTurn;
		const float ivMaxInverseShiftY, ivMaxInnerTurn; // ivMaxInverseShiftX = ivMaxShiftX
		const float ivMaxStepWidth;
		const float ivStepCost;
		const int   ivCollisionCheckAccuracy;
		const int	ivPlannerMaxSteps;

		const boost::shared_ptr<const Heuristic> ivHeuristicConstPtr;

		boost::shared_ptr<GridMap2D> ivMapPtr;

		/// priority queue of states (can contain equal states)
		openlist_type ivOpenList;
		/// mapping of states to their currently known rhs and g values
		statehash_type ivStateHash;
		/// the most up to date state in the priority queue is identified with an unique hash value from this hash map
		openhash_type ivOpenHash;

		void  addState(const State& u);
		int   computeShortestPath();
		float cost(const State& a, const State& b) const;
		float getG(const State& u) const;
		void  getPredecessors(const State& u, std::vector<State>* s);
		void  getSuccessors(const State& u, std::vector<State>* s);
		float getRhs(const State& u) const;
		void  insert(State& u);
		bool  isCloseToStart(const State& s) const;
		bool  isValid(const State& u) const;
		float keyHashCode(const State& u) const;
		bool  reachable(const State& from, const State& to) const;
		void  remove(const State& u);
		void  setG(const State& u, float g);
		void  setRhs(const State& u, float rhs);
		void  update(State& u);
		void  updateRhs(const State& pred, const State& succ, float g_old);

		State::key calculateKey(const State& u) const;

	};

} // end of namespace


#endif

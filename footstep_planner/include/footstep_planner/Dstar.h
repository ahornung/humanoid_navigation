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
#include <footstep_planner/Astar.h>
#include <footstep_planner/helper.h>
#include <footstep_planner/Heuristic.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <ros/ros.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <vector>

namespace footstep_planner
{

	/**
	 * @brief A class representing a footstep (i.e. a translation and rotation
	 * of a specific foot) that can be performed by a humanoid robot.
	 */
	class Footstep
	{

	public:

		Footstep();
		Footstep(float x, float y, float theta, Leg leg = NOLEG);
		virtual ~Footstep();

		/**
		 * @brief Performs the footstep within the current state to receive the
		 * successor state.
		 *
		 * @param current state of the support leg
		 * @param successor state of the opposite leg (with respect to the support
		 * leg) after performing the footstep
		 * @param footSeparation standard separation of both feet in initial position
		 */
		void performMeOnThisState(const State& current, State* successor, float footSeparation) const;

		/**
		 * @brief Reverts the footstep within the current state to receive the
		 * predecessor state.
		 *
		 * @param current state of the support leg
		 * @param predecessor state of the opposite leg (with respect to the support
		 * leg) after reverting the footstep
		 * @param footSeparation standard separation of both feet in initial position
		 */
		void revertMeOnThisState(const State& current, State* predecessor, float footSeparation) const;


	private:

		/// translation on the x axis (w.r. to the support leg)
		float ivX;
		/// translation on the y axis (w.r. to the support leg)
		float ivY;
		/// rotation (w.r. to the support leg)
		float ivTheta;
		/// support leg
		Leg   ivLeg;

	};


	/**
	 * @brief The footstep planning algorithm calculating a footstep path from
	 * a given start pose to a goal pose in an environment with planar obstacles.
	 * Allows efficient replanning due to smart reusage of information collected
	 * in previous planning tasks.
	 */
	class Dstar
	{

	public:

		typedef std::priority_queue<State, std::vector<State>, std::greater<State> > openlist_type;
		typedef boost::unordered_map<State, State::state_info, State::state_hash, std::equal_to<State> > statehash_type;
		typedef boost::unordered_map<State, float, State::state_hash, std::equal_to<State> > openhash_type;

		typedef std::vector<State>::const_iterator stateIterator;


		Dstar(const std::vector<Footstep>& footstepSet,
		      const float footSeparation,
		      const float footOriginShiftX,
		      const float footOriginShiftY,
		      const float footsizeX,
		      const float footsizeY,
		      const float footMaxStepX,
		      const float footMaxStepY,
		      const float footMaxStepTheta,
		      const float footMaxInverseStepY,
		      const float footMaxInverseStepTheta,
		      const float maxStepWidth,
		      const float stepCosts,
		      const int   collisionCheckAccuracy,
		      const int   roundingThreshold,
		      const int   plannerMaxSteps,
		      const boost::shared_ptr<Heuristic> heuristicConstPtr);
		virtual ~Dstar();

		/**
		 * @brief Init Dstar with start and goal coordinates, rest is as per
		 * [S. Koenig, 2002].
		 *
		 * @return true if the initialization was successful
		 */
		bool setUp(const State& startFootLeft,
		           const State& startFootRight,
		           const State& goal);

		/**
		 * @return true if state s is close to the goal state
		 */
		bool isCloseToGoal(const State& s) const;

		/**
		 * @brief Update the pose of the robot, this does not force a replan.
		 */
		void updateStart(const State& startFootLeft, const State& startFootRight);

		/**
		 * @brief Updates the costs for all cells and computes the shortest path
		 * to goal. The path is computed by doing a greedy search over the
		 * cost + g values of each state.
		 *
		 * @return true if a path is found, false otherwise.
		 */
		bool replan();

		/**
		 * @brief Update the goal pose. All previously collected planning
		 * information will be discarded.
		 */
		void setGoal(const State& goal);

		/**
		 * @brief Resets the Dstar planner discarding all planning information.
		 */
		void reset();

		stateIterator getPathBegin() const { return ivPath.begin(); };
		stateIterator getPathEnd() const { return ivPath.end(); };

		stateIterator getExpandedBegin() const { return ivExpandedStates.begin(); };
		stateIterator getExpandedEnd() const { return ivExpandedStates.end(); };

		/**
		 * @brief Receive the successor state with the smallest g value.
		 */
		bool getMinSucc(const State u, State* succ);

		/**
		 * @brief Update the distance map. Formerly unoccupied states are set
		 * locally inconsistent and vice versa.
		 */
		void updateDistanceMap(boost::shared_ptr<GridMap2D> map);

		/**
		 * @return Costs of the footstep path.
		 */
		float getPathCosts() { return ivPathCosts; };

		/**
		 * @return Number of expanded states.
		 */
		size_t getNumExpandedStates() const { return ivExpandedStates.size(); };

		/**
		 * @return Number of necessary footsteps.
		 */
		size_t getNumFootsteps() const { return ivPath.size(); };

		static int cvRoundingThreshold;


	private:

		std::vector<State> ivExpandedStates;
		std::vector<State> ivPath;

		float ivKM;
		float ivPathCosts;
		State ivGoal;
		State ivLast;
		State ivStart;
		State ivStartStateLeft;
		State ivStartStateRight;

		const std::vector<Footstep> ivFootstepSet;
		const float ivFootSeparation;
		const float ivFootOriginShiftX, ivFootOriginShiftY;
		const float ivFootsizeX, ivFootsizeY;
		const float ivFootMaxStepX, ivFootMaxStepY, ivFootMaxStepTheta;
		const float ivFootMaxInverseStepY, ivFootMaxInverseStepTheta; // ivFootMaxInverseStepX = ivFootMaxStepX
		const float ivMaxStepWidth;
		const float ivStepCosts;
		const int   ivCollisionCheckAccuracy;
		const int	ivPlannerMaxSteps;
		const boost::shared_ptr<Heuristic> ivHeuristicConstPtr;

		boost::shared_ptr<GridMap2D> ivMapPtr;

		/**
		 * @brief The Open list of states containing these states that are
		 * locally inconsistent (rhs(state) != g(state)). Due to lacking direct
		 * access to arbitrary states the actuality of states is organized in an
		 * extra open hash list. Therefore the open list can contain several
		 * states representing the same pose.
		 */
		openlist_type ivOpenList;

		/**
		 * @brief Mapping of states to their currently known rhs and g values.
		 */
		statehash_type ivStateHash;

		/**
		 * @brief The most up to date state in the open list is identified
		 * with an unique hash value from this hash map.
		 */
		openhash_type ivOpenHash;


		/**
		 * @brief Checks if a state is in the state hash, if not it is added.
		 */
		void addState(const State& u);

		/**
		 * @brief As per [S. Koenig, 2002] except for 2 main modifications:
		 * 1. We stop planning after a number of steps. We do this because this
		 *    algorithm can plan forever if the start is surrounded by obstacles.
		 * 2. We lazily remove states from the open list so we never have to
		 *    iterate through it (usage of open hash instead).
		 */
		int computeShortestPath();

		/**
		 * @return true if the foot placement can be reached by the step leg
		 * by a feasible footstsep
		 */
		bool closeSteps(Leg stepLeg, tf::Transform& supportFoot, tf::Transform& footPlacement);

		/**
		 * @return the cost of moving from state a to state b.
		 */
		float cost(const State& a, const State& b) const;

		/**
		 * @return the currently known g value for state u
		 */
		float getG(const State& u) const;

		/**
		 * @brief Ćalculates a list of all the predecessor states for state u,
		 * e.g. all the states the robot ends at after performing the set of
		 * footsteps. Invalid predecessors are discarded.
		 */
		void getPredecessors(const State& u, std::vector<State>* s);

		/**
		 * @brief Calculates a list of successor states for state u unless the
		 * state is occupied in which case it has no successors.
		 */
		void getSuccessors(const State& u, std::vector<State>* s);

		/**
		 * @return the currently know rhs value for state u.
		 */
		float getRhs(const State& u) const;

		/**
		 * @brief Inserts state u into the open list and open hash. If already
		 * inserted the state is just updated.
		 */
		void insert(State& u);

		/**
		 * @brief Check if a feasible footstep exists to reach s from one of the
		 * start states (i.e. the left and right foot).
		 */
		bool isCloseToStart(const State& s) const;

		/**
		 * @return true if state u is in the open list and if it is up to date
		 * (since the open list can contain expired states).
		 */
		bool isValid(const State& u) const;

		/**
		 * @brief Calculate the key hash code for the state u. This is used to
		 * compare the actuality of states (since the open list can contain
		 * expired states).
		 */
		float keyHashCode(const State& u) const;

		/**
		 * @return true if a foot placement in state u collides with an obstacle
		 */
		bool occupied(const State& u) const;

		/**
		 * @return true if state to can be reached from state from by a feasible
		 * footstep
		 */
		bool reachable(const State& from, const State& to) const;

		/**
		 * @brief Removes state u from the open hash. The state is removed from
		 * the open list lazily (in Dstar::computeShortestPath()) to save
		 * computation time.
		 */
		void remove(const State& u);

		/**
		 * @brief Sets the g value for state u (update of the state hash).
		 */
		void setG(const State& u, float g);

		/**
		 * @brief Sets the rhs value for state u (update of the state hash).
		 */
		void setRhs(const State& u, float rhs);

		/**
		 * @brief Update state u as per [S. Koenig, 2002] (optimized version).
		 */
		void update(State& u);

		/**
		 * @brief Update somehow the heuristic function from outside; here: used
		 * to calculate a new subgoal path for the A* heuristic.
		 */
		bool updateHeuristicValues();

		/**
		 * @brief Method for the optimized D* lite as per [S. Koenig, 2002]
		 * figure 4 {26'-27'}.
		 */
		void updateRhs(const State& pred, const State& succ, float g_old);

		/**
		 * As per [S. Koenig, 2002].
		 *
		 * @return key.first = min{g(s), rhs(s)} + h(s_start, s) + km,
		 * key.second = min{g(s), rhs(s)} (based on the currently know rhs and
		 * g values)
		 */
		State::key calculateKey(const State& u) const;

	};

} // end of namespace


#endif

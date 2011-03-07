// SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_navigation/footstep_planner/include/footstep_planner/Dstar.h $
// SVN $Id: Dstar.h 855 2011-02-01 10:08:48Z hornunga@informatik.uni-freiburg.de $

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

#ifndef ASTAR_H
#define ASTAR_H

#include <boost/tr1/unordered_map.hpp>
#include <boost/tr1/unordered_set.hpp>
#include <footstep_planner/helper.h>
#include <footstep_planner/Heuristic.h>
#include <gridmap_2d/GridMap2D.h>
#include <list>
#include <math.h>
#include <queue>
#include <ros/ros.h>
#include <vector>


namespace footstep_planner
{

	/**
	 * @brief Determining the heuristic value by a list of subgoals. The euclidean
	 * distance to the next subgoal plus the path along each subgoal to the real
	 * goal is used as goal distance. Additionally this distance is used to calculate
	 * the expected step costs.
	 * The grid cell path from which the subgoals are extracted is calculated
	 * with an A* search.
	 * NOTE: This heuristic potentially overestimates the real costs. Anyway if
	 * the robot -- due to a small step width -- cannot step over obstacles this
	 * heuristic is also admissible.
	 */
	class AstarHeuristic : public Heuristic
	{

		/**
		 * @brief Data structure to represent search states.
		 */
		class SearchNode
		{

		public:

			/// gridcell.first and gridcell.second correspond to x and y of the grid cell
			typedef std::pair<unsigned int, unsigned int> gridcell;
			typedef boost::shared_ptr<const SearchNode> searchnode_ptr;


			SearchNode();
			SearchNode(const gridcell& cell, const State::key& key);
			SearchNode(const gridcell& cell,
					   const State::key& key,
					   const searchnode_ptr predecessor);
			virtual ~SearchNode();

			const gridcell& getCell() const { return ivCell; };
			const State::key& getKey() const { return ivKey; };
			const searchnode_ptr getPredecessor() const { return ivPredecessor; };


		private:

			gridcell   ivCell;
			/// key.first represents the g value, key.second represents the h value
			State::key ivKey;

			searchnode_ptr ivPredecessor;


		public:

			/// Comparison of two search nodes used within the closed list.
			struct equal_to
			{
				bool operator ()(searchnode_ptr n1, searchnode_ptr n2) const;
			};

			/// Comparison of two search nodes used within the priority queue.
			struct greater
			{
				bool operator ()(searchnode_ptr n1, searchnode_ptr n2) const;
			};

			/// Comparison operators used within the distance hash map.
			struct gridcell_hash
			{
				size_t operator ()(searchnode_ptr n) const;
				size_t operator ()(const SearchNode::gridcell& c) const;
			};

		};


	public:

		/// The priority queue sorting the search nodes by their f-value.
		typedef std::priority_queue<SearchNode::searchnode_ptr, std::vector<SearchNode::searchnode_ptr >, SearchNode::greater > openlist_type;

		/// The closed list containing already expanded search nodes.
		typedef boost::unordered_set<SearchNode::searchnode_ptr, SearchNode::gridcell_hash, SearchNode::equal_to > closedlist_type;

		/// A hash map pointing from a search node to their currently known g-value.
		typedef boost::unordered_map<SearchNode::gridcell, float, SearchNode::gridcell_hash, std::equal_to<SearchNode::gridcell> > distancelist_type;

		typedef std::list<SearchNode::gridcell>::const_iterator gridcelllist_iter;

		typedef std::pair<float, float> coordinate;
		typedef std::list<std::pair<coordinate, float> > subgoaldistances_type;


		AstarHeuristic(HeuristicType type, float stepCosts, float maxStepSize, float distanceThreshold, float subgoalDistance);
		virtual ~AstarHeuristic();

		/// Returning the heuristic value for the two states.
		virtual float getHValue(const State& from, const State& to) const;

		/// Method used to calculate a path from state 'from' to state 'to'.
		bool astarPlanning(const State& from, const State& to);

		void setMap(boost::shared_ptr<GridMap2D> map);


	private:

		const float ivDistanceThreshold;
		const float ivMaxStepSize;
		const float ivStepCosts;
		const float ivSubgoalDistance;

		subgoaldistances_type ivSubgoalGoalDistances;

		boost::shared_ptr<GridMap2D> ivMapPtr;

		SearchNode::gridcell ivStart;
		SearchNode::gridcell ivGoal;

		/**
		 * @return The costs to get from one grid cell simply based on the
		 * euclidean distance.
		 */
		float cost(const SearchNode::gridcell& c1, const SearchNode::gridcell& c2) const;

		/**
		 * @brief Method used to extract the calculated path.
		 *
		 * @return True if the extraction was successful.
		 */
		bool  extractPath(SearchNode::searchnode_ptr extractionStart);

		/// Receive the successor grid cells.
		void  getSuccessors(SearchNode::gridcell c, std::vector<SearchNode::gridcell>& successors);

		/// @return The distance between two grid cells.
		float gridDistance(const SearchNode::gridcell& c1, const SearchNode::gridcell& c2) const;

		/// @return True if grid cell is close to the goal cell.
		bool  isCloseToGoal(const SearchNode::gridcell& c) const;

		/// @return True if occupied.
		bool  occupied(const SearchNode::gridcell& c) const;

	};

} // end of namespace


#endif

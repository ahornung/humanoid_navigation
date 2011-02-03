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

	class SearchNode
	{

	public:

		/// gridcell.first and gridcell.second correspond to x and y of the grid cell
		typedef std::pair<unsigned int, unsigned int> gridcell;

		SearchNode();
		SearchNode(const gridcell& cell, const State::key& key, const SearchNode* const predecessor);
		virtual ~SearchNode();

		bool   operator  >(const SearchNode& s) const;
		bool   operator ==(const SearchNode& s) const;

		const gridcell& getCell() const { return ivCell; };
		const State::key& getKey() const { return ivKey; };
		const SearchNode& getPredecessor() const { return *ivPredecessor; };


	private:

		gridcell   ivCell;
		/// key.first represents the g value, key.second represents the h value
		State::key ivKey;

		const SearchNode* ivPredecessor;


	public:

		struct gridcell_hash
		{

			size_t operator ()(const SearchNode n) const { return 19*n.getCell().first + 13*n.getCell().second; };
			size_t operator ()(const SearchNode::gridcell& c) const { return 19*c.first + 13*c.second; };

		};

	};


	class AstarHeuristic : public Heuristic
	{

	public:

		typedef std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode> > openlist_type;
		typedef boost::unordered_set<SearchNode, SearchNode::gridcell_hash, std::equal_to<SearchNode> > closedlist_type;
		typedef boost::unordered_map<SearchNode::gridcell, float, SearchNode::gridcell_hash, std::equal_to<SearchNode::gridcell> > distancelist_type;

		typedef std::list<SearchNode::gridcell>::const_iterator gridcellListIter;

		AstarHeuristic(float stepCosts, float maxStepWidth, float distanceThreshold);
		virtual ~AstarHeuristic();

		virtual float getHValue(const State& from, const State& to) const;

		bool astarPlanning(const State& from, const State& to);
		void setMap(boost::shared_ptr<GridMap2D> map);

		gridcellListIter getPathBegin() const { return ivPath.begin(); };
		gridcellListIter getPathEnd() const { return ivPath.end(); };


	private:

		const float ivDistanceThreshold;
		const float ivMaxStepWidth;
		const float ivStepCosts;

		boost::shared_ptr<GridMap2D> ivMapPtr;
		std::list<SearchNode::gridcell> ivPath;

		distancelist_type ivSubgoalGoalDistance;

		SearchNode::gridcell ivStart;
		SearchNode::gridcell ivGoal;

		float distance(const SearchNode::gridcell& c1, const SearchNode::gridcell& c2) const;
		bool  extractPath(const SearchNode& extractionStart);
		void  getSuccessors(SearchNode::gridcell c, std::vector<SearchNode::gridcell>& successors);
		bool  isCloseToGoal(const SearchNode::gridcell& cell) const;
		bool  occupied(const SearchNode::gridcell& c) const;

	};

} // end of namespace


#endif

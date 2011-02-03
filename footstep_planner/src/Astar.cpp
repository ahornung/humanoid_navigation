// SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_navigation/footstep_planner/src/Dstar.cpp $
// SVN $Id: Dstar.cpp 855 2011-02-01 10:08:48Z hornunga@informatik.uni-freiburg.de $

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

#include <footstep_planner/Astar.h>


namespace footstep_planner
{

	/*
	 * #########################################################################
	 * #### class AstarHeuristic
	 * #########################################################################
	 */

	AstarHeuristic::AstarHeuristic(float stepCosts, float maxStepWidth, float distanceThreshold)
		: ivDistanceThreshold(distanceThreshold),
		  ivMaxStepWidth(maxStepWidth),
		  ivStepCosts(stepCosts)
	{}


	AstarHeuristic::~AstarHeuristic()
	{}


	float
	AstarHeuristic::getHValue(const State& from, const State& to)
	const
	{

		if (from == to)
			return 0;

		if (!ivSubgoalGoalDistance.size() <= 0)
		{
			ROS_ERROR("Subgoal distances haven't been calculated yet.");
			return -INFINITY;
		}

		float x, y;
		SearchNode::gridcell subgoal;
		float minDistance = INFINITY;
		float h = INFINITY;
		distancelist_type::const_iterator iter = ivSubgoalGoalDistance.begin();
		// receive the nearest subgoal for state 'from'
		for(; iter != ivSubgoalGoalDistance.end(); iter++)
		{
			double wx, wy;
			ivMapPtr->mapToWorld(iter->first.first, iter->first.second, wx, wy);
			x = from.getX() - wx;
			y = from.getY() - wy;
			float distanceSqr = x*x + y*y;
			if (minDistance > distanceSqr)
			{
				minDistance = distanceSqr;
				subgoal = iter->first;
				h = iter->second;
			}
		}
		// check if state 'to' might be the nearest subgoal..
		x = from.getX() - to.getX();
		y = from.getY() - to.getY();
		// ..then the h value is 0
		if (minDistance > (x*x + y*y))
		{
			h = 0;
		}
		// otherwise receive distances to the nearest subgoal
		else
		{
			x = from.getX() - subgoal.first;
			y = from.getY() - subgoal.second;
		}

		// calculate expected step costs to the nearest subgoal
		float distance = sqrt(x*x + y*y) + h;
		int expectedSteps = (int)(distance*2 / ivMaxStepWidth);

		return distance + expectedSteps * ivStepCosts;

	}


	bool
	AstarHeuristic::astarPlanning(const State& from, const State& to)
	{

		if (!ivMapPtr)
		{
			ROS_ERROR("Distance map hasn't been submitted yet.");
			return false;
		}

		openlist_type     openList;
		closedlist_type   closedList;
		distancelist_type distanceList;
		closedlist_type::const_iterator closedListIter;
		std::vector<SearchNode::gridcell> successors;
		std::vector<SearchNode::gridcell>::iterator successorIter;
		std::pair<closedlist_type::iterator, bool> closeListPair;

		ivMapPtr->worldToMap(from.getX(), from.getY(), ivStart.first, ivStart.second);
		ivMapPtr->worldToMap(to.getX(), to.getY(), ivGoal.first, ivGoal.second);
		State::key key(0, distance(ivStart, ivGoal));
		SearchNode cur(ivStart, key, NULL);
		openList.push(cur);

		while (!openList.empty())
		{
			cur = openList.top();
			SearchNode::gridcell curCell = cur.getCell();
			State::key curKey = cur.getKey();
			openList.pop();
			closedListIter = closedList.find(cur);
			bool distSmaller = true;
			if (distanceList.find(curCell) != distanceList.end())
				distSmaller = (curKey.first < distanceList[curCell]);
			if (closedListIter == closedList.end() && distSmaller)
			{
				closeListPair = closedList.insert(cur);
				// TODO: test this - seems to work
				const SearchNode& predecessor = *(closeListPair.first);
				distanceList[curCell] = curKey.first;
				if (isCloseToGoal(curCell))
				{
					return extractPath(cur);
				}
				getSuccessors(curCell, successors);
				successorIter = successors.begin();
				for (; successorIter != successors.end(); successorIter++)
				{
					float g = curKey.first + distance(curCell, *successorIter);
					float h = distance(*successorIter, ivGoal);
					SearchNode n(*successorIter, State::key(g, h), &predecessor);
					openList.push(n);
				}
			}
		}

		return false;

	}


	bool
	AstarHeuristic::extractPath(const SearchNode& extractionStart)
	{

		ivPath.clear();

		float pathLength = 0;

		SearchNode curNode = extractionStart;
		SearchNode pred;
		SearchNode::gridcell curCell = curNode.getCell();
		while(curCell != ivStart)
		{
			ivPath.push_front(curCell);
			pred = curNode.getPredecessor();
			pathLength = distance(curCell, pred.getCell());
			curNode = pred;
		}
		pathLength *= ivMapPtr->getInfo().resolution;

		float subgoalDistance = 0;
		float subPathLength = 0;

		const float subgoalMinDistance = 0.0;

		gridcellListIter pathIter = ivPath.begin();
		ivSubgoalGoalDistance.insert(std::pair<SearchNode::gridcell, float>(*pathIter, pathLength));
		for(; pathIter != ivPath.end(); )
		{
			SearchNode::gridcell curCell = *pathIter;
			pathIter++;
			subgoalDistance += ivMapPtr->worldDist(curCell.first,
												   curCell.second,
												   pathIter->first,
												   pathIter->second);
			if (subgoalDistance > subgoalMinDistance)
			{
				subPathLength += subgoalDistance;
				ivSubgoalGoalDistance.insert(std::pair<SearchNode::gridcell, float>(*pathIter, pathLength - subPathLength));
				subgoalDistance = 0;
			}
		}

		return true;

	}


	void
	AstarHeuristic::getSuccessors(SearchNode::gridcell c, std::vector<SearchNode::gridcell>& successors)
	{

		successors.clear();

		if (occupied(c))
			return;

		c.first += 1;
		successors.push_back(c);
		c.second += 1;
		successors.push_back(c);
		c.first -= 1;
		successors.push_back(c);
		c.first -= 1;
		successors.push_back(c);
		c.second -= 1;
		successors.push_back(c);
		c.second -= 1;
		successors.push_back(c);
		c.first += 1;
		successors.push_back(c);
		c.first += 1;
		successors.push_back(c);

	}


	void
	AstarHeuristic::setMap(boost::shared_ptr<GridMap2D> map)
	{

		ivMapPtr.reset();
		ivMapPtr = map;
		// clear old path; a new one has to be calculated
		ivPath.clear();

	}


	bool
	AstarHeuristic::occupied(const SearchNode::gridcell& c)
	const
	{

		if (ivMapPtr->isOccupiedAtCell(c.first, c.second))
			return true;

		float dist = ivMapPtr->distanceMapAtCell(c.first, c.second);
		return dist < ivDistanceThreshold;

	}


	bool
	AstarHeuristic::isCloseToGoal(const SearchNode::gridcell& cell)
	const
	{

		return cell == ivGoal;

	}


	float
	AstarHeuristic::distance(const SearchNode::gridcell& c1, const SearchNode::gridcell& c2)
	const
	{

		int xd = abs(c1.first - c2.first);
		int yd = abs(c1.second - c2.second);

		float retVal = 1;
		if (xd+yd > 1)
			retVal = M_SQRT2;

		return retVal;

	}


	/*
	 * #########################################################################
	 * #### class SearchNode
	 * #########################################################################
	 */

	SearchNode::SearchNode()
	{

		ivCell = gridcell(0, 0);
		ivKey = State::key(0, INFINITY);
		ivPredecessor = NULL;

	}


	SearchNode::SearchNode(const gridcell& cell, const State::key& key, const SearchNode* const predecessor)
		: ivCell(cell), ivKey(key), ivPredecessor(predecessor)
	{}


	SearchNode::~SearchNode()
	{}


	bool
	SearchNode::operator >(const SearchNode& s)
	const
	{

		State::key sKey = s.getKey();
		return ivKey.first + ivKey.second > sKey.first + sKey.second;

	}


	bool
	SearchNode::operator ==(const SearchNode& s)
	const
	{

		return ivCell == s.getCell();

	}

} // end of namespace

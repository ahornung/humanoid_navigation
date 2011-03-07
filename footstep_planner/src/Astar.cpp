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

	AstarHeuristic::AstarHeuristic(HeuristicType type,
	                               float stepCosts,
	                               float maxStepSize,
	                               float distanceThreshold,
	                               float subgoalDistance)
		: Heuristic(type),
	      ivDistanceThreshold(distanceThreshold),
		  ivMaxStepSize(maxStepSize),
		  ivStepCosts(stepCosts),
		  ivSubgoalDistance(subgoalDistance)
	{}


	AstarHeuristic::~AstarHeuristic()
	{}


	float
	AstarHeuristic::getHValue(const State& from, const State& to)
	const
	{

		if (from == to)
			return 0;

		if (ivSubgoalGoalDistances.size() <= 0)
		{
			ROS_ERROR("Subgoal distances haven't been calculated yet.");
			return INFINITY;
		}

		float x, y;
		coordinate subgoal;
		float minDistance = INFINITY;
		float h = INFINITY;
		subgoaldistances_type::const_iterator iter = ivSubgoalGoalDistances.begin();
		// receive the nearest subgoal for state 'from'
		for(; iter != ivSubgoalGoalDistances.end(); iter++)
		{
			x = from.getX() - iter->first.first;
			y = from.getY() - iter->first.second;
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
		if (minDistance > (x*x + y*y))
		{
			// ..then the h value is 0
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
		int expectedSteps = (int)(distance*2 / ivMaxStepSize);

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

		ivSubgoalGoalDistances.clear();

		openlist_type    openList;
		closedlist_type  closedList;
		distancelist_type distanceList;
		closedlist_type::const_iterator closedListIter;
		std::vector<SearchNode::gridcell> successors;
		std::vector<SearchNode::gridcell>::iterator successorIter;
		std::pair<closedlist_type::iterator, bool> closedListPair;

		ivMapPtr->worldToMap(from.getX(), from.getY(), ivStart.first, ivStart.second);
		ivMapPtr->worldToMap(to.getX(), to.getY(), ivGoal.first, ivGoal.second);

		State::key key(0, cost(ivStart, ivGoal));
		SearchNode::searchnode_ptr cur(new SearchNode(ivStart, key));
		openList.push(cur);

		bool success = false;
		while (!openList.empty())
		{
			cur = openList.top();
			openList.pop();
			const SearchNode::gridcell& curCell = cur->getCell();
			const State::key& curKey = cur->getKey();
			bool distSmaller = true;
			distancelist_type::iterator distIter = distanceList.find(curCell);
			if (distIter != distanceList.end())
				distSmaller = (curKey.second < distIter->second);
			closedListIter = closedList.find(cur);
			bool notInClosedList = (closedListIter == closedList.end());
			if (notInClosedList || distSmaller)
			{
				if (!notInClosedList)
					closedList.erase(closedListIter);
				closedList.insert(cur);
				distanceList[curCell] = curKey.second;
				if (isCloseToGoal(curCell))
				{
					success = extractPath(cur);
					break;
				}
				getSuccessors(curCell, successors);
				successorIter = successors.begin();
				for (; successorIter != successors.end(); successorIter++)
				{
					float g = curKey.second + cost(curCell, *successorIter);
					float h = gridDistance(*successorIter, ivGoal);
					SearchNode::searchnode_ptr n(new SearchNode(*successorIter, State::key(g+h, g), cur));
					openList.push(n);
				}
			}
		}

		// garbage collection
		while (!openList.empty())
		{
			openList.pop();
		}

		return success;

	}


	bool
	AstarHeuristic::extractPath(SearchNode::searchnode_ptr extractionStart)
	{

		double wx, wy;

		SearchNode::searchnode_ptr cur = extractionStart;

		float resolution = ivMapPtr->getResolution();
		float pathLength = extractionStart->getKey().first;

		float stepDistance = 0;
		const float subgoalMinDistance = ivMaxStepSize * ivSubgoalDistance / resolution;

		ivMapPtr->mapToWorld(cur->getCell().first, cur->getCell().second, wx, wy);
		std::pair<coordinate,float> subgoal(coordinate(wx, wy), pathLength * resolution);
		ivSubgoalGoalDistances.push_front(subgoal);

		bool finished = false;
		do
		{
			SearchNode::gridcell curCell = cur->getCell();
			SearchNode::gridcell predCell = cur->getPredecessor()->getCell();

			int x = predCell.first - curCell.first;
			int y = predCell.second - curCell.second;
			stepDistance += sqrt(x*x + y*y);
			if (stepDistance > subgoalMinDistance)
			{
				pathLength -= stepDistance;
				ivMapPtr->mapToWorld(cur->getCell().first, cur->getCell().second, wx, wy);
				subgoal = std::pair<coordinate,float>(coordinate(wx, wy),
				                                      pathLength * resolution);
				ivSubgoalGoalDistances.push_front(subgoal);
				stepDistance = 0;

				if (pathLength <= subgoalMinDistance)
					finished = true;
			}

			cur = cur->getPredecessor();
		} while (cur->getPredecessor() != NULL && not finished);

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
		ivSubgoalGoalDistances.clear();

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
	AstarHeuristic::isCloseToGoal(const SearchNode::gridcell& c)
	const
	{

		return c == ivGoal;

	}


	float
	AstarHeuristic::cost(const SearchNode::gridcell& c1, const SearchNode::gridcell& c2)
	const
	{

		int xd = abs(c1.first - c2.first);
		int yd = abs(c1.second - c2.second);

		float retVal = 1;
		if (xd+yd > 1)
			retVal = M_SQRT2;

		return retVal;

	}


	float
	AstarHeuristic::gridDistance(const SearchNode::gridcell& c1, const SearchNode::gridcell& c2)
	const
	{

		  double min = abs(c1.first - c2.first);
		  double max = abs(c1.second - c2.second);

		  if (min > max)
		  {
		    double tmp = min;
		    min = max;
		    max = tmp;
		  }

		  return ((M_SQRT2-1.0)*min + max);

	}


	/*
	 * #########################################################################
	 * #### class SearchNode
	 * #########################################################################
	 */

	AstarHeuristic::SearchNode::SearchNode()
		: ivCell(gridcell(0, 0)), ivKey(State::key(0, INFINITY))
	{

		ivPredecessor.reset();

	}


	AstarHeuristic::SearchNode::SearchNode(const gridcell& cell, const State::key& key)
		: ivCell(cell), ivKey(key)
	{

		ivPredecessor.reset();

	}


	AstarHeuristic::SearchNode::SearchNode(const gridcell& cell,
	                                       const State::key& key,
	                                       const searchnode_ptr predecessor)
		: ivCell(cell), ivKey(key)
	{

		ivPredecessor.reset();
		ivPredecessor = predecessor;

	}


	AstarHeuristic::SearchNode::~SearchNode()
	{}


	bool
	AstarHeuristic::SearchNode::equal_to::operator ()(searchnode_ptr n1, searchnode_ptr n2)
	const
	{
		return (n1->getCell() == n2->getCell());
	}


	bool
	AstarHeuristic::SearchNode::greater::operator ()(searchnode_ptr n1, searchnode_ptr n2)
	const
	{
		return (n1->getKey().first > n2->getKey().first);
	}


	size_t
	AstarHeuristic::SearchNode::gridcell_hash::operator ()(searchnode_ptr n)
	const
	{
		return 19*n->getCell().first + 13*n->getCell().second;
	}


	size_t
	AstarHeuristic::SearchNode::gridcell_hash::operator ()(const SearchNode::gridcell& c)
	const
	{
		return 19*c.first + 13*c.second;
	}

} // end of namespace

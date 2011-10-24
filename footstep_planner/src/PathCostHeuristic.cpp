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

#include <footstep_planner/PathCostHeuristic.h>

#define OBSTACLE_THRESHOLD 200

namespace footstep_planner
{
    PathCostHeuristic::PathCostHeuristic(int max_step_width)
        : Heuristic(PATH_COST),
          ivpGrid(NULL),
          ivGoal(NULL),
          ivMaxStepWidth(max_step_width)
    {};


    PathCostHeuristic::~PathCostHeuristic()
    {
        if (ivpGrid)
            resetGrid();
    }


    double
    PathCostHeuristic::getHValue(const PlanningState& from, const PlanningState& to)
    const
    {
        if (from == to)
            return 0;

        unsigned int from_x;
        unsigned int from_y;
        bool valid = ivMapPtr->worldToMap(from.getContX(), from.getContY(),
                                          from_x, from_y);
        if (!valid)
        {
            ROS_ERROR("invalid planning state: (%i, %i) (%f, %f)", from.getX(),
                      from.getY(), from.getContX(), from.getContY());
            ROS_ERROR("equv. grid map state: (%i, %i)", from_x, from_y);
        }
        assert(valid);

        int cost = ivGridSearchPtr->getlowerboundoncostfromstart_inmm(from_x,
                                                                      from_y);
        return cost / 1000.0;
    }


    bool
    PathCostHeuristic::calculateDistances(const PlanningState& start,
                                          const PlanningState& goal)
    {
        assert(ivMapPtr);

        if (ivGoal)
        {
            delete ivGoal;
            ivGoal = NULL;
        }
        ivGoal = new PlanningState(goal);

        unsigned int start_x;
        unsigned int start_y;
        bool valid = ivMapPtr->worldToMap(start.getContX(), start.getContY(),
                                          start_x, start_y);
        assert(valid);
        unsigned int goal_x;
        unsigned int goal_y;
        valid = ivMapPtr->worldToMap(goal.getContX(), goal.getContY(), goal_x,
                                     goal_y);
        assert(valid);

        ivGridSearchPtr->search(ivpGrid, OBSTACLE_THRESHOLD, goal_x, goal_y,
                                start_x, start_y,
                                SBPL_2DGRIDSEARCH_TERM_CONDITION_OPTPATHFOUND);

        return true;
    }


    void
    PathCostHeuristic::setMap(const boost::shared_ptr<GridMap2D> map)
    {
        ivMapPtr.reset();
        ivMapPtr = map;

        CvSize size = map->size();

        if (ivGridSearchPtr)
            ivGridSearchPtr->destroy();
        ivGridSearchPtr.reset(new SBPL2DGridSearch(size.width, size.height, 1));

        if (ivpGrid)
            resetGrid();
        ivpGrid = new unsigned char* [size.width];
        for (int x = 0; x < size.width; ++x)
        {
            ivpGrid[x] = new unsigned char [size.height];
        }
        for (int y = 0; y < size.height; ++y)
        {
            for (int x = 0; x < size.width; ++x)
            {
                if (ivMapPtr->isOccupiedAtCell(x, y))
                    ivpGrid[x][y] = 255;
                else
                    ivpGrid[x][y] = 0;
            }
        }
    }


    void
    PathCostHeuristic::resetGrid()
    {
        CvSize size = ivMapPtr->size();
        for (int x = 0; x < size.width; ++x)
        {
            if (ivpGrid[x])
            {
                delete[] ivpGrid[x];
                ivpGrid[x] = NULL;
            }
        }
        delete[] ivpGrid;
        ivpGrid = NULL;
    }
} // end of namespace

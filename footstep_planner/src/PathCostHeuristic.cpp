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


namespace footstep_planner
{
PathCostHeuristic::PathCostHeuristic(double cell_size,
                                     int    num_angle_bins,
                                     double step_cost,
                                     double diff_angle_cost,
                                     double max_step_width,
                                     double inflation_radius)
: Heuristic(cell_size, num_angle_bins, PATH_COST),
  ivpGrid(NULL),
  ivStepCost(step_cost),
  ivDiffAngleCost(diff_angle_cost),
  ivMaxStepWidth(max_step_width),
  ivInflationRadius(inflation_radius),
  ivGoalX(0),
  ivGoalY(0)
{};


PathCostHeuristic::~PathCostHeuristic()
{
  if (ivpGrid)
    resetGrid();
}


double
PathCostHeuristic::getHValue(const PlanningState& from,
                             const PlanningState& to)
const
{
  if (from == to)
    return 0.0;

  unsigned int from_x;
  unsigned int from_y;
  // could be removed after more testing (then use ...noBounds... again)
  ivMapPtr->worldToMapNoBounds(cell_2_state(from.getX(), ivCellSize),
                               cell_2_state(from.getY(), ivCellSize),
                               from_x, from_y);

  unsigned int to_x;
  unsigned int to_y;
  // could be removed after more testing (then use ...noBounds... again)
  ivMapPtr->worldToMapNoBounds(cell_2_state(to.getX(), ivCellSize),
                               cell_2_state(to.getY(), ivCellSize),
                               to_x, to_y);

  if (ivGoalX != to_x || ivGoalY != to_y){
    ROS_ERROR("PathCostHeuristic::getHValue to a different value than precomputed, heuristic values will be wrong. "
        "You need to call calculateDistances() before!");

  }

  double dist = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(
      from_x, from_y)) / 1000.0;

  double expected_steps = dist / ivMaxStepWidth;
  double diff_angle = 0.0;
  if (ivDiffAngleCost > 0.0)
  {
    // get the number of bins between from.theta and to.theta
    int diff_angle_disc = (
        ((to.getTheta() - from.getTheta()) % ivNumAngleBins) +
        ivNumAngleBins) % ivNumAngleBins;
    // get the rotation independent from the rotation direction
    diff_angle = std::abs(angles::normalize_angle(
        angle_cell_2_state(diff_angle_disc, ivNumAngleBins)));
  }

  return (dist + expected_steps * ivStepCost +
      diff_angle * ivDiffAngleCost);
}


bool
PathCostHeuristic::calculateDistances(const PlanningState& start,
                                      const PlanningState& goal)
{
  assert(ivMapPtr);

  unsigned int start_x;
  unsigned int start_y;
  ivMapPtr->worldToMapNoBounds(cell_2_state(start.getX(), ivCellSize),
                               cell_2_state(start.getY(), ivCellSize),
                               start_x, start_y);

  unsigned int to_x;
  unsigned int to_y;
  ivMapPtr->worldToMapNoBounds(cell_2_state(goal.getX(), ivCellSize),
                               cell_2_state(goal.getY(), ivCellSize),
                               to_x, to_y);

  if (to_x != ivGoalX || to_y != ivGoalY){
    ivGoalX = to_x;
    ivGoalY = to_y;
    ivGridSearchPtr->search(ivpGrid, cvObstacleThreshold,
                            ivGoalX, ivGoalY, start_x, start_y,
                            SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);

  }

  return true;
}


void
PathCostHeuristic::updateMap(gridmap_2d::GridMap2DPtr map)
{
  ivMapPtr.reset();
  ivMapPtr = map;

  unsigned width = map->getInfo().width;
  unsigned height = map->getInfo().height;

  if (ivGridSearchPtr)
    ivGridSearchPtr->destroy();
  ivGridSearchPtr.reset(new SBPL2DGridSearch(width, height,
                                             ivMapPtr->getResolution()));
  if (ivpGrid)
    resetGrid();
  ivpGrid = new unsigned char* [width];

  for (unsigned x = 0; x < width; ++x)
    ivpGrid[x] = new unsigned char [height];
  for (unsigned y = 0; y < height; ++y)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      float dist = ivMapPtr->distanceMapAtCell(x,y);
      if (dist < 0.0f){
        ROS_ERROR("Distance map at %d %d out of bounds", x, y);
      } else if (dist <= ivInflationRadius){
        ivpGrid[x][y] = 255;
      } else {
        ivpGrid[x][y] = 0;
      }
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

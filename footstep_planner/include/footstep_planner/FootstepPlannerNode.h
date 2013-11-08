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

#ifndef FOOTSTEP_PLANNER_FOOTSTEPPLANNERNODE_H_
#define FOOTSTEP_PLANNER_FOOTSTEPPLANNERNODE_H_


#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <footstep_planner/FootstepPlanner.h>


namespace footstep_planner
{
/**
 * @brief Wrapper class for FootstepPlanner, providing callbacks for
 * the node functionality.
 */
class FootstepPlannerNode
{
public:
  FootstepPlannerNode();
  virtual ~FootstepPlannerNode();

protected:
  FootstepPlanner ivFootstepPlanner;

  ros::Subscriber ivGoalPoseSub;
  ros::Subscriber ivGridMapSub;
  ros::Subscriber ivStartPoseSub;
  ros::Subscriber ivRobotPoseSub;

  ros::ServiceServer ivFootstepPlanService;
  ros::ServiceServer ivFootstepPlanFeetService;
};
}
#endif  // FOOTSTEP_PLANNER_FOOTSTEPPLANNERNODE_H_

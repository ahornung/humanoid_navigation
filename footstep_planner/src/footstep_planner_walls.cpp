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

#include <ros/ros.h>
#include <footstep_planner/FootstepPlanner.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gridmap_2d/GridMap2D.h>


using namespace footstep_planner;
using gridmap_2d::GridMap2D;
using gridmap_2d::GridMap2DPtr;

/**
 * @brief Wrapper class for FootstepPlanner, providing callbacks for
 * the node functionality. This node additionally sets wall regions
 * for the footstep planner, from a dedicated map callback.
 *
 */
class FootstepPlannerWallsNode {
public:
  FootstepPlannerWallsNode()
  {
    ros::NodeHandle privateNh("~");
    // params:
    privateNh.param("footstep_wall_dist", ivFootstepWallDist, 0.15);

    // provide callbacks to interact with the footstep planner:
    ivGridMapSub = ivNh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &FootstepPlannerWallsNode::mapCallback, this);
    ivGoalPoseSub = ivNh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &FootstepPlanner::goalPoseCallback, &ivFootstepPlanner);
    ivStartPoseSub = ivNh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &FootstepPlanner::startPoseCallback, &ivFootstepPlanner);

    // service:
    ivFootstepPlanService = ivNh.advertiseService("plan_footsteps", &FootstepPlanner::planService, &ivFootstepPlanner);
  }

  virtual ~FootstepPlannerWallsNode(){}

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancyMap)
  {
    ROS_INFO("Obstacle map received, now waiting for wall map.");
    ivGridMap = GridMap2DPtr(new GridMap2D(occupancyMap));
    // don't set wall => wait for wall map!
    //ivFootstepPlanner.setMap(ivGridMap);

    // now subscribe to walls, so that they arrive in order:
    ivWallMapSub = ivNh.subscribe<nav_msgs::OccupancyGrid>(
      "map_walls", 1, &FootstepPlannerWallsNode::wallMapCallback, this);
  }

  void wallMapCallback(const nav_msgs::OccupancyGridConstPtr& occupancyMap)
  {
    ROS_INFO("Wall / Obstacle map received");
    assert(ivGridMap);
    GridMap2DPtr wallMap(new GridMap2D(occupancyMap));

    GridMap2DPtr enlargedWallMap(new GridMap2D(occupancyMap));
    cv::Mat binaryMap =  (enlargedWallMap->distanceMap() > ivFootstepWallDist);
    bitwise_and(binaryMap, ivGridMap->binaryMap(), binaryMap);

    enlargedWallMap->setMap(binaryMap);

    ivFootstepPlanner.updateMap(enlargedWallMap);
  }

protected:
  ros::NodeHandle ivNh;
  footstep_planner::FootstepPlanner ivFootstepPlanner;
  GridMap2DPtr ivGridMap;
  double ivFootstepWallDist;
  ros::Subscriber ivGoalPoseSub, ivGridMapSub, ivWallMapSub, ivStartPoseSub, ivRobotPoseSub;
  ros::ServiceServer ivFootstepPlanService;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "footstep_planner");

  FootstepPlannerWallsNode planner;

  ros::spin();

  return 0;
}

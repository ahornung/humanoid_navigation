/*
 * Copyright 2013 Armin Hornung, University of Freiburg
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "humanoid_planner_2d/SBPLPlanner2D.h"

SBPLPlanner2D::SBPLPlanner2D()
  : nh_(),
  robot_radius_(0.25),
  start_received_(false), goal_received_(false),
  path_costs_(0.0)
{
  
  // private NodeHandle for parameters:
  ros::NodeHandle nh_private("~");
  nh_private.param("planner_type", planner_type_, std::string("ARAPlanner"));
  nh_private.param("search_until_first_solution", search_until_first_solution_, false);
  nh_private.param("allocated_time", allocated_time_, 7.0);
  nh_private.param("forward_search", forward_search_, false);
  nh_private.param("initial_epsilon", initial_epsilon_, 3.0);
  nh_private.param("robot_radius", robot_radius_, robot_radius_);

  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 0);

  // subscriptions in SBPLPlanner2DNode
}

SBPLPlanner2D::~SBPLPlanner2D() {

}

void SBPLPlanner2D::goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose){
  // set goal:
  goal_pose_ = goal_pose->pose;
  goal_received_ = true;
  ROS_DEBUG("Received goal: %f %f", goal_pose_.position.x, goal_pose_.position.y);

  if (goal_pose->header.frame_id != map_->getFrameID()){
    ROS_WARN("Goal pose frame id \"%s\" different from map frame id \"%s\"", goal_pose->header.frame_id.c_str(), map_->getFrameID().c_str());
  }
  
  // planning?
  if (start_received_)
    plan();
}

void SBPLPlanner2D::startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose){
  // set start:
  start_pose_ = start_pose->pose.pose;
  start_received_ = true;
  ROS_DEBUG("Received start: %f %f", start_pose_.position.x, start_pose_.position.y);

  if (start_pose->header.frame_id != map_->getFrameID()){
    ROS_WARN("Start pose frame id \"%s\" different from map frame id \"%s\"", start_pose->header.frame_id.c_str(), map_->getFrameID().c_str());
  }
  
  // planning?
  if (goal_received_)
    plan();
}

bool SBPLPlanner2D::plan(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal){
  start_pose_ = start;
  goal_pose_ = goal;

  start_received_ = true;
  goal_received_ = true;

  return plan();
}

bool SBPLPlanner2D::plan(double startX, double startY, double goalX, double goalY){
  start_pose_.position.x = startX;
  start_pose_.position.y = startY;

  goal_pose_.position.x = goalX;
  goal_pose_.position.y = goalY;

  start_received_ = true;
  goal_received_ = true;

  return plan();
}

bool SBPLPlanner2D::plan(){
  path_.poses.clear();

  if (!map_){
    ROS_ERROR("Map not set");
    return false;
  }

  unsigned start_x, start_y, goal_x, goal_y;
  if (!map_->worldToMap(start_pose_.position.x, start_pose_.position.y, start_x, start_y)){
    ROS_ERROR("Start coordinates out of map bounds");
    return false;
  }
  if (!map_->worldToMap(goal_pose_.position.x, goal_pose_.position.y, goal_x, goal_y)){
    ROS_ERROR("Goal coordinates out of map bounds");
    return false;
  }

  if (map_->isOccupiedAtCell(start_x, start_y)){
    ROS_ERROR("Start coordinate (%f %f) is occupied in map", start_pose_.position.x, start_pose_.position.y);
    return false;
  }
  if (map_->isOccupiedAtCell(goal_x, goal_y)){
    ROS_ERROR("Goal coordinate (%f %f) is occupied in map", goal_pose_.position.x, goal_pose_.position.y);
    return false;
  }

  int start_id = planner_environment_->SetStart(start_x, start_y);
  int goal_id = planner_environment_->SetGoal(goal_x, goal_y);

  if (start_id < 0 || planner_->set_start(start_id) == 0){
    ROS_ERROR("Failed to set start state");
    return false;
  }

  if (goal_id < 0 || planner_->set_goal(goal_id) == 0){
    ROS_ERROR("Failed to set goal state");
    return false;
  }

  // set planner params:
  planner_->set_initialsolution_eps(initial_epsilon_);
  planner_->set_search_mode(search_until_first_solution_);
  std::vector<int> solution_stateIDs;
  int solution_cost;


  if(planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost))
    ROS_DEBUG("Solution found. Costs: %d;  final eps: %f", solution_cost, planner_->get_final_epsilon());
  else{
    ROS_INFO("Solution not found");
    return false;
  }

  // scale costs (SBPL uses mm and does not know map res)
  path_costs_ = double(solution_cost) / ENVNAV2D_COSTMULT * map_->getResolution();

  // extract / publish path:
  path_.poses.reserve(solution_stateIDs.size());
  path_.header.frame_id = map_->getFrameID();
  path_.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped pose;
  pose.header = path_.header;
  for (size_t i = 0; i < solution_stateIDs.size(); i++) {
    int mx, my;
    planner_environment_->GetCoordFromState(solution_stateIDs[i], mx, my);
    //ROS_INFO("p: %d - [%d %d]", solution_stateIDs[i], mx, my);
    double wx,wy;
    map_->mapToWorld(mx,my,wx,wy);


    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    path_.poses.push_back(pose);
  }

  path_pub_.publish(path_);

  return true;
}

void SBPLPlanner2D::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map){
  gridmap_2d::GridMap2DPtr map(new gridmap_2d::GridMap2D(occupancy_map));
  updateMap(map);
}

bool SBPLPlanner2D::updateMap(gridmap_2d::GridMap2DPtr map){
  planner_environment_.reset(new EnvironmentNAV2D());
  planner_environment_->InitializeEnv(int(map->getInfo().width), int(map->getInfo().height), 0, OBSTACLE_COST);
  // environment is set up, reset planner:
  setPlanner();

  // store local copy:
  map_.reset(new gridmap_2d::GridMap2D(*map));
  map_->inflateMap(robot_radius_);


  for(unsigned int j = 0; j < map_->getInfo().height; ++j){
    for(unsigned int i = 0; i < map_->getInfo().width; ++i){
      if (map_->isOccupiedAtCell(i,j))
        planner_environment_->UpdateCost(i, j, OBSTACLE_COST);
      else
        planner_environment_->UpdateCost(i,j,0);

    }
  }

  ROS_DEBUG("Map set");

  return true;
}

void SBPLPlanner2D::setPlanner(){
  if (planner_type_ == "ARAPlanner"){
    planner_.reset(new ARAPlanner(planner_environment_.get(),forward_search_));
  } else if (planner_type_ == "ADPlanner"){
    planner_.reset(new ADPlanner(planner_environment_.get(),forward_search_));
  } else if (planner_type_ == "RSTARPlanner"){
    planner_.reset(new RSTARPlanner(planner_environment_.get(),forward_search_));
  }
}



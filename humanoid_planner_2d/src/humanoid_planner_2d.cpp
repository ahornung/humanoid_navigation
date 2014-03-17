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

#include <ros/ros.h>
#include <humanoid_planner_2d/SBPLPlanner2D.h>

class SBPLPlanner2DNode{
public:
  SBPLPlanner2DNode(){
    ros::NodeHandle nh;

    map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &SBPLPlanner2D::mapCallback, &planner_);
    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &SBPLPlanner2D::goalCallback, &planner_);
    start_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &SBPLPlanner2D::startCallback, &planner_);

  }

  virtual ~SBPLPlanner2DNode(){}

protected:
  SBPLPlanner2D planner_;
  ros::Subscriber map_sub_, goal_sub_, start_sub_;


};

int main(int argc, char** argv){
  ros::init(argc, argv, "humanoid_planner_2d");

  SBPLPlanner2DNode planner;

  ros::spin();

  return 0;
}

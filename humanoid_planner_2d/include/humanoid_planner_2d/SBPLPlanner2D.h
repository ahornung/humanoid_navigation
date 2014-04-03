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

#ifndef HUMANOID_PLANNER_2D_SBPL_2D_PLANNER_
#define HUMANOID_PLANNER_2D_SBPL_2D_PLANNER_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sbpl/headers.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <gridmap_2d/GridMap2D.h>


class SBPLPlanner2D {
public:
  SBPLPlanner2D();
  virtual ~SBPLPlanner2D();
  /// Set goal and plan when start was already set
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal);
  /// Set start and plan when goal was already set
  void startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start);
  /// calls updateMap()
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map);
  /// Setup the internal map representation and initialize the SBPL planning environment
  bool updateMap(gridmap_2d::GridMap2DPtr map);

  gridmap_2d::GridMap2DPtr getMap() const { return map_;};

  /**
   * @brief Plans from start to goal, assuming that the map was set with updateMap().
   * When successful, you can retrieve the path with getPath().
   *
   * @param start
   * @param goal
   * @return success of planning
   */
  bool plan(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);

  /**
   * @brief Plans from start to goal, assuming that the map was set with updateMap().
   * When successful, you can retrieve the path with getPath().
   *
   * @return success of planning
   */
  bool plan(double startX, double startY, double goalX, double goalY);

  /// @return costs of the path (=length in m), if planning was successful
  inline double getPathCosts() const{return path_costs_;};

  inline const nav_msgs::Path& getPath() const{return path_;};
  inline double getRobotRadius() const{return robot_radius_;};

protected:
  bool plan();
  void setPlanner(); ///< (re)sets the planner
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_, start_sub_, map_sub_;
  ros::Publisher path_pub_;
  boost::shared_ptr<SBPLPlanner> planner_;
  boost::shared_ptr<EnvironmentNAV2D> planner_environment_;
  gridmap_2d::GridMap2DPtr map_;

  std::string planner_type_;
  double allocated_time_;
  double initial_epsilon_;
  bool search_until_first_solution_;
  bool forward_search_;
  double robot_radius_;

  bool start_received_, goal_received_;
  geometry_msgs::Pose start_pose_, goal_pose_;
  nav_msgs::Path path_;
  double path_costs_;
  
  static const unsigned char OBSTACLE_COST = 20;

};

#endif

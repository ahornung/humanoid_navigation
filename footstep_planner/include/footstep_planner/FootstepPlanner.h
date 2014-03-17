/*
 * A footstep planner for humanoid robots.
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


#ifndef FOOTSTEP_PLANNER_FOOTSTEPPLANNER_H_
#define FOOTSTEP_PLANNER_FOOTSTEPPLANNER_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <humanoid_nav_msgs/PlanFootstepsBetweenFeet.h>
#include <footstep_planner/helper.h>
#include <footstep_planner/PathCostHeuristic.h>
#include <footstep_planner/FootstepPlannerEnvironment.h>
#include <footstep_planner/PlanningStateChangeQuery.h>
#include <footstep_planner/State.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <assert.h>
#include <time.h>


namespace footstep_planner
{
typedef std::vector<State>::const_iterator state_iter_t;

/**
 * @brief A class to control the interaction between ROS and the footstep
 * planner.
 */
class FootstepPlanner
{
public:
  FootstepPlanner();
  virtual ~FootstepPlanner();

  /**
   * @brief Start a planning task from scratch (will delete information
   * of previous planning tasks). Map and start, goal poses need to be
   * set beforehand.
   *
   * @return Success of planning.
   */
  bool plan(bool force_new_plan=true);

  /// @brief Sets start, goal poses and calls FootstepPlanner::plan().
  bool plan(const geometry_msgs::PoseStampedConstPtr start,
            const geometry_msgs::PoseStampedConstPtr goal);

  /// @brief Sets start, goal poses and calls FootstepPlanner::plan().
  bool plan(float start_x, float start_y, float start_theta,
            float goal_x, float goal_y, float goal_theta);

  /**
   * @brief Starts a planning task based on previous planning information
   * (note that this method can also be used when no previous planning was
   * performed). Map and start, goal poses need to be set beforehand.
   *
   * @return Success of planning.
   */
  bool replan();

  /// @brief Service handle to plan footsteps.
  bool planService(humanoid_nav_msgs::PlanFootsteps::Request &req,
                   humanoid_nav_msgs::PlanFootsteps::Response &resp);

  /// @brief Service handle to plan footsteps.
  bool planFeetService(humanoid_nav_msgs::PlanFootstepsBetweenFeet::Request &req,
                   humanoid_nav_msgs::PlanFootstepsBetweenFeet::Response &resp);

  /**
   * @brief Sets the goal pose as two feet (left / right)
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setGoal(const State& left_foot, const State& right_foot);

  /**
   * @brief Sets the goal pose as a robot pose centered between two feet.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setGoal(const geometry_msgs::PoseStampedConstPtr goal_pose);

  /**
   * @brief Sets the goal pose as a robot pose centered between two feet.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setGoal(float x, float y, float theta);

  /**
   * @brief Sets the start pose as a robot pose centered between two feet.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setStart(const geometry_msgs::PoseStampedConstPtr start_pose);

  /**
   * @brief Sets the start pose as a robot pose centered between two feet.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setStart(float x, float y, float theta);

  /**
   * @brief Sets the start pose as position of left and right footsteps.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setStart(const State& left_foot, const State& right_foot);

  /**
   * @brief Updates the map in the planning environment.
   *
   * @return True if a replanning is necessary, i.e. the old path is not valid
   * any more.
   */
  bool updateMap(const gridmap_2d::GridMap2DPtr map);

  void setMarkerNamespace(const std::string& ns)
  {
    ivMarkerNamespace = ns;
  }

  /// @brief Set the maximal search time.
  void setMaxSearchTime(int search_time)
  {
    ivMaxSearchTime = search_time;
  }

  /**
   * @brief Callback to set the goal pose as a robot pose centered between
   * two feet. If the start pose has been set previously the planning is
   * started.
   *
   * Subscribed to 'goal'.
   *
   * @return True if the two foot poses have been set successfully.
   */
  void goalPoseCallback(
      const geometry_msgs::PoseStampedConstPtr& goal_pose);
  /**
   * @brief Callback to set the start pose as a robot pose centered
   * between two feet. If the goal pose has been set previously the
   * planning is started.
   *
   * Subscribed to 'initialpose'.
   *
   * @return True if the two foot poses have been set successfully.
   */
  void startPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose);

  /**
   * @brief Callback to set the map.
   *
   * Subscribed to 'map'.
   */
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map);

  /**
   * @brief Clear the footstep path visualization from a previous planning
   * task.
   */
  void clearFootstepPathVis(unsigned num_footsteps=0);

  /// @return Costs of the planned footstep path.
  double getPathCosts() const { return ivPathCost; }

  /// @return Number of expanded states.
  size_t getNumExpandedStates() const
  {
    return ivPlannerPtr->get_n_expands();
  }

  /// @return Number of planned foot poses.
  size_t getNumFootPoses() const { return ivPath.size(); }

  state_iter_t getPathBegin() const { return ivPath.begin(); }
  state_iter_t getPathEnd() const { return ivPath.end(); }

  /// @return Size of the planned path.
  int getPathSize() { return ivPath.size(); }

  State getStartFootLeft() { return ivStartFootLeft; }
  State getStartFootRight() { return ivStartFootRight; }

  /// @brief Reset the previous planning information.
  void reset();

  /// @brief Reset and reinitialize the environment.
  void resetTotally();

  /// @return True if for the current start and goal pose a path exists.
  bool pathExists() { return (bool)ivPath.size(); }

  /// @brief Planning parameters.
  environment_params ivEnvironmentParams;

protected:
  void broadcastExpandedNodesVis();
  void broadcastRandomNodesVis();
  void broadcastFootstepPathVis();
  void broadcastHeuristicPathVis();
  void broadcastPathVis();

  /// helper to create service response
  void extractFootstepsSrv(std::vector<humanoid_nav_msgs::StepTarget> & footsteps) const;

  /**
   * @return True if the newly calculated path is different from the existing
   * one (if one exists).
   */
  bool pathIsNew(const std::vector<int>& new_path);

  /**
   * @brief Extracts the path (list of foot poses) from a list of state
   * IDs calculated by the SBPL.
   */
  bool extractPath(const std::vector<int>& state_ids);

  /// @brief Generates a visualization msgs for a foot pose.
  void footPoseToMarker(const State& footstep,
                        visualization_msgs::Marker* marker);

  /**
   * @brief Starts the planning task in the underlying SBPL.
   *
   * NOTE: Never call this directly. Always use either plan() or replan() to
   * invoke this method.
   */
  bool run();

  /// @brief Returns the foot pose of a leg for a given robot pose.
  State getFootPose(const State& robot, Leg side);

  /// @brief Sets the planning algorithm used by SBPL.
  void setPlanner();

  /// @brief Updates the environment in case of a changed map.
  void updateEnvironment(const gridmap_2d::GridMap2DPtr old_map);

  boost::shared_ptr<FootstepPlannerEnvironment> ivPlannerEnvironmentPtr;
  gridmap_2d::GridMap2DPtr ivMapPtr;
  boost::shared_ptr<SBPLPlanner> ivPlannerPtr;

  boost::shared_ptr<const PathCostHeuristic> ivPathCostHeuristicPtr;

  std::vector<State> ivPath;

  State ivStartFootLeft;
  State ivStartFootRight;
  State ivGoalFootLeft;
  State ivGoalFootRight;

  ros::Publisher  ivExpandedStatesVisPub;
  ros::Publisher  ivFootstepPathVisPub;
  ros::Publisher  ivRandomStatesVisPub;
  ros::Subscriber ivGridMapSub;
  ros::Publisher  ivHeuristicPathVisPub;
  ros::Publisher  ivPathVisPub;
  ros::Publisher  ivStartPoseVisPub;
  ros::ServiceServer ivFootstepPlanService;
  ros::ServiceServer ivFootstepPlanFeetService;

  double ivFootSeparation;
  double ivMaxStepWidth;
  int    ivCollisionCheckAccuracy;

  bool   ivStartPoseSetUp, ivGoalPoseSetUp;
  int    ivLastMarkerMsgSize;
  double ivPathCost;
  bool   ivSearchUntilFirstSolution;
  double ivMaxSearchTime;
  double ivInitialEpsilon;

  /**
   * @brief If limit of changed cells is reached the planner starts a new
   * task from the scratch.
   */
   int ivChangedCellsLimit;

  std::string ivPlannerType;
  std::string ivMarkerNamespace;

  std::vector<int> ivPlanningStatesIds;
};
}

#endif  // FOOTSTEP_PLANNER_FOOTSTEPPLANNER_H_

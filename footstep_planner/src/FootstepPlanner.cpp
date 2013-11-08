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

#include <footstep_planner/FootstepPlanner.h>
#include <humanoid_nav_msgs/ClipFootstep.h>


using gridmap_2d::GridMap2D;
using gridmap_2d::GridMap2DPtr;


namespace footstep_planner
{
FootstepPlanner::FootstepPlanner()
: ivStartPoseSetUp(false),
  ivGoalPoseSetUp(false),
  ivLastMarkerMsgSize(0),
  ivPathCost(0),
  ivMarkerNamespace("")
{
  // private NodeHandle for parameters and private messages (debug / info)
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public;

  // ..publishers
  ivExpandedStatesVisPub = nh_private.advertise<
      sensor_msgs::PointCloud>("expanded_states", 1);
  ivRandomStatesVisPub = nh_private.advertise<
      sensor_msgs::PointCloud>("random_states", 1);
  ivFootstepPathVisPub = nh_private.advertise<
      visualization_msgs::MarkerArray>("footsteps_array", 1);
  ivHeuristicPathVisPub = nh_private.advertise<
      nav_msgs::Path>("heuristic_path", 1);
  ivPathVisPub = nh_private.advertise<nav_msgs::Path>("path", 1);
  ivStartPoseVisPub = nh_private.advertise<
      geometry_msgs::PoseStamped>("start", 1);

  std::string heuristic_type;
  double diff_angle_cost;

  // read parameters from config file:
  // planner environment settings
  nh_private.param("heuristic_type", heuristic_type,
                   std::string("EuclideanHeuristic"));
  nh_private.param("heuristic_scale", ivEnvironmentParams.heuristic_scale, 1.0);
  nh_private.param("max_hash_size", ivEnvironmentParams.hash_table_size, 65536);
  nh_private.param("accuracy/collision_check",
                   ivEnvironmentParams.collision_check_accuracy,
                   2);
  nh_private.param("accuracy/cell_size", ivEnvironmentParams.cell_size, 0.01);
  nh_private.param("accuracy/num_angle_bins",
                   ivEnvironmentParams.num_angle_bins,
                   64);
  nh_private.param("step_cost", ivEnvironmentParams.step_cost, 0.05);
  nh_private.param("diff_angle_cost", diff_angle_cost, 0.0);

  nh_private.param("planner_type", ivPlannerType, std::string("ARAPlanner"));
  nh_private.param("search_until_first_solution", ivSearchUntilFirstSolution,
                   false);
  nh_private.param("allocated_time", ivMaxSearchTime, 7.0);
  nh_private.param("forward_search", ivEnvironmentParams.forward_search, false);
  nh_private.param("initial_epsilon", ivInitialEpsilon, 3.0);
  nh_private.param("changed_cells_limit", ivChangedCellsLimit, 20000);
  nh_private.param("num_random_nodes", ivEnvironmentParams.num_random_nodes,
                   20);
  nh_private.param("random_node_dist", ivEnvironmentParams.random_node_distance,
                   1.0);

  // footstep settings
  nh_private.param("foot/size/x", ivEnvironmentParams.footsize_x, 0.16);
  nh_private.param("foot/size/y", ivEnvironmentParams.footsize_y, 0.06);
  nh_private.param("foot/size/z", ivEnvironmentParams.footsize_z, 0.015);
  nh_private.param("foot/separation", ivFootSeparation, 0.1);
  nh_private.param("foot/origin_shift/x",
                   ivEnvironmentParams.foot_origin_shift_x,
                   0.02);
  nh_private.param("foot/origin_shift/y",
                   ivEnvironmentParams.foot_origin_shift_y,
                   0.0);
  nh_private.param("foot/max/step/x", ivEnvironmentParams.max_footstep_x, 0.08);
  nh_private.param("foot/max/step/y", ivEnvironmentParams.max_footstep_y, 0.16);
  nh_private.param("foot/max/step/theta",
                   ivEnvironmentParams.max_footstep_theta,
                   0.3);
  nh_private.param("foot/max/inverse/step/x",
                   ivEnvironmentParams.max_inverse_footstep_x,
                   -0.04);
  nh_private.param("foot/max/inverse/step/y",
                   ivEnvironmentParams.max_inverse_footstep_y,
                   0.09);
  nh_private.param("foot/max/inverse/step/theta",
                   ivEnvironmentParams.max_inverse_footstep_theta,
                   -0.3);

  // footstep discretization
  XmlRpc::XmlRpcValue footsteps_x;
  XmlRpc::XmlRpcValue footsteps_y;
  XmlRpc::XmlRpcValue footsteps_theta;
  nh_private.getParam("footsteps/x", footsteps_x);
  nh_private.getParam("footsteps/y", footsteps_y);
  nh_private.getParam("footsteps/theta", footsteps_theta);
  if (footsteps_x.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("Error reading footsteps/x from config file.");
  if (footsteps_y.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("Error reading footsteps/y from config file.");
  if (footsteps_theta.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("Error reading footsteps/theta from config file.");
  int size_x = footsteps_x.size();
  int size_y = footsteps_y.size();
  int size_t = footsteps_theta.size();
  if (size_x != size_y || size_x != size_t)
  {
    ROS_ERROR("Footstep parameterization has different sizes for x/y/theta. "
              "Exit!");
    exit(2);
  }
  // create footstep set
  ivEnvironmentParams.footstep_set.clear();
  double max_step_width = 0;
  for(int i=0; i < footsteps_x.size(); ++i)
  {
    double x = (double)footsteps_x[i];
    double y = (double)footsteps_y[i];
    double theta = (double)footsteps_theta[i];

    Footstep f(x, y, theta,
               ivEnvironmentParams.cell_size,
               ivEnvironmentParams.num_angle_bins,
               ivEnvironmentParams.hash_table_size);
    ivEnvironmentParams.footstep_set.push_back(f);

    double cur_step_width = sqrt(x*x + y*y);

    if (cur_step_width > max_step_width)
      max_step_width = cur_step_width;
  }

  // step range
  XmlRpc::XmlRpcValue step_range_x;
  XmlRpc::XmlRpcValue step_range_y;
  nh_private.getParam("step_range/x", step_range_x);
  nh_private.getParam("step_range/y", step_range_y);
  if (step_range_x.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("Error reading footsteps/x from config file.");
  if (step_range_y.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("Error reading footsteps/y from config file.");
  if (step_range_x.size() != step_range_y.size())
  {
    ROS_ERROR("Step range points have different size. Exit!");
    exit(2);
  }
  // create step range
  ivEnvironmentParams.step_range.clear();
  ivEnvironmentParams.step_range.reserve(step_range_x.size());
  double x, y;
  double max_x = 0.0;
  double max_y = 0.0;
  double cell_size = ivEnvironmentParams.cell_size;
  for (int i=0; i < step_range_x.size(); ++i)
  {
    x = (double)step_range_x[i];
    y = (double)step_range_y[i];
    if (fabs(x) > max_x)
      max_x = fabs(x);
    if (fabs(y) > max_y)
      max_y = fabs(y);
    ivEnvironmentParams.step_range.push_back(
      std::pair<int, int>(disc_val(x, cell_size), disc_val(y, cell_size)));
  }
  // insert first point again at the end!
  ivEnvironmentParams.step_range.push_back(ivEnvironmentParams.step_range[0]);
  ivEnvironmentParams.max_step_width = sqrt(max_x*max_x + max_y*max_y) * 1.5;

  // initialize the heuristic
  boost::shared_ptr<Heuristic> h;
  if (heuristic_type == "EuclideanHeuristic")
  {
    h.reset(
        new EuclideanHeuristic(ivEnvironmentParams.cell_size,
                               ivEnvironmentParams.num_angle_bins));
    ROS_INFO("FootstepPlanner heuristic: euclidean distance");
  }
  else if(heuristic_type == "EuclStepCostHeuristic")
  {
    h.reset(
        new EuclStepCostHeuristic(ivEnvironmentParams.cell_size,
                                  ivEnvironmentParams.num_angle_bins,
                                  ivEnvironmentParams.step_cost,
                                  diff_angle_cost,
                                  max_step_width));
    ROS_INFO("FootstepPlanner heuristic: euclidean distance with step costs");
  }
  else if (heuristic_type == "PathCostHeuristic")
  {
    // for heuristic inflation
    double foot_incircle =
      std::min((ivEnvironmentParams.footsize_x / 2.0 -
                std::abs(ivEnvironmentParams.foot_origin_shift_x)),
               (ivEnvironmentParams.footsize_y / 2.0 -
                std::abs(ivEnvironmentParams.foot_origin_shift_y)));
    assert(foot_incircle > 0.0);

    h.reset(
        new PathCostHeuristic(ivEnvironmentParams.cell_size,
                              ivEnvironmentParams.num_angle_bins,
                              ivEnvironmentParams.step_cost,
                              diff_angle_cost,
                              max_step_width,
                              foot_incircle));
    ROS_INFO("FootstepPlanner heuristic: 2D path euclidean distance with step "
             "costs");

    // keep a local ptr for visualization
    ivPathCostHeuristicPtr = boost::dynamic_pointer_cast<PathCostHeuristic>(h);
  }
  else
  {
    ROS_ERROR_STREAM("Heuristic " << heuristic_type << " not available, "
                     "exiting.");
    exit(1);
  }
  ivEnvironmentParams.heuristic = h;

  // initialize the planner environment
  ivPlannerEnvironmentPtr.reset(
    new FootstepPlannerEnvironment(ivEnvironmentParams));

  // set up planner
  if (ivPlannerType == "ARAPlanner" ||
      ivPlannerType == "ADPlanner"  ||
      ivPlannerType == "RSTARPlanner" )
  {
    ROS_INFO_STREAM("Planning with " << ivPlannerType);
  }
  else
  {
    ROS_ERROR_STREAM("Planner "<< ivPlannerType <<" not available / "
                     "untested.");
    exit(1);
  }
  if (ivEnvironmentParams.forward_search)
  {
    ROS_INFO_STREAM("Search direction: forward planning");
  }
  else
  {
    ROS_INFO_STREAM("Search direction: backward planning");
  }
  setPlanner();
}


FootstepPlanner::~FootstepPlanner()
{}


void
FootstepPlanner::setPlanner()
{
  if (ivPlannerType == "ARAPlanner")
  {
    ivPlannerPtr.reset(
        new ARAPlanner(ivPlannerEnvironmentPtr.get(),
                       ivEnvironmentParams.forward_search));
  }
  else if (ivPlannerType == "ADPlanner")
  {
    ivPlannerPtr.reset(
        new ADPlanner(ivPlannerEnvironmentPtr.get(),
                      ivEnvironmentParams.forward_search));
  }
  else if (ivPlannerType == "RSTARPlanner")
  {
    RSTARPlanner* p =
        new RSTARPlanner(ivPlannerEnvironmentPtr.get(),
                         ivEnvironmentParams.forward_search);
    // new options, require patched SBPL
    //          p->set_local_expand_thres(500);
    //          p->set_eps_step(1.0);
    ivPlannerPtr.reset(p);
  }
  //        else if (ivPlannerType == "ANAPlanner")
  //        	ivPlannerPtr.reset(new anaPlanner(ivPlannerEnvironmentPtr.get(),
  //        	                                  ivForwardSearch));
}


bool
FootstepPlanner::run()
{
  bool path_existed = (bool)ivPath.size();
  int ret = 0;
  MDPConfig mdp_config;
  std::vector<int> solution_state_ids;

  // commit start/goal poses to the environment
  ivPlannerEnvironmentPtr->updateStart(ivStartFootLeft, ivStartFootRight);
  ivPlannerEnvironmentPtr->updateGoal(ivGoalFootLeft, ivGoalFootRight);
  ivPlannerEnvironmentPtr->updateHeuristicValues();
  ivPlannerEnvironmentPtr->InitializeEnv(NULL);
  ivPlannerEnvironmentPtr->InitializeMDPCfg(&mdp_config);

  // inform AD planner about changed (start) states for replanning
  if (path_existed &&
      !ivEnvironmentParams.forward_search &&
      ivPlannerType == "ADPlanner")
  {
    std::vector<int> changed_edges;
    changed_edges.push_back(mdp_config.startstateid);
    // update the AD planner
    boost::shared_ptr<ADPlanner> ad_planner =
      boost::dynamic_pointer_cast<ADPlanner>(ivPlannerPtr);
    ad_planner->update_preds_of_changededges(&changed_edges);
  }

  // set up SBPL
  if (ivPlannerPtr->set_start(mdp_config.startstateid) == 0)
  {
    ROS_ERROR("Failed to set start state.");
    return false;
  }
  if (ivPlannerPtr->set_goal(mdp_config.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state\n");
    return false;
  }

  ivPlannerPtr->set_initialsolution_eps(ivInitialEpsilon);
  ivPlannerPtr->set_search_mode(ivSearchUntilFirstSolution);

  ROS_INFO("Start planning (max time: %f, initial eps: %f (%f))\n",
           ivMaxSearchTime, ivInitialEpsilon,
           ivPlannerPtr->get_initial_eps());
  int path_cost;
  ros::WallTime startTime = ros::WallTime::now();
  try
  {
    ret = ivPlannerPtr->replan(ivMaxSearchTime, &solution_state_ids,
                               &path_cost);
  }
  catch (const SBPL_Exception& e)
  {
    // ROS_ERROR("SBPL planning failed (%s)", e.what());
    return false;
  }
  ivPathCost = double(path_cost) / FootstepPlannerEnvironment::cvMmScale;

  bool path_is_new = pathIsNew(solution_state_ids);
  if (ret && solution_state_ids.size() > 0)
  {
    if (!path_is_new)
      ROS_WARN("Solution found by SBPL is the same as the old solution. This could indicate that replanning failed.");

    ROS_INFO("Solution of size %zu found after %f s",
             solution_state_ids.size(),
             (ros::WallTime::now()-startTime).toSec());

    if (extractPath(solution_state_ids))
    {
      ROS_INFO("Expanded states: %i total / %i new",
               ivPlannerEnvironmentPtr->getNumExpandedStates(),
               ivPlannerPtr->get_n_expands());
      ROS_INFO("Final eps: %f", ivPlannerPtr->get_final_epsilon());
      ROS_INFO("Path cost: %f (%i)\n", ivPathCost, path_cost);

      ivPlanningStatesIds = solution_state_ids;

      broadcastExpandedNodesVis();
      broadcastRandomNodesVis();
      broadcastFootstepPathVis();
      broadcastPathVis();

      return true;
    }
    else
    {
      ROS_ERROR("extracting path failed\n\n");
      return false;
    }
  }
  else
  {
    broadcastExpandedNodesVis();
    broadcastRandomNodesVis();

    ROS_ERROR("No solution found");
    return false;
  }
}


bool
FootstepPlanner::extractPath(const std::vector<int>& state_ids)
{
  ivPath.clear();

  State s;
  State start_left;
  std::vector<int>::const_iterator state_ids_iter = state_ids.begin();

  // first state is always the robot's left foot
  if (!ivPlannerEnvironmentPtr->getState(*state_ids_iter, &start_left))
  {
    ivPath.clear();
    return false;
  }
  ++state_ids_iter;
  if (!ivPlannerEnvironmentPtr->getState(*state_ids_iter, &s))
  {
    ivPath.clear();
    return false;
  }
  ++state_ids_iter;

  // check if the robot's left foot can be ommited as first state in the path,
  // i.e. the robot's right foot is appended first to the path
  if (s.getLeg() == LEFT)
    ivPath.push_back(ivStartFootRight);
  else
    ivPath.push_back(start_left);
  ivPath.push_back(s);

  for(; state_ids_iter < state_ids.end(); ++state_ids_iter)
  {
    if (!ivPlannerEnvironmentPtr->getState(*state_ids_iter, &s))
    {
      ivPath.clear();
      return false;
    }
    ivPath.push_back(s);
  }

  // add last neutral step
  if (ivPath.back().getLeg() == RIGHT)
    ivPath.push_back(ivGoalFootLeft);
  else // last_leg == LEFT
    ivPath.push_back(ivGoalFootRight);

  return true;
}


void
FootstepPlanner::reset()
{
  ROS_INFO("Resetting planner");
  // reset the previously calculated paths
  ivPath.clear();
  ivPlanningStatesIds.clear();
  // reset the planner
  // INFO: force_planning_from_scratch was not working properly the last time
  // checked; therefore instead of using this function the planner is manually
  // reset
  //ivPlannerPtr->force_planning_from_scratch();
  ivPlannerEnvironmentPtr->reset();
  setPlanner();
}


void
FootstepPlanner::resetTotally()
{
  ROS_INFO("Resetting planner and environment");
  // reset the previously calculated paths
  ivPath.clear();
  ivPlanningStatesIds.clear();
  // reinitialize the planner environment
  ivPlannerEnvironmentPtr.reset(
      new FootstepPlannerEnvironment(ivEnvironmentParams));
  setPlanner();
}


bool
FootstepPlanner::plan(bool force_new_plan)
{
  if (!ivMapPtr)
  {
    ROS_ERROR("FootstepPlanner has no map for planning yet.");
    return false;
  }
  if (!ivGoalPoseSetUp || !ivStartPoseSetUp)
  {
    ROS_ERROR("FootstepPlanner has not set the start and/or goal pose "
              "yet.");
    return false;
  }

  if (force_new_plan
      || ivPlannerType == "RSTARPlanner" || ivPlannerType == "ARAPlanner" )
  {
    reset();
  }
  // start the planning and return success
  return run();
}


bool
FootstepPlanner::replan()
{
  return plan(false);
}


bool
FootstepPlanner::plan(const geometry_msgs::PoseStampedConstPtr start,
                      const geometry_msgs::PoseStampedConstPtr goal)
{
  return plan(start->pose.position.x, start->pose.position.y,
              tf::getYaw(start->pose.orientation),
              goal->pose.position.x, goal->pose.position.y,
              tf::getYaw(goal->pose.orientation));
}


bool
FootstepPlanner::plan(float start_x, float start_y, float start_theta,
                      float goal_x, float goal_y, float goal_theta)
{
  if (!(setStart(start_x, start_y, start_theta) &&
      setGoal(goal_x, goal_y, goal_theta)))
  {
    return false;
  }

  return plan(false);
}


bool
FootstepPlanner::planService(humanoid_nav_msgs::PlanFootsteps::Request &req,
                             humanoid_nav_msgs::PlanFootsteps::Response &resp)
{
  bool result = plan(req.start.x, req.start.y, req.start.theta,
                     req.goal.x, req.goal.y, req.goal.theta);

  resp.costs = getPathCosts();
  resp.footsteps.reserve(getPathSize());
  resp.final_eps = ivPlannerPtr->get_final_epsilon();
  resp.expanded_states = ivPlannerEnvironmentPtr->getNumExpandedStates();
  extractFootstepsSrv(resp.footsteps);

  resp.result = result;

  // return true since service call was successful (independent from the
  // success of the planning call)
  return true;
}


bool
FootstepPlanner::planFeetService(humanoid_nav_msgs::PlanFootstepsBetweenFeet::Request &req,
                             humanoid_nav_msgs::PlanFootstepsBetweenFeet::Response &resp)
{
  // TODO check direction and change of states, force planning from scratch if does not fit
  setStart(State(req.start_left.pose.x, req.start_left.pose.y, req.start_left.pose.theta, LEFT),
           State(req.start_right.pose.x, req.start_right.pose.y, req.start_right.pose.theta, RIGHT));
  setGoal(State(req.goal_left.pose.x, req.goal_left.pose.y, req.goal_left.pose.theta, LEFT),
           State(req.goal_right.pose.x, req.goal_right.pose.y, req.goal_right.pose.theta, RIGHT));

  bool result = plan(false);

  resp.costs = getPathCosts();
  resp.footsteps.reserve(getPathSize());
  resp.final_eps = ivPlannerPtr->get_final_epsilon();
  resp.expanded_states = ivPlannerEnvironmentPtr->getNumExpandedStates();
  extractFootstepsSrv(resp.footsteps);

  resp.result = result;

  // return true since service call was successful (independent from the
  // success of the planning call)
  return true;
}

void
FootstepPlanner::extractFootstepsSrv(std::vector<humanoid_nav_msgs::StepTarget> & footsteps) const{
  humanoid_nav_msgs::StepTarget foot;
  state_iter_t path_iter;
  for (path_iter = getPathBegin(); path_iter != getPathEnd(); ++path_iter)
  {
    foot.pose.x = path_iter->getX();
    foot.pose.y = path_iter->getY();
    foot.pose.theta = path_iter->getTheta();
    if (path_iter->getLeg() == LEFT)
      foot.leg = humanoid_nav_msgs::StepTarget::left;
    else if (path_iter->getLeg() == RIGHT)
      foot.leg = humanoid_nav_msgs::StepTarget::right;
    else
    {
      ROS_ERROR("Footstep pose at (%f, %f, %f) is set to NOLEG!",
                path_iter->getX(), path_iter->getY(),
                path_iter->getTheta());
      continue;
    }

    footsteps.push_back(foot);
  }

}


void
FootstepPlanner::goalPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  // update the goal states in the environment
  if (setGoal(goal_pose))
  {
    if (ivStartPoseSetUp)
    {
      // force planning from scratch when backwards direction
      plan(!ivEnvironmentParams.forward_search);
    }
  }
}


void
FootstepPlanner::startPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose)
{
  if (setStart(start_pose->pose.pose.position.x,
               start_pose->pose.pose.position.y,
               tf::getYaw(start_pose->pose.pose.orientation)))
  {
    if (ivGoalPoseSetUp)
    {
      // force planning from scratch when forward direction
      plan(ivEnvironmentParams.forward_search);
    }
  }
}


void
FootstepPlanner::mapCallback(
    const nav_msgs::OccupancyGridConstPtr& occupancy_map)
{
  GridMap2DPtr map(new GridMap2D(occupancy_map));

  // new map: update the map information
  if (updateMap(map))
  {
    // NOTE: update map currently simply resets the planner, i.e. replanning
    // here is in fact a planning from the scratch
    plan(false);
  }
}


bool
FootstepPlanner::setGoal(const geometry_msgs::PoseStampedConstPtr goal_pose)
{
  return setGoal(goal_pose->pose.position.x,
                 goal_pose->pose.position.y,
                 tf::getYaw(goal_pose->pose.orientation));
}


bool
FootstepPlanner::setGoal(float x, float y, float theta)
{
  if (!ivMapPtr)
  {
    ROS_ERROR("Distance map hasn't been initialized yet.");
    return false;
  }

  State goal(x, y, theta, NOLEG);
  State foot_left = getFootPose(goal, LEFT);
  State foot_right = getFootPose(goal, RIGHT);

  if (ivPlannerEnvironmentPtr->occupied(foot_left) ||
      ivPlannerEnvironmentPtr->occupied(foot_right))
  {
    ROS_ERROR("Goal pose at (%f %f %f) not accessible.", x, y, theta);
    ivGoalPoseSetUp = false;
    return false;
  }
  ivGoalFootLeft = foot_left;
  ivGoalFootRight = foot_right;

  ivGoalPoseSetUp = true;
  ROS_INFO("Goal pose set to (%f %f %f)", x, y, theta);

  return true;
}

bool
FootstepPlanner::setGoal(const State& left_foot, const State& right_foot)
{
  if (ivPlannerEnvironmentPtr->occupied(left_foot) ||
      ivPlannerEnvironmentPtr->occupied(right_foot))
  {
    ivGoalPoseSetUp = false;
    return false;
  }
  ivGoalFootLeft = left_foot;
  ivGoalFootRight = right_foot;

  ivGoalPoseSetUp = true;

  return true;
}


bool
FootstepPlanner::setStart(const geometry_msgs::PoseStampedConstPtr start_pose)
{
  return setStart(start_pose->pose.position.x,
                  start_pose->pose.position.y,
                  tf::getYaw(start_pose->pose.orientation));
}


bool
FootstepPlanner::setStart(const State& left_foot, const State& right_foot)
{
  if (ivPlannerEnvironmentPtr->occupied(left_foot) ||
      ivPlannerEnvironmentPtr->occupied(right_foot))
  {
    ivStartPoseSetUp = false;
    return false;
  }
  ivStartFootLeft = left_foot;
  ivStartFootRight = right_foot;

  ivStartPoseSetUp = true;

  return true;
}


bool
FootstepPlanner::setStart(float x, float y, float theta)
{
  if (!ivMapPtr)
  {
    ROS_ERROR("Distance map hasn't been initialized yet.");
    return false;
  }

  State start(x, y, theta, NOLEG);
  State foot_left = getFootPose(start, LEFT);
  State foot_right = getFootPose(start, RIGHT);

  bool success = setStart(foot_left, foot_right);
  if (success)
    ROS_INFO("Start pose set to (%f %f %f)", x, y, theta);
  else
    ROS_ERROR("Start pose (%f %f %f) not accessible.", x, y, theta);

  // publish visualization:
  geometry_msgs::PoseStamped start_pose;
  start_pose.pose.position.x = x;
  start_pose.pose.position.y = y;
  start_pose.pose.position.z = 0.025;
  start_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  start_pose.header.frame_id = ivMapPtr->getFrameID();
  start_pose.header.stamp = ros::Time::now();
  ivStartPoseVisPub.publish(start_pose);

  return success;
}


bool
FootstepPlanner::updateMap(const GridMap2DPtr map)
{
  // store old map pointer locally
  GridMap2DPtr old_map = ivMapPtr;
  // store new map
  ivMapPtr.reset();
  ivMapPtr = map;

  // check if a previous map and a path existed
  if (old_map && (bool)ivPath.size())
  {
    updateEnvironment(old_map);
    return true;
  }

  // ..otherwise the environment's map can simply be updated
  ivPlannerEnvironmentPtr->updateMap(map);
  return false;
}


void
FootstepPlanner::updateEnvironment(const GridMap2DPtr old_map)
{
  ROS_INFO("Reseting the planning environment.");
  // reset environment
  resetTotally();
  // set the new map
  ivPlannerEnvironmentPtr->updateMap(ivMapPtr);


  // The following is not used any more

  // Replanning based on old planning info currently disabled
  //        // TODO: handle size changes of the map; currently the planning
  //        // information is reseted
  //
  //        if (ivPlannerType == "ADPlanner" &&
  //            ivMapPtr->getResolution() == old_map->getResolution() &&
  //            ivMapPtr->size().height == old_map->size().height &&
  //            ivMapPtr->size().width == old_map->size().width)
  //        {
  //            ROS_INFO("Received an updated map => change detection");
  //
  //            std::vector<State2> changed_states;
  //            cv::Mat changed_cells;
  //
  //            // get new occupied cells only (0: occupied in binary map)
  //            // changedCells(x,y) = old(x,y) AND NOT(new(x,y))
  ////          cv::bitwise_not(gridMap->binaryMap(), changedCells);
  ////          cv::bitwise_and(ivMapPtr->binaryMap(), changedCells, changedCells);
  //
  //            // to get all changed cells (new free and occupied) use XOR:
  //            cv::bitwise_xor(old_map->binaryMap(), ivMapPtr->binaryMap(),
  //                            changed_cells);
  //
  //            //inflate by outer foot radius:
  //            cv::bitwise_not(changed_cells, changed_cells); // invert for distanceTransform
  //            cv::Mat changedDistMap = cv::Mat(changed_cells.size(), CV_32FC1);
  //            cv::distanceTransform(changed_cells, changedDistMap,
  //                                  CV_DIST_L2, CV_DIST_MASK_PRECISE);
  //            double max_foot_radius = sqrt(
  //                    pow(std::abs(ivOriginFootShiftX) + ivFootsizeX / 2.0, 2.0) +
  //                    pow(std::abs(ivOriginFootShiftY) + ivFootsizeY / 2.0, 2.0))
  //                    / ivMapPtr->getResolution();
  //            changed_cells = (changedDistMap <= max_foot_radius); // threshold, also invert back
  //
  //            // loop over changed cells (now marked with 255 in the mask):
  //            unsigned int num_changed_cells = 0;
  //            double wx, wy;
  //            State2 s;
  //            for (int y = 0; y < changed_cells.rows; ++y)
  //            {
  //                for (int x = 0; x < changed_cells.cols; ++x)
  //                {
  //                    if (changed_cells.at<uchar>(x,y) == 255)
  //                    {
  //                        ++num_changed_cells;
  //                        ivMapPtr->mapToWorld(x, y, wx, wy);
  //                        s.setX(wx);
  //                        s.setY(wy);
  //                        // on each grid cell ivNumAngleBins-many planning states
  //                        // can be placed (if the resolution of the grid cells is
  //                        // the same as of the planning state grid)
  //                        for (int theta = 0; theta < ivNumAngleBins; ++theta)
  //                        {
  //                            s.setTheta(angle_cell_2_state(theta, ivNumAngleBins));
  //                            changed_states.push_back(s);
  //                        }
  //                    }
  //                }
  //            }
  //
  //            if (num_changed_cells == 0)
  //            {
  //                ROS_INFO("old map equals new map; no replanning necessary");
  //                return;
  //            }
  //
  //            ROS_INFO("%d changed map cells found", num_changed_cells);
  //            if (num_changed_cells <= ivChangedCellsLimit)
  //            {
  //                // update planer
  //                ROS_INFO("Use old information in new planning taks");
  //
  //                std::vector<int> neighbour_ids;
  //                if (ivForwardSearch)
  //                    ivPlannerEnvironmentPtr->getSuccsOfGridCells(
  //                            changed_states, &neighbour_ids);
  //                else
  //                    ivPlannerEnvironmentPtr->getPredsOfGridCells(
  //                            changed_states, &neighbour_ids);
  //
  //                boost::shared_ptr<ADPlanner> h =
  //                        boost::dynamic_pointer_cast<ADPlanner>(ivPlannerPtr);
  //                h->costs_changed(PlanningStateChangeQuery(neighbour_ids));
  //            }
  //            else
  //            {
  //                ROS_INFO("Reset old information in new planning task");
  //                // reset planner
  //                ivPlannerEnvironmentPtr->reset();
  //                setPlanner();
  //                //ivPlannerPtr->force_planning_from_scratch();
  //            }
  //        }
  //        else
  //        {
  //            ROS_INFO("Reset old information in new planning task");
  //            // reset planner
  //            ivPlannerEnvironmentPtr->reset();
  //            setPlanner();
  //            //ivPlannerPtr->force_planning_from_scratch();
  //        }
}


State
FootstepPlanner::getFootPose(const State& robot, Leg leg)
{
  double shift_x = -sin(robot.getTheta()) * ivFootSeparation / 2.0;
  double shift_y =  cos(robot.getTheta()) * ivFootSeparation / 2.0;

  double sign = -1.0;
  if (leg == LEFT)
    sign = 1.0;

  return State(robot.getX() + sign * shift_x,
               robot.getY() + sign * shift_y,
               robot.getTheta(),
               leg);
}


bool
FootstepPlanner::pathIsNew(const std::vector<int>& new_path)
{
  if (new_path.size() != ivPlanningStatesIds.size())
    return true;

  bool unequal = true;
  for (unsigned i = 0; i < new_path.size(); ++i)
    unequal = new_path[i] != ivPlanningStatesIds[i] && unequal;

  return unequal;
}


void
FootstepPlanner::clearFootstepPathVis(unsigned num_footsteps)
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_msg;

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = ivMapPtr->getFrameID();


  if (num_footsteps < 1)
    num_footsteps = ivLastMarkerMsgSize;

  for (unsigned i = 0; i < num_footsteps; ++i)
  {
    marker.ns = ivMarkerNamespace;
    marker.id = i;
    marker.action = visualization_msgs::Marker::DELETE;

    marker_msg.markers.push_back(marker);
  }

  ivFootstepPathVisPub.publish(marker_msg);
}


void
FootstepPlanner::broadcastExpandedNodesVis()
{
  if (ivExpandedStatesVisPub.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud cloud_msg;
    geometry_msgs::Point32 point;
    std::vector<geometry_msgs::Point32> points;

    State s;
    FootstepPlannerEnvironment::exp_states_2d_iter_t state_id_it;
    for(state_id_it = ivPlannerEnvironmentPtr->getExpandedStatesStart();
        state_id_it != ivPlannerEnvironmentPtr->getExpandedStatesEnd();
        ++state_id_it)
    {
      point.x = cell_2_state(state_id_it->first,
                             ivEnvironmentParams.cell_size);
      point.y = cell_2_state(state_id_it->second,
                             ivEnvironmentParams.cell_size);
      point.z = 0.01;
      points.push_back(point);
    }
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = ivMapPtr->getFrameID();

    cloud_msg.points = points;

    ivExpandedStatesVisPub.publish(cloud_msg);
  }
}


void
FootstepPlanner::broadcastFootstepPathVis()
{
  if (getPathSize() == 0)
  {
    ROS_INFO("no path has been extracted yet");
    return;
  }

  clearFootstepPathVis(0);

  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray broadcast_msg;
  std::vector<visualization_msgs::Marker> markers;

  int markers_counter = 0;

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = ivMapPtr->getFrameID();

  // add the missing start foot to the publish vector for visualization:
  if (ivPath.front().getLeg() == LEFT)
    footPoseToMarker(ivStartFootRight, &marker);
  else
    footPoseToMarker(ivStartFootLeft, &marker);
  marker.id = markers_counter++;
  markers.push_back(marker);

  // add the footsteps of the path to the publish vector
  for(state_iter_t path_iter = getPathBegin(); path_iter != getPathEnd();
      ++path_iter)
  {
    footPoseToMarker(*path_iter, &marker);
    marker.id = markers_counter++;
    markers.push_back(marker);
  }

  broadcast_msg.markers = markers;
  ivLastMarkerMsgSize = markers.size();

  ivFootstepPathVisPub.publish(broadcast_msg);
}


void
FootstepPlanner::broadcastRandomNodesVis()
{
  if (ivRandomStatesVisPub.getNumSubscribers() > 0){
    sensor_msgs::PointCloud cloud_msg;
    geometry_msgs::Point32 point;
    std::vector<geometry_msgs::Point32> points;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray broadcast_msg;
    std::vector<visualization_msgs::Marker> markers;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = ivMapPtr->getFrameID();

    State s;
    FootstepPlannerEnvironment::exp_states_iter_t state_id_iter;
    for(state_id_iter = ivPlannerEnvironmentPtr->getRandomStatesStart();
        state_id_iter != ivPlannerEnvironmentPtr->getRandomStatesEnd();
        ++state_id_iter)
    {
      if (!ivPlannerEnvironmentPtr->getState(*state_id_iter, &s))
      {
        ROS_WARN("Could not get random state %d", *state_id_iter);
      }
      else
      {
        point.x = s.getX();
        point.y = s.getY();
        point.z = 0.01;
        points.push_back(point);
      }
    }
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = ivMapPtr->getFrameID();

    cloud_msg.points = points;

    ivRandomStatesVisPub.publish(cloud_msg);
  }
}


void
FootstepPlanner::broadcastPathVis()
{
  if (getPathSize() == 0)
  {
    ROS_INFO("no path has been extracted yet");
    return;
  }

  nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped state;

  state.header.stamp = ros::Time::now();
  state.header.frame_id = ivMapPtr->getFrameID();

  state_iter_t path_iter;
  for(path_iter = getPathBegin(); path_iter != getPathEnd(); ++path_iter)
  {
    state.pose.position.x = path_iter->getX();
    state.pose.position.y = path_iter->getY();
    path_msg.poses.push_back(state);
  }

  path_msg.header = state.header;
  ivPathVisPub.publish(path_msg);
}


void
FootstepPlanner::footPoseToMarker(const State& foot_pose,
                                  visualization_msgs::Marker* marker)
{
  marker->header.stamp = ros::Time::now();
  marker->header.frame_id = ivMapPtr->getFrameID();
  marker->ns = ivMarkerNamespace;
  marker->type = visualization_msgs::Marker::CUBE;
  marker->action = visualization_msgs::Marker::ADD;

  float cos_theta = cos(foot_pose.getTheta());
  float sin_theta = sin(foot_pose.getTheta());
  float x_shift = cos_theta * ivEnvironmentParams.foot_origin_shift_x -
                  sin_theta * ivEnvironmentParams.foot_origin_shift_y;
  float y_shift;
  if (foot_pose.getLeg() == LEFT)
    y_shift = sin_theta * ivEnvironmentParams.foot_origin_shift_x +
              cos_theta * ivEnvironmentParams.foot_origin_shift_y;
  else // leg == RLEG
    y_shift = sin_theta * ivEnvironmentParams.foot_origin_shift_x -
              cos_theta * ivEnvironmentParams.foot_origin_shift_y;
  marker->pose.position.x = foot_pose.getX() + x_shift;
  marker->pose.position.y = foot_pose.getY() + y_shift;
  marker->pose.position.z = ivEnvironmentParams.footsize_z / 2.0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(foot_pose.getTheta()),
                        marker->pose.orientation);

  marker->scale.x = ivEnvironmentParams.footsize_x; // - 0.01;
  marker->scale.y = ivEnvironmentParams.footsize_y; // - 0.01;
  marker->scale.z = ivEnvironmentParams.footsize_z;

  // TODO: make color configurable?
  if (foot_pose.getLeg() == RIGHT)
  {
    marker->color.r = 0.0f;
    marker->color.g = 1.0f;
  }
  else // leg == LEFT
      {
    marker->color.r = 1.0f;
    marker->color.g = 0.0f;
      }
  marker->color.b = 0.0;
  marker->color.a = 0.6;

  marker->lifetime = ros::Duration();
}
}

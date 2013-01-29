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

#include <footstep_planner/FootstepPlanner.h>
#include <humanoid_nav_msgs/ClipFootstep.h>

#include <time.h>


using gridmap_2d::GridMap2D;
using gridmap_2d::GridMap2DPtr;


namespace footstep_planner
{
FootstepPlanner::FootstepPlanner()
: ivStartPoseSetUp(false),
  ivGoalPoseSetUp(false),
  ivPathExists(false),
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

  int max_hash_size;
  std::string heuristic_type;
  double step_cost;
  double diff_angle_cost;
  int num_random_nodes;
  double random_node_dist;
  double heuristic_scale;

  // read parameters from config file:
  // - planner environment settings
  nh_private.param("heuristic_type", heuristic_type,
                   std::string("EuclideanHeuristic"));
  nh_private.param("heuristic_scale", heuristic_scale, 1.0);
  nh_private.param("max_hash_size", max_hash_size, 65536);
  nh_private.param("accuracy/collision_check", ivCollisionCheckAccuracy, 2);
  nh_private.param("accuracy/cell_size", ivCellSize, 0.01);
  nh_private.param("accuracy/num_angle_bins", ivNumAngleBins, 64);
  nh_private.param("step_cost", step_cost, 0.05);
  nh_private.param("diff_angle_cost", diff_angle_cost, 0.0);

  nh_private.param("planner_type", ivPlannerType, std::string("ARAPlanner"));
  nh_private.param("search_until_first_solution", ivSearchUntilFirstSolution,
                   false);
  nh_private.param("allocated_time", ivMaxSearchTime, 7.0);
  nh_private.param("forward_search", ivForwardSearch, false);
  nh_private.param("initial_epsilon", ivInitialEpsilon, 3.0);
  nh_private.param("changed_cells_limit", ivChangedCellsLimit, 20000);
  nh_private.param("num_random_nodes", num_random_nodes, 20);
  nh_private.param("random_node_dist", random_node_dist, 1.0);

  // - footstep settings
  nh_private.param("foot/size/x", ivFootsizeX, 0.16);
  nh_private.param("foot/size/y", ivFootsizeY, 0.06);
  nh_private.param("foot/size/z", ivFootsizeZ, 0.015);
  nh_private.param("foot/separation", ivFootSeparation, 0.1);
  nh_private.param("foot/origin_shift/x", ivOriginFootShiftX, 0.02);
  nh_private.param("foot/origin_shift/y", ivOriginFootShiftY, 0.0);

  // for heuristic inflation
  double foot_incircle = std::min(
      (ivFootsizeX / 2.0 -std::abs(ivOriginFootShiftX)),
      (ivFootsizeY / 2.0 -std::abs(ivOriginFootShiftY)));
  assert (foot_incircle > 0.0);

  // - footstep discretization
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
  // check if received footstep discretization is valid
  int size, size_y, size_t;
  try
  {
    size = footsteps_x.size();
    size_y = footsteps_y.size();
    size_t = footsteps_theta.size();

    if (size != size_y || size != size_t)
    {
      ROS_ERROR("Footstep parameterization has different sizes for "
          "x/y/theta, exiting.");
      exit(2);
    }
  }
  catch (const XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("No footstep parameterization available, exiting.");
    exit(2);
  }

  // create footstep set
  ivFootstepSet.clear();
  double max_step_width = 0;
  for(int i=0; i < size; i++)
  {
    double x = (double) footsteps_x[i];
    double y = (double) footsteps_y[i];
    double theta = (double) footsteps_theta[i];

    Footstep f(x, y, theta, ivCellSize, ivNumAngleBins, max_hash_size);
    ivFootstepSet.push_back(f);

    double cur_step_width = sqrt(x*x + y*y);

    if (cur_step_width > max_step_width)
      max_step_width = cur_step_width;
  }

  // initialize the heuristic
  boost::shared_ptr<Heuristic> h;
  if (heuristic_type == "EuclideanHeuristic")
  {
    h.reset(new EuclideanHeuristic(ivCellSize, ivNumAngleBins));
    ROS_INFO("FootstepPlanner heuristic: euclidean distance");
  }
  else if(heuristic_type == "EuclStepCostHeuristic")
  {
    h.reset(new EuclStepCostHeuristic(ivCellSize, ivNumAngleBins,
                                      step_cost, diff_angle_cost,
                                      max_step_width));
    ROS_INFO("FootstepPlanner heuristic: euclidean distance with step "
        "costs");
  }
  else if (heuristic_type == "PathCostHeuristic")
  {
    h.reset(new PathCostHeuristic(ivCellSize, ivNumAngleBins, step_cost,
                                  diff_angle_cost, max_step_width,
                                  foot_incircle));
    ROS_INFO("FootstepPlanner heuristic: 2D path euclidean distance with step "
             "costs");
    // keep a local ptr for visualization
    ivPathCostHeuristicPtr = boost::dynamic_pointer_cast<
        PathCostHeuristic>(h);
  }
  else
  {
    ROS_ERROR_STREAM("Heuristic " << heuristic_type << " not available, "
                     "exiting.");
    exit(1);
  }

  // create the polygon that defines the executable range of a single step
  // this range is valid for all thetas in [-0.3, 0.3]
  std::vector<std::pair<int, int> > step_range;
  int x = disc_val(0.0, ivCellSize);
  int y = disc_val(0.15, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(0.01, ivCellSize);
  y = disc_val(0.15, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(0.02, ivCellSize);
  y = disc_val(0.15, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(0.03, ivCellSize);
  y = disc_val(0.14, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(0.05, ivCellSize);
  y = disc_val(0.13, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(0.06, ivCellSize);
  y = disc_val(0.13, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(0.07, ivCellSize);
  y = disc_val(0.12, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(0.07, ivCellSize);
  y = disc_val(0.09, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(-0.03, ivCellSize);
  y = disc_val(0.09, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(-0.03, ivCellSize);
  y = disc_val(0.13, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(-0.02, ivCellSize);
  y = disc_val(0.14, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(-0.02, ivCellSize);
  y = disc_val(0.14, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  x = disc_val(-0.01, ivCellSize);
  y = disc_val(0.15, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));
  // first point has to be included at the end of the list again
  x = disc_val(0.0, ivCellSize);
  y = disc_val(0.15, ivCellSize);
  step_range.push_back(std::pair<int, int>(x, y));

  double max_x = max(0.07, fabs(-0.3));
  double max_y = max(0.15, 0.09);

  // initialize the planner environment
  ivPlannerEnvironmentPtr.reset(
      new FootstepPlannerEnvironment(ivFootstepSet,
                                     h,
                                     ivFootsizeX,
                                     ivFootsizeY,
                                     ivOriginFootShiftX,
                                     ivOriginFootShiftY,
                                     0.07,
                                     0.15,
                                     0.3,
                                     -0.3,
                                     0.09,
                                     0.0,
                                     step_range,
                                     step_cost,
                                     ivCollisionCheckAccuracy,
                                     max_hash_size,
                                     ivCellSize,
                                     ivNumAngleBins,
                                     ivForwardSearch,
                                     sqrt(max_x * max_x + max_y * max_y) * 1.5,
                                     num_random_nodes,
                                     random_node_dist,
                                     heuristic_scale));

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
  if (ivForwardSearch)
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
      new ARAPlanner(ivPlannerEnvironmentPtr.get(), ivForwardSearch));
  }
  else if (ivPlannerType == "ADPlanner")
  {
    ivPlannerPtr.reset(
      new ADPlanner(ivPlannerEnvironmentPtr.get(), ivForwardSearch));
  }
  else if (ivPlannerType == "RSTARPlanner")
  {
    RSTARPlanner* p =
      new RSTARPlanner(ivPlannerEnvironmentPtr.get(), ivForwardSearch);
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
  ivPathExists = false;

  // commit start/goal poses to the environment
  ivPlannerEnvironmentPtr->updateStart(ivStartFootLeft, ivStartFootRight);
  ivPlannerEnvironmentPtr->updateGoal(ivGoalFootLeft, ivGoalFootRight);
  ivPlannerEnvironmentPtr->updateHeuristicValues();

  int ret = 0;
  MDPConfig mdp_config;
  std::vector<int> solution_state_ids;

  ivPlannerEnvironmentPtr->InitializeEnv(NULL);
  ivPlannerEnvironmentPtr->InitializeMDPCfg(&mdp_config);

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
  ret = ivPlannerPtr->replan(ivMaxSearchTime, &solution_state_ids,
                             &path_cost);
  ivPathCost = double(path_cost) / FootstepPlannerEnvironment::cvMmScale;

  if (ret && solution_state_ids.size() > 0 &&
      pathIsNew(solution_state_ids))
  {
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
      ivPathExists = true;

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
  for(unsigned int i = 0; i < state_ids.size(); ++i)
  {
    if (!ivPlannerEnvironmentPtr->getState(state_ids[i], &s))
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
  // reset the previously calculated paths
  ivPath.clear();
  ivPlanningStatesIds.clear();
  // reset the planner
  ivPlannerEnvironmentPtr->reset();
  //ivPlannerPtr->force_planning_from_scratch();
  setPlanner();
}


bool
FootstepPlanner::plan()
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

  reset();
  // start the planning and return success
  return run();
}


bool
FootstepPlanner::replan()
{
  if (!ivMapPtr)
  {
    ROS_ERROR("FootstepPlanner has no map for re-planning yet.");
    return false;
  }
  if (!ivGoalPoseSetUp || !ivStartPoseSetUp)
  {
    ROS_ERROR("FootstepPlanner has not set start and/or goal pose yet.");
    return false;
  }
  // Workaround for R* and ARA: need to reinit. everything
  if (ivPlannerType == "RSTARPlanner" || ivPlannerType == "ARAPlanner")
  {
    ROS_INFO("Reset planning information because planner cannot handle "
             "replanning.");
    reset();
  }

  return run();
}


bool
FootstepPlanner::plan(const geometry_msgs::PoseStampedConstPtr& start,
                      const geometry_msgs::PoseStampedConstPtr& goal)
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

  return plan();
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

  humanoid_nav_msgs::StepTarget foot;
  state_iter_t path_iter;
  for (path_iter = getPathBegin(); path_iter != getPathEnd(); path_iter++)
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

    resp.footsteps.push_back(foot);
  }
  resp.result = result;

  // return true since service call was successful (independent from the
  // success of the planning call)
  return true;
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
      // this check enforces a planning from scratch if necessary (dependent on
      // planning direction)
	  if (ivForwardSearch)
        replan();
	  else
        plan();
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
      // this check enforces a planning from scratch if necessary (dependent on
      // planning direction)
      if (ivForwardSearch)
        plan();
      else
        replan();
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
    replan();
  }
}


bool
FootstepPlanner::setGoal(
    const geometry_msgs::PoseStampedConstPtr& goal_pose)
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
FootstepPlanner::setStart(
    const geometry_msgs::PoseStampedConstPtr& start_pose)
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
FootstepPlanner::updateMap(const GridMap2DPtr& map)
{
  // store old map locally
  GridMap2DPtr old_map = ivMapPtr;
  // store new map
  ivMapPtr.reset();
  ivMapPtr = map;
  // update map of planning environment
  ivPlannerEnvironmentPtr->updateMap(map);

  if (old_map && ivPathExists)
  {
    // updating the environment currently means resetting all previous planning
    // information
    updateEnvironment(old_map);

    return true;
  }
  else
  {
    return false;
  }
}


void
FootstepPlanner::updateEnvironment(const GridMap2DPtr& old_map)
{
  ROS_INFO("Reset old information in new planning task");
  // reset planner
  reset();

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
  //                        num_changed_cells++;
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
  for (unsigned i = 0; i < new_path.size(); i++)
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

  for (unsigned i = 0; i < num_footsteps; i++)
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
        state_id_it++)
    {
      point.x = cell_2_state(state_id_it->first, ivCellSize);
      point.y = cell_2_state(state_id_it->second, ivCellSize);
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
  state_iter_t path_iter = getPathBegin();
  for(; path_iter != getPathEnd(); path_iter++)
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
        state_id_iter++)
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
  for(path_iter = getPathBegin(); path_iter != getPathEnd(); path_iter++)
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
  float x_shift = cos_theta * ivOriginFootShiftX -
      sin_theta * ivOriginFootShiftY;
  float y_shift;
  if (foot_pose.getLeg() == LEFT)
    y_shift = sin_theta * ivOriginFootShiftX +
    cos_theta * ivOriginFootShiftY;
  else // leg == RLEG
      y_shift = sin_theta * ivOriginFootShiftX -
      cos_theta * ivOriginFootShiftY;
  marker->pose.position.x = foot_pose.getX() + x_shift;
  marker->pose.position.y = foot_pose.getY() + y_shift;
  marker->pose.position.z = ivFootsizeZ / 2.0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(foot_pose.getTheta()),
                        marker->pose.orientation);

  marker->scale.x = ivFootsizeX; // - 0.01;
  marker->scale.y = ivFootsizeY; // - 0.01;
  marker->scale.z = ivFootsizeZ;

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

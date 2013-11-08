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

#include <footstep_planner/FootstepNavigation.h>


namespace footstep_planner
{
FootstepNavigation::FootstepNavigation()
: ivIdFootRight("/r_sole"),
  ivIdFootLeft("/l_sole"),
  ivIdMapFrame("map"),
  ivExecutingFootsteps(false),
  ivFootstepsExecution("footsteps_execution", true),
  ivExecutionShift(2),
  ivControlStepIdx(-1),
  ivResetStepIdx(0)
{
  // private NodeHandle for parameters and private messages (debug / info)
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public;

  // service
  ivFootstepSrv =
    nh_public.serviceClient<humanoid_nav_msgs::StepTargetService>(
      "footstep_srv");
  ivClipFootstepSrv =
    nh_public.serviceClient<humanoid_nav_msgs::ClipFootstep>(
      "clip_footstep_srv");

  // subscribers
  ivGridMapSub =
    nh_public.subscribe<nav_msgs::OccupancyGrid>(
      "map", 1, &FootstepNavigation::mapCallback, this);
  ivGoalPoseSub =
    nh_public.subscribe<geometry_msgs::PoseStamped>(
      "goal", 1, &FootstepNavigation::goalPoseCallback, this);

  // read parameters from config file:
  nh_private.param("rfoot_frame_id", ivIdFootRight, ivIdFootRight);
  nh_private.param("lfoot_frame_id", ivIdFootLeft, ivIdFootLeft);

  nh_private.param("accuracy/footstep/x", ivAccuracyX, 0.01);
  nh_private.param("accuracy/footstep/y", ivAccuracyY, 0.01);
  nh_private.param("accuracy/footstep/theta", ivAccuracyTheta, 0.1);

  nh_private.param("accuracy/cell_size", ivCellSize, 0.005);
  nh_private.param("accuracy/num_angle_bins", ivNumAngleBins, 128);

  nh_private.param("forward_search", ivForwardSearch, false);

  nh_private.param("feedback_frequency", ivFeedbackFrequency, 5.0);
  nh_private.param("safe_execution", ivSafeExecution, true);

  nh_private.param("foot/max/step/x", ivMaxStepX, 0.07);
  nh_private.param("foot/max/step/y", ivMaxStepY, 0.15);
  nh_private.param("foot/max/step/theta", ivMaxStepTheta, 0.3);
  nh_private.param("foot/max/inverse/step/x", ivMaxInvStepX, -0.03);
  nh_private.param("foot/max/inverse/step/y", ivMaxInvStepY, 0.09);
  nh_private.param("foot/max/inverse/step/theta", ivMaxInvStepTheta, -0.01);

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
  ivStepRange.clear();
  ivStepRange.reserve(step_range_x.size());
  double x, y;
  for (int i = 0; i < step_range_x.size(); ++i)
  {
    x = (double)step_range_x[i];
    y = (double)step_range_y[i];
    ivStepRange.push_back(std::pair<double, double>(x, y));
  }
  // insert first point again at the end!
  ivStepRange.push_back(ivStepRange[0]);
}


FootstepNavigation::~FootstepNavigation()
{}


bool
FootstepNavigation::plan()
{
  if (!updateStart())
  {
    ROS_ERROR("Start pose not accessible!");
    return false;
  }

  if (ivPlanner.plan())
  {
    startExecution();
    return true;
  }
  // path planning unsuccessful
  return false;
}


bool
FootstepNavigation::replan()
{
  if (!updateStart())
  {
    ROS_ERROR("Start pose not accessible!");
    return false;
  }

  bool path_existed = ivPlanner.pathExists();

  // calculate path by replanning (if no planning information exists
  // this call is equal to ivPlanner.plan())
  if (ivPlanner.replan())
  {
    startExecution();
    return true;
  }
  else if (path_existed)
  {
    ROS_INFO("Replanning unsuccessful. Reseting previous planning "
             "information.");
    if (ivPlanner.plan())
    {
      startExecution();
      return true;
    }
  }
  // path planning unsuccessful
  ivExecutingFootsteps = false;
  return false;
}


void
FootstepNavigation::startExecution()
{
  if (ivSafeExecution)
  {
    ivFootstepExecutionPtr.reset(
      new boost::thread(
        boost::bind(&FootstepNavigation::executeFootsteps, this)));
  }
  else
  {
    // ALTERNATIVE:
    executeFootstepsFast();
  }
}


void
FootstepNavigation::executeFootsteps()
{
  if (ivPlanner.getPathSize() <= 1)
    return;

  // lock this thread
  ivExecutingFootsteps = true;

  ROS_INFO("Start walking towards the goal.");

  humanoid_nav_msgs::StepTarget step;
  humanoid_nav_msgs::StepTargetService step_srv;

  tf::Transform from;
  std::string support_foot_id;

  // calculate and perform relative footsteps until goal is reached
  state_iter_t to_planned = ivPlanner.getPathBegin();
  if (to_planned == ivPlanner.getPathEnd())
  {
    ROS_ERROR("No plan available. Return.");
    return;
  }

  const State* from_planned = to_planned.base();
  to_planned++;
  while (to_planned != ivPlanner.getPathEnd())
  {
    try
    {
      boost::this_thread::interruption_point();
    }
    catch (const boost::thread_interrupted&)
    {
      // leave this thread
      return;
    }

    if (from_planned->getLeg() == RIGHT)
      support_foot_id = ivIdFootRight;
    else // support_foot = LLEG
      support_foot_id = ivIdFootLeft;

    // try to get real placement of the support foot
    if (getFootTransform(support_foot_id, ivIdMapFrame, ros::Time::now(),
                         ros::Duration(0.5), &from))
    {
      // calculate relative step and check if it can be performed
      if (getFootstep(from, *from_planned, *to_planned, &step))
      {
        step_srv.request.step = step;
        ivFootstepSrv.call(step_srv);
      }
      // ..if it cannot be performed initialize replanning
      else
      {
        ROS_INFO("Footstep cannot be performed. Replanning necessary.");

        replan();
        // leave the thread
        return;
      }
    }
    else
    {
      // if the support foot could not be received wait and try again
      ros::Duration(0.5).sleep();
      continue;
    }

    from_planned = to_planned.base();
    to_planned++;
  }
  ROS_INFO("Succeeded walking to the goal.\n");

  // free the lock
  ivExecutingFootsteps = false;
}


void
FootstepNavigation::executeFootstepsFast()
{
  if (ivPlanner.getPathSize() <= 1)
	return;

  // lock the planning and execution process
  ivExecutingFootsteps = true;

  // make sure the action client is connected to the action server
  ivFootstepsExecution.waitForServer();

  humanoid_nav_msgs::ExecFootstepsGoal goal;
  State support_leg;
  if (ivPlanner.getPathBegin()->getLeg() == RIGHT)
    support_leg = ivPlanner.getStartFootRight();
  else // leg == LEFT
    support_leg = ivPlanner.getStartFootLeft();
  if (getFootstepsFromPath(support_leg, 1, goal.footsteps))
  {
    goal.feedback_frequency = ivFeedbackFrequency;
    ivControlStepIdx = 0;
    ivResetStepIdx = 0;

    // start the execution via action; _1, _2 are place holders for
    // function arguments (see boost doc)
    ivFootstepsExecution.sendGoal(
      goal,
      boost::bind(&FootstepNavigation::doneCallback, this, _1, _2),
      boost::bind(&FootstepNavigation::activeCallback, this),
      boost::bind(&FootstepNavigation::feedbackCallback, this, _1));
  }
  else
  {
    // free the lock
    ivExecutingFootsteps = false;

    replan();
  }
}


void
FootstepNavigation::activeCallback()
{
	// lock the execution
	ivExecutingFootsteps = true;

	ROS_INFO("Start walking towards the goal.");
}


void
FootstepNavigation::doneCallback(
	const actionlib::SimpleClientGoalState& state,
	const humanoid_nav_msgs::ExecFootstepsResultConstPtr& result)
{
	if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Succeeded walking to the goal.");
	else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
		ROS_INFO("Preempted walking to the goal.");
	// TODO: distinct between further states??
	else
		ROS_INFO("Failed walking to the goal.");

	// free the lock
	ivExecutingFootsteps = false;
}


void
FootstepNavigation::feedbackCallback(
	const humanoid_nav_msgs::ExecFootstepsFeedbackConstPtr& fb)
{
	int executed_steps_idx = fb->executed_footsteps.size() - ivExecutionShift;
	// make sure at least one step has been performed
	if (executed_steps_idx < 0)
    return;
	// if the currently executed footstep equals the currently observed one
	// everything is ok
	if (executed_steps_idx == ivControlStepIdx)
    return;

	// get planned foot placement
  const State& planned = *(ivPlanner.getPathBegin() + ivControlStepIdx + 1 +
                           ivResetStepIdx);
  // get executed foot placement
  tf::Transform executed_tf;
  std::string foot_id;
  if (planned.getLeg() == RIGHT)
    foot_id = ivIdFootRight;
  else
    foot_id = ivIdFootLeft;

  if (!getFootTransform(foot_id, ivIdMapFrame, ros::Time::now(),
		                    ros::Duration(0.5), &executed_tf))
  {
    State executed(executed_tf.getOrigin().x(), executed_tf.getOrigin().y(),
                   tf::getYaw(executed_tf.getRotation()), planned.getLeg());
    ivFootstepsExecution.cancelGoal();
    humanoid_nav_msgs::ExecFootstepsGoal goal;
    // try to reach the calculated path
    if (getFootstepsFromPath(executed, executed_steps_idx + ivResetStepIdx,
                             goal.footsteps))
    {
      goal.feedback_frequency = ivFeedbackFrequency;
      // adjust the internal counters
      ivResetStepIdx += ivControlStepIdx + 1;
      ivControlStepIdx = 0;

      // restart the footstep execution
      ivFootstepsExecution.sendGoal(
        goal,
        boost::bind(&FootstepNavigation::doneCallback, this, _1, _2),
        boost::bind(&FootstepNavigation::activeCallback, this),
        boost::bind(&FootstepNavigation::feedbackCallback, this, _1));
    }
    // the previously calculated path cannot be reached so we have plan
    // a new path
    else
    {
      replan();
    }
  }

  State executed(executed_tf.getOrigin().x(), executed_tf.getOrigin().y(),
                 tf::getYaw(executed_tf.getRotation()), planned.getLeg());

  // check if the currently executed footstep is no longer observed (i.e.
  // the robot no longer follows its calculated path)
  if (executed_steps_idx >= ivControlStepIdx + 2)
	{
    ivFootstepsExecution.cancelGoal();

    ROS_DEBUG("Footstep execution incorrect.");

    humanoid_nav_msgs::ExecFootstepsGoal goal;
    // try to reach the calculated path
    if (getFootstepsFromPath(executed, executed_steps_idx + ivResetStepIdx,
                             goal.footsteps))
    {
      ROS_INFO("Try to reach calculated path.");

      goal.feedback_frequency = ivFeedbackFrequency;
      // adjust the internal counters
      ivResetStepIdx += ivControlStepIdx + 1;
      ivControlStepIdx = 0;

      // restart the footstep execution
      ivFootstepsExecution.sendGoal(
        goal,
        boost::bind(&FootstepNavigation::doneCallback, this, _1, _2),
        boost::bind(&FootstepNavigation::activeCallback, this),
        boost::bind(&FootstepNavigation::feedbackCallback, this, _1));
    }
    // the previously calculated path cannot be reached so we have plan
    // a new path
    else
    {
      replan();
    }

    return;
	}
    // check the currently observed footstep
	else
	{
    ROS_DEBUG("planned (%f, %f, %f, %i) vs. executed (%f, %f, %f, %i)",
              planned.getX(), planned.getY(), planned.getTheta(),
              planned.getLeg(),
              executed.getX(), executed.getY(), executed.getTheta(),
              executed.getLeg());

    // adjust the internal step counters if the footstep has been
    // performed correctly; otherwise check in the next iteration if
    // the step really has been incorrect
    if (performanceValid(planned, executed))
      ivControlStepIdx++;
    else
      ROS_DEBUG("Invalid step. Wait next step update before declaring"
                " step incorrect.");
	}
}


void
FootstepNavigation::goalPoseCallback(
  const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  // check if the execution is locked
  if (ivExecutingFootsteps)
  {
    ROS_INFO("Already performing a navigation task. Wait until it is "
             "finished.");
    return;
  }

  if (setGoal(goal_pose))
  {
    // this check enforces a planning from scratch if necessary (dependent on
    // planning direction)
	  if (ivForwardSearch)
	    replan();
	  else
	    plan();
  }
}


void
FootstepNavigation::mapCallback(
  const nav_msgs::OccupancyGridConstPtr& occupancy_map)
{
  // stop execution if an execution was performed
  if (ivExecutingFootsteps)
  {
    if (ivSafeExecution)
    {
      // interrupt the thread and wait until it has finished its execution
  	  ivFootstepExecutionPtr->interrupt();
      ivFootstepExecutionPtr->join();
    }
    else
    {
  		ivFootstepsExecution.cancelAllGoals();
    }
  }

  gridmap_2d::GridMap2DPtr map(new gridmap_2d::GridMap2D(occupancy_map));
  ivIdMapFrame = map->getFrameID();

  // updates the map and starts replanning if necessary
  if (ivPlanner.updateMap(map))
  {
    replan();
  }
}


bool
FootstepNavigation::setGoal(const geometry_msgs::PoseStampedConstPtr goal_pose)
{
  return setGoal(goal_pose->pose.position.x,
                 goal_pose->pose.position.y,
                 tf::getYaw(goal_pose->pose.orientation));
}


bool
FootstepNavigation::setGoal(float x, float y, float theta)
{
	return ivPlanner.setGoal(x, y, theta);
}


bool
FootstepNavigation::updateStart()
{
  ros::Duration(0.5).sleep();

  tf::Transform foot_left, foot_right;
  {
    // get real placement of the feet
	  if (!getFootTransform(ivIdFootLeft, ivIdMapFrame, ros::Time::now(),
      		                ros::Duration(0.5), &foot_left))
	  {
	    if (ivPlanner.pathExists())
	    {
	      ivExecutingFootsteps = false;
	    }
	    return false;
	  }
    if (!getFootTransform(ivIdFootRight, ivIdMapFrame, ros::Time::now(),
    		                  ros::Duration(0.5), &foot_right))
    {
      if (ivPlanner.pathExists())
      {
        ivExecutingFootsteps = false;
      }
      return false;
    }
  }
  State left(foot_left.getOrigin().x(), foot_left.getOrigin().y(),
  		       tf::getYaw(foot_left.getRotation()), LEFT);
  State right(foot_right.getOrigin().x(), foot_right.getOrigin().y(),
              tf::getYaw(foot_right.getRotation()), RIGHT);

  ROS_INFO("Robot standing at (%f, %f, %f, %i) (%f, %f, %f, %i).",
		       left.getX(), left.getY(), left.getTheta(), left.getLeg(),
		       right.getX(), right.getY(), right.getTheta(), right.getLeg());

  return ivPlanner.setStart(left, right);
}


bool
FootstepNavigation::getFootstep(const tf::Pose& from,
                                const State& from_planned,
		                            const State& to,
		                            humanoid_nav_msgs::StepTarget* footstep)
{
  // get footstep to reach 'to' from 'from'
  tf::Transform step = from.inverse() *
                       tf::Pose(tf::createQuaternionFromYaw(to.getTheta()),
                                tf::Point(to.getX(), to.getY(), 0.0));

  // set the footstep
  footstep->pose.x = step.getOrigin().x();
  footstep->pose.y = step.getOrigin().y();
  footstep->pose.theta = tf::getYaw(step.getRotation());
  if (to.getLeg() == LEFT)
    footstep->leg = humanoid_nav_msgs::StepTarget::left;
  else // to.leg == RIGHT
    footstep->leg = humanoid_nav_msgs::StepTarget::right;


  /* check if the footstep can be performed by the NAO robot ******************/

  // check if the step lies within the executable range
  if (performable(*footstep))
  {
    return true;
  }
  else
  {
    // check if there is only a minor divergence between the current support
	// foot and the foot placement used during the plannig task: in such a case
	// perform the step that has been used during the planning
    float step_diff_x = fabs(from.getOrigin().x() - from_planned.getX());
    float step_diff_y = fabs(from.getOrigin().y() - from_planned.getY());
    float step_diff_theta = fabs(
        angles::shortest_angular_distance(
            tf::getYaw(from.getRotation()), from_planned.getTheta()));
    if (step_diff_x < ivAccuracyX && step_diff_y < ivAccuracyY &&
        step_diff_theta < ivAccuracyTheta)
    {
	  step = tf::Pose(tf::createQuaternionFromYaw(from_planned.getTheta()),
	                  tf::Point(from_planned.getX(), from_planned.getY(), 0.0)
	                  ).inverse() *
		     tf::Pose(tf::createQuaternionFromYaw(to.getTheta()),
				      tf::Point(to.getX(), to.getY(), 0.0));

	  footstep->pose.x = step.getOrigin().x();
	  footstep->pose.y = step.getOrigin().y();
	  footstep->pose.theta = tf::getYaw(step.getRotation());

	  return true;
    }

    return false;
  }

//  // ALTERNATIVE: clip the footstep into the executable range; if nothing was
//  // clipped: perform; if too much was clipped: do not perform
//  humanoid_nav_msgs::ClipFootstep footstep_clip;
//  footstep_clip.request.step = footstep;
//  ivClipFootstepSrv.call(footstep_clip);
//
//  if (performanceValid(footstep_clip))
//  {
//  	footstep.pose.x = footstep_clip.response.step.pose.x;
//  	footstep.pose.y = footstep_clip.response.step.pose.y;
//  	footstep.pose.theta = footstep_clip.response.step.pose.theta;
//  	return true;
//  }
//  else
//  {
//    return false;
//  }
}


bool
FootstepNavigation::getFootstepsFromPath(
  const State& current_support_leg, int starting_step_num,
  std::vector<humanoid_nav_msgs::StepTarget>& footsteps)
{
  humanoid_nav_msgs::StepTarget footstep;

  state_iter_t to_planned = ivPlanner.getPathBegin() + starting_step_num - 1;
  tf::Pose last(tf::createQuaternionFromYaw(current_support_leg.getTheta()),
                tf::Point(current_support_leg.getX(), current_support_leg.getY(),
                          0.0));
  const State* from_planned = to_planned.base();
  to_planned++;
  for (; to_planned != ivPlanner.getPathEnd(); to_planned++)
  {
    if (getFootstep(last, *from_planned, *to_planned, &footstep))
    {
      footsteps.push_back(footstep);
    }
    else
    {
      ROS_ERROR("Calculated path cannot be performed!");
      return false;
    }

    last = tf::Pose(tf::createQuaternionFromYaw(to_planned->getTheta()),
                    tf::Point(to_planned->getX(), to_planned->getY(), 0.0));
    from_planned = to_planned.base();
  }

  return true;
}


bool
FootstepNavigation::getFootTransform(const std::string& foot_id,
                                     const std::string& world_frame_id,
                                     const ros::Time& time,
                                     const ros::Duration& waiting_time,
                                     tf::Transform* foot)
{
  tf::StampedTransform stamped_foot_transform;
  try
  {
    ivTransformListener.waitForTransform(world_frame_id, foot_id, time,
                                         waiting_time);
    ivTransformListener.lookupTransform(world_frame_id, foot_id, time,
                                        stamped_foot_transform);
  }
  catch (const tf::TransformException& e)
  {
    ROS_WARN("Failed to obtain FootTransform from tf (%s)", e.what());
    return false;
  }

  foot->setOrigin(stamped_foot_transform.getOrigin());
  foot->setRotation(stamped_foot_transform.getRotation());

  return true;
}


bool
FootstepNavigation::performanceValid(float a_x, float a_y, float a_theta,
                                     float b_x, float b_y, float b_theta)
{
  return (fabs(a_x - b_x) < ivAccuracyX &&
          fabs(a_y - b_y) < ivAccuracyY &&
          fabs(angles::shortest_angular_distance(a_theta, b_theta)) <
            ivAccuracyTheta);
}


bool
FootstepNavigation::performanceValid(
  const humanoid_nav_msgs::ClipFootstep& step)
{
  return performanceValid(step.request.step.pose.x,
                          step.request.step.pose.y,
                          step.request.step.pose.theta,
                          step.response.step.pose.x,
                          step.response.step.pose.y,
                          step.response.step.pose.theta);
}


bool
FootstepNavigation::performanceValid(const State& planned,
                                     const State& executed)
{
  return performanceValid(
    planned.getX(), planned.getY(), planned.getTheta(),
    executed.getX(), executed.getY(), executed.getTheta());
}


bool
FootstepNavigation::performable(const humanoid_nav_msgs::StepTarget& footstep)
{
  float step_x = footstep.pose.x;
  float step_y = footstep.pose.y;
  float step_theta = footstep.pose.theta;

  if (footstep.leg == humanoid_nav_msgs::StepTarget::right)
  {
    step_y = -step_y;
    step_theta = -step_theta;
  }

  if (step_x + FLOAT_CMP_THR > ivMaxStepX ||
      step_x - FLOAT_CMP_THR < ivMaxInvStepX)
    return false;
  if (step_y + FLOAT_CMP_THR > ivMaxStepY ||
      step_y - FLOAT_CMP_THR < ivMaxInvStepY)
    return false;
  if (step_theta + FLOAT_CMP_THR > ivMaxStepTheta ||
      step_theta - FLOAT_CMP_THR < ivMaxInvStepTheta)
    return false;

  return performable(step_x, step_y);
}


bool
FootstepNavigation::performable(float step_x, float step_y)
{
  int cn = 0;

  // loop through all ivStepRange of the polygon
  for(unsigned int i = 0; i < ivStepRange.size() - 1; ++i)
  {
    if ((ivStepRange[i].second <= step_y &&
    	 ivStepRange[i + 1].second > step_y) ||
        (ivStepRange[i].second >= step_y &&
         ivStepRange[i + 1].second < step_y))
    {
      float vt = (float)(step_y - ivStepRange[i].second) /
        (ivStepRange[i + 1].second - ivStepRange[i].second);
      if (step_x <
          ivStepRange[i].first + vt *
            (ivStepRange[i + 1].first - ivStepRange[i].first))
      {
        ++cn;
      }
    }
  }
  return cn & 1;
}
}

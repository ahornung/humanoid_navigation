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

#include <footstep_planner/FootstepNavigation.h>


namespace footstep_planner
{
	FootstepNavigation::FootstepNavigation()
        : ivLastRobotTime(0),
          ivIdFootRight("/r_sole"),
          ivIdFootLeft("/l_sole"),
          ivIdMapFrame("map"),
          ivExecutingFootsteps(false),
          ivFootstepsExecution("footsteps_execution", true),
          ivEqualStepsThreshold(-1),
          ivEqualStepsNum(-1),
          ivExecutionShift(2),
          ivLastStepNum(-1)
    {
        // private NodeHandle for parameters and private messages (debug / info)
        ros::NodeHandle nh_private("~");
        ros::NodeHandle nh_public;

        // service
        ivFootstepSrv = nh_public.serviceClient<
                humanoid_nav_msgs::StepTargetService>("footstep_srv");
        ivClipFootstepSrv = nh_public.serviceClient<
                humanoid_nav_msgs::ClipFootstep>("clip_footstep_srv");

        // subscribers
        ivGridMapSub = nh_public.subscribe<nav_msgs::OccupancyGrid>(
                "map", 1, &FootstepNavigation::mapCallback, this);
		ivGoalPoseSub = nh_public.subscribe<geometry_msgs::PoseStamped>(
		        "goal", 1, &FootstepNavigation::goalPoseCallback, this);
        // subscribe to robot pose to get latest time
        ivRobotPoseSub = nh_public.subscribe<
        		geometry_msgs::PoseWithCovarianceStamped>(
        				"amcl_pose", 5, &FootstepNavigation::robotPoseCallback,
        				this);

        // read parameters from config file:
        nh_private.param("rfoot_frame_id", ivIdFootRight, ivIdFootRight);
        nh_private.param("lfoot_frame_id", ivIdFootLeft, ivIdFootLeft);

        nh_private.param("accuracy/footstep/x", ivAccuracyX, 0.005);
        nh_private.param("accuracy/footstep/y", ivAccuracyY, 0.005);
        nh_private.param("accuracy/footstep/theta", ivAccuracyTheta, 0.05);

        nh_private.param("accuracy/cell_size", ivCellSize, 0.005);
        nh_private.param("accuracy/num_angle_bins", ivNumAngleBins, 128);

        nh_private.param("feedback_frequence", ivFeedbackFrequence, 5.0);

        ivEqualStepsThreshold = (int) ((0.5 / ivFeedbackFrequence) * 0.7);

//        ROS_INFO("accuracy (%f, %f, %f)",
//		         ivAccuracyX, ivAccuracyY, ivAccuracyTheta);
//        ROS_INFO("equal steps thr: %i", ivEqualStepsThreshold);
    }


	FootstepNavigation::~FootstepNavigation()
    {}


	void
	FootstepNavigation::run()
	{
		// lock the planning and execution process
		ivExecutingFootsteps = true;
		// calculate path
        if (ivPlanner.plan())
			// execution thread
			boost::thread footstepExecutionThread(
					&FootstepNavigation::executeFootsteps, this);

//        	// ALTERNATIVE:
//        	executeFootsteps_alt();
        else
        	// free the lock if the planning failed
        	ivExecutingFootsteps = false;
	}


    void
    FootstepNavigation::executeFootsteps()
    {
    	if (ivPlanner.getPathSize() <= 1)
    		return;

    	// lock this thread
    	ivExecutingFootsteps = true;

    	humanoid_nav_msgs::StepTarget step;
    	humanoid_nav_msgs::StepTargetService step_srv;

    	tf::Transform from_transform;
    	State from;
    	std::string support_foot_id;
    	Leg from_leg;

    	// calculate and perform relative footsteps until goal is reached
    	bool performable = false;
    	state_iter_t to_planned = ivPlanner.getPathBegin();
    	if (to_planned == ivPlanner.getPathEnd())
    	{
    		ROS_ERROR("No plan available. Return.");
    		return;
    	}

    	to_planned++;
    	while (to_planned != ivPlanner.getPathEnd())
    	{
    		if (to_planned->leg == LEFT)
    		{
    			support_foot_id = ivIdFootRight;
    			from_leg = RIGHT;
    		}
    		else // support_foot = LLEG
    		{
    			support_foot_id = ivIdFootLeft;
    			from_leg = LEFT;
    		}
    		{
    			boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    			// get real placement of the support foot
    			getFootTransform(support_foot_id, ivIdMapFrame,
    			                 ros::Time::now(), from_transform);
    		}
			from = State(from_transform.getOrigin().x(),
			             from_transform.getOrigin().y(),
			             tf::getYaw(from_transform.getRotation()),
						 from_leg);
    		// calculate relative step
    		performable = getFootstep(from, *to_planned, step);

    		to_planned++;

    		// if step cannot be performed initialize replanning..
    		if (!performable)
    		{
    			ROS_INFO("Footstep cannot be performed: new path planning "
    					 "necessary");

    			if (updateStart())
    			{
                    if (ivPlanner.replan())
                    {
                        // if replanning was successful start new execution thread
                        boost::thread footstepExecutionThread(
                                &FootstepNavigation::executeFootsteps, this);
                    }
    			}
    	        else
    	            ROS_ERROR("start pose not accessible: check your odometry");

                // leave this thread
                return;
    		}
    		// ..otherwise perform step
    		else
    		{
    			step_srv.request.step = step;
    			ivFootstepSrv.call(step_srv);
    		}
    	}

    	// free the lock
    	ivExecutingFootsteps = false;
    }


    void
    FootstepNavigation::executeFootsteps_alt()
    {
    	if (ivPlanner.getPathSize() <= 1)
    		return;

    	// make sure the action client is connected to the action server
    	ivFootstepsExecution.waitForServer();

    	humanoid_nav_msgs::ExecFootstepsGoal goal;
    	State support_leg;
    	if (ivPlanner.getPathBegin()->leg == RIGHT)
    		support_leg = ivPlanner.getStartFootRight();
    	else // leg == LEFT
    		support_leg = ivPlanner.getStartFootLeft();
    	if (getFootstepsFromPath(support_leg, 1, goal.footsteps))
    	{
			goal.feedback_frequence = ivFeedbackFrequence;
			ivEqualStepsNum = 0;
			ivLastStepNum = 0;

			ROS_INFO("Start walking towards the goal.");
			// start the execution via action; _1, _2 are place holders for
			// function arguments (see boost doc)
			ivFootstepsExecution.sendGoal(
				goal,
				boost::bind(&FootstepNavigation::doneCallback, this, _1, _2),
				boost::bind(&FootstepNavigation::activeCallback, this),
				boost::bind(&FootstepNavigation::feedbackCallback, this, _1));
    	}
    }


    void
    FootstepNavigation::activeCallback()
    {
    	// lock the execution
    	ivExecutingFootsteps = true;

    	ROS_INFO("Start walking towards goal.");
    }


    void
    FootstepNavigation::doneCallback(
    		const actionlib::SimpleClientGoalState& state,
    		const humanoid_nav_msgs::ExecFootstepsResultConstPtr& result)
    {
    	// TODO: check if goal pose is the requested pose?
    	if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    		ROS_INFO("Succeeded walking to the goal.");
    	else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
    		ROS_INFO("Preempted walking to the goal.");
    	// TODO: distinct between further states
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
    	if (executed_steps_idx < 0)
    	    return;
    	else if (executed_steps_idx == ivLastStepNum)
    	    ivEqualStepsNum++;

    	if (ivEqualStepsNum != ivEqualStepsThreshold)
    	    return;

    	ROS_INFO("%i / %i (%i)", executed_steps_idx, ivLastStepNum, ivEqualStepsNum);

    	ivLastStepNum++;
        ivEqualStepsNum = 0;

    	const State& planned = *(ivPlanner.getPathBegin() + executed_steps_idx);

    	tf::Transform executed_tf;
    	std::string foot_id;
    	if (planned.leg == RIGHT)
    	    foot_id = ivIdFootRight;
    	else
    	    foot_id = ivIdFootLeft;
    	getFootTransform(foot_id, ivIdMapFrame, ros::Time::now(), executed_tf);
    	State executed(executed_tf.getOrigin().x(), executed_tf.getOrigin().y(),
    	               tf::getYaw(executed_tf.getRotation()), planned.leg);

    	ROS_INFO("planned (%f, %f, %f, %i) vs. executed (%f, %f, %f, %i)",
    	         planned.x, planned.y, planned.theta, planned.leg,
    	         executed.x, executed.y, executed.theta, executed.leg);
    	ROS_INFO("valid? %i\n", performanceValid(planned, executed));
    }


    void
    FootstepNavigation::goalPoseCallback(
            const geometry_msgs::PoseStampedConstPtr& goal_pose)
    {
    	// check if the execution is locked
        if (ivExecutingFootsteps)
        {
            ROS_INFO("Already performing a navigation task. Wait until it is"
			         "finished.");
            return;
        }

    	if (setGoal(goal_pose))
    	{
    		if (updateStart())
    			run();
    		else
    			ROS_ERROR("Start pose not accessible: check your odometry");
    	}
    }


    void
    FootstepNavigation::robotPoseCallback(
            const geometry_msgs::PoseWithCovarianceStampedConstPtr& robot_pose)
    {
    	boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    	ivLastRobotTime = robot_pose->header.stamp;
    }


	void
    FootstepNavigation::mapCallback(
            const nav_msgs::OccupancyGridConstPtr& occupancy_map)
    {
        GridMap2DPtr map(new GridMap2D(occupancy_map));
        ivIdMapFrame = map->getFrameID();
        ivPlanner.updateMap(map);
    }


    bool
    FootstepNavigation::setGoal(
            const geometry_msgs::PoseStampedConstPtr& goal_pose)
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
        tf::Transform foot_left, foot_right;
        {
            boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
            // get real placement of the feet
            getFootTransform(ivIdFootLeft, ivIdMapFrame, ivLastRobotTime,
                             foot_left);
            getFootTransform(ivIdFootRight, ivIdMapFrame, ivLastRobotTime,
                             foot_right);
        }
        State left(foot_left.getOrigin().x(), foot_left.getOrigin().y(),
        		tf::getYaw(foot_left.getRotation()), LEFT);
        State right(foot_right.getOrigin().x(), foot_right.getOrigin().y(),
		            tf::getYaw(foot_right.getRotation()), RIGHT);

        return ivPlanner.setStart(left, right);
    }


    bool
    FootstepNavigation::getFootstep(const State& from, const State& to,
                                    humanoid_nav_msgs::StepTarget& footstep)
    {
        // calculate the necessary footstep to reach the foot placement
        double footstep_x, footstep_y, footstep_theta;
        get_footstep(from.x, from.y, from.theta, from.leg,
                     to.x, to.y, to.theta,
                     footstep_x, footstep_y, footstep_theta);

        footstep.pose.x = footstep_x;
        if (from.leg == RIGHT)
        {
            footstep.pose.y = footstep_y;
            footstep.pose.theta = footstep_theta;
            footstep.leg = humanoid_nav_msgs::StepTarget::left;
        }
        else // from.leg == LEFT
        {
            footstep.pose.y = -footstep_y;
            footstep.pose.theta = -footstep_theta;
            footstep.leg = humanoid_nav_msgs::StepTarget::right;
        }

        humanoid_nav_msgs::ClipFootstep footstep_srv;
        footstep_srv.request.step = footstep;
        ivClipFootstepSrv.call(footstep_srv);

        if (performable(footstep_srv))
        {
            footstep.pose.x = footstep_srv.response.step.pose.x;
            footstep.pose.y = footstep_srv.response.step.pose.y;
            footstep.pose.theta = footstep_srv.response.step.pose.theta;
            return true;
        }
        else
        {
            return false;
        }
    }


    bool
    FootstepNavigation::getFootstepsFromPath(
    		const State& current_support_leg, int starting_step_num,
    		std::vector<humanoid_nav_msgs::StepTarget>& footsteps)
    {
    	humanoid_nav_msgs::StepTarget footstep;

    	state_iter_t current = ivPlanner.getPathBegin() + starting_step_num;
    	State last = current_support_leg;
    	bool performable = false;
    	for (; current != ivPlanner.getPathEnd(); current++)
    	{
    		ROS_INFO("(%f, %f, %f, %i)",
			         current->x, current->y, current->theta, current->leg);

    		performable = getFootstep(last, *current, footstep);
    		if (!performable)
    		{
	        	ROS_ERROR("Calculated path cannot be performed!");
    			return false;
    		}
    		else
    		{
    			footsteps.push_back(footstep);
    		}
    		last = *current;
    	}

    	return true;
    }


    void
    FootstepNavigation::getFootTransform(
            const std::string& foot_id, const std::string& world_frame_id,
    		const ros::Time& time, tf::Transform& foot)
    {
    	tf::StampedTransform stamped_foot_transform;
    	try
    	{
			ivTransformListener.waitForTransform(world_frame_id, foot_id, time,
			                                     ros::Duration(0.1));
			ivTransformListener.lookupTransform(world_frame_id, foot_id,
			                                    ros::Time(0),
			                                    stamped_foot_transform);
    	}
    	catch (const tf::TransformException& e)
    	{
    		ROS_WARN("Failed to obtain FootTransform from tf (%s)", e.what());
    	}

    	foot.setOrigin(stamped_foot_transform.getOrigin());
    	foot.setRotation(stamped_foot_transform.getRotation());
    }


    bool
    FootstepNavigation::performable(const humanoid_nav_msgs::ClipFootstep& step)
    {
        return (fabs(step.request.step.pose.x - step.response.step.pose.x) <=
                ivAccuracyX &&
                fabs(step.request.step.pose.y - step.response.step.pose.y) <=
                ivAccuracyY &&
                fabs(step.request.step.pose.theta -
                step.response.step.pose.theta) <= ivAccuracyTheta &&
                step.request.step.leg == step.response.step.leg);
    }


    bool
    FootstepNavigation::performanceValid(const State& planned,
	                                     const State& executed)
    {
    	return (fabs(planned.x - executed.x) <= ivAccuracyX &&
    			fabs(planned.y - executed.y) <= ivAccuracyY &&
    			fabs(planned.theta - executed.theta) <= ivAccuracyTheta &&
    			planned.leg == executed.leg);
    }
}

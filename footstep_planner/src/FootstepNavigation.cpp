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
          ivExecutionShift(2),
          ivControlStepIdx(-1),
          ivResetStepIdx(0),
          ivSafeExecution(true)
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

        nh_private.param("feedback_frequency", ivFeedbackFrequency, 5.0);

        nh_private.param("protective_execution", ivSafeExecution, true);

        // check if each footstep can be performed by the NAO robot
        XmlRpc::XmlRpcValue footsteps_x;
        XmlRpc::XmlRpcValue footsteps_y;
        XmlRpc::XmlRpcValue footsteps_theta;
        ros::ServiceClient footstep_clip_srv = nh_public.serviceClient<
                humanoid_nav_msgs::ClipFootstep>("clip_footstep_srv");
        humanoid_nav_msgs::ClipFootstep performable_step;
        humanoid_nav_msgs::StepTarget step;
        // read the footstep parameters
        nh_private.getParam("footsteps/x", footsteps_x);
        nh_private.getParam("footsteps/y", footsteps_y);
        nh_private.getParam("footsteps/theta", footsteps_theta);
        if (footsteps_x.getType() != XmlRpc::XmlRpcValue::TypeArray)
            ROS_ERROR("Error reading footsteps/x from config file.");
        if (footsteps_y.getType() != XmlRpc::XmlRpcValue::TypeArray)
            ROS_ERROR("Error reading footsteps/y from config file.");
        if (footsteps_theta.getType() != XmlRpc::XmlRpcValue::TypeArray)
            ROS_ERROR("Error reading footsteps/theta from config file.");
        // check each footstep
        for(int i=0; i < footsteps_x.size(); i++)
        {
            double x = (double) footsteps_x[i];
            double y = (double) footsteps_y[i];
            double theta = (double) footsteps_theta[i];

            step.pose.x = x;
            step.pose.y = y;
            step.pose.theta = theta;
            step.leg = humanoid_nav_msgs::StepTarget::left;

            performable_step.request.step = step;
            footstep_clip_srv.call(performable_step);

            if (fabs(step.pose.x - performable_step.response.step.pose.x) >
                        FLOAT_CMP_THR ||
                fabs(step.pose.y - performable_step.response.step.pose.y) >
                        FLOAT_CMP_THR ||
                fabs(angles::shortest_angular_distance(
                    step.pose.theta,
                    performable_step.response.step.pose.theta)) >
                        FLOAT_CMP_THR)
            {
                ROS_ERROR("Step (%f, %f, %f) (%f, %f, %f) cannot be performed by the NAO "
                          "robot. Exit!", x, y, theta,
                          performable_step.response.step.pose.x,
                          performable_step.response.step.pose.y,
                          performable_step.response.step.pose.theta);
                exit(2);
            }
        }
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
            if (ivSafeExecution)
                // execution thread
                boost::thread footstepExecutionThread(
                        &FootstepNavigation::executeFootsteps, this);
            else
                // ALTERNATIVE:
                executeFootstepsFast();
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

    	to_planned++;
    	while (to_planned != ivPlanner.getPathEnd())
    	{
    		if (to_planned->getLeg() == LEFT)
    			support_foot_id = ivIdFootRight;
    		else // support_foot = LLEG
    			support_foot_id = ivIdFootLeft;
    		{
    			boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    			// get real placement of the support foot
    			getFootTransform(support_foot_id, ivIdMapFrame,
    			                 ros::Time::now(), from);
    		}

    		// calculate relative step and check if it can be performed
    		if (getFootstep(from, *to_planned, step))
    		{
    			step_srv.request.step = step;
    			ivFootstepSrv.call(step_srv);
    		}
    		// ..if it cannot be performed initialize replanning
    		else
    		{
    			ROS_INFO("Footstep cannot be performed. Replanning necessary");

    			if (updateStart())
                    if (ivPlanner.replan())
                    {
                        // start new execution thread
                        boost::thread footstepExecutionThread(
                                &FootstepNavigation::executeFootsteps, this);
                    }
                    else
                    {
                        ROS_INFO("Replanning not possible. Trying planning from"
                                 " the scratch.");
                        run();
                    }
    	        else
    	            ROS_ERROR("Start pose not accessible. Robot navigation "
    	                      "not possible.");

                // leave this thread
                return;
    		}

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
    	int executed_steps_idx = fb->executed_footsteps.size() -
    			                 ivExecutionShift;
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
        getFootTransform(foot_id, ivIdMapFrame, ros::Time::now(), executed_tf);
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
    	    if (getFootstepsFromPath(executed,
    	                             executed_steps_idx + ivResetStepIdx,
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
                    boost::bind(
                        &FootstepNavigation::doneCallback, this, _1, _2),
                    boost::bind(&FootstepNavigation::activeCallback, this),
                    boost::bind(
                        &FootstepNavigation::feedbackCallback, this, _1));
    	    }
    	    // the previously calculated path cannot be reached so we have plan
    	    // a new path
    	    else
    	    {
                ROS_INFO("Footstep cannot be performed. Replanning necessary");

                if (updateStart())
                    if (ivPlanner.replan())
                    {
                        // start new execution
                        executeFootstepsFast();
                    }
                    else
                    {
                        ROS_INFO("Replanning not possible. Trying planning from"
                                 " the scratch.");
                        run();
                    }
                else
                    ROS_ERROR("Start pose not accessible. Robot navigation "
                              "not possible.");
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
    FootstepNavigation::getFootstep(const tf::Pose& from, const State& to,
                                    humanoid_nav_msgs::StepTarget& footstep)
    {
        // get footstep to reach 'to' from 'from'
        tf::Transform step = from.inverse() * tf::Pose(
                tf::createQuaternionFromYaw(to.getTheta()),
                tf::Point(to.getX(), to.getY(), 0.0));

        // set the footstep
        footstep.pose.x = step.getOrigin().x();
        footstep.pose.y = step.getOrigin().y();
        footstep.pose.theta = tf::getYaw(step.getRotation());
        if (to.getLeg() == LEFT)
            footstep.leg = humanoid_nav_msgs::StepTarget::left;
        else // to.leg == RIGHT
            footstep.leg = humanoid_nav_msgs::StepTarget::right;

        // check if the footstep can be performed by the NAO robot
        humanoid_nav_msgs::ClipFootstep footstep_clip;
        footstep_clip.request.step = footstep;
        ivClipFootstepSrv.call(footstep_clip);
        return performanceValid(footstep_clip);
    }


    bool
    FootstepNavigation::getFootstepsFromPath(
    		const State& current_support_leg, int starting_step_num,
    		std::vector<humanoid_nav_msgs::StepTarget>& footsteps)
    {
    	humanoid_nav_msgs::StepTarget footstep;

    	state_iter_t current = ivPlanner.getPathBegin() + starting_step_num;
    	tf::Pose last(
            tf::createQuaternionFromYaw(current_support_leg.getTheta()),
            tf::Point(current_support_leg.getX(), current_support_leg.getY(),
                      0.0));
    	for (; current != ivPlanner.getPathEnd(); current++)
    	{
//    		ROS_INFO("(%f, %f, %f, %i)",
//			         current->x, current->y, current->theta, current->leg);

    		if (getFootstep(last, *current, footstep))
    		{
    		    footsteps.push_back(footstep);
    		}
    		else
    		{
	        	ROS_ERROR("Calculated path cannot be performed!");
    			return false;
    		}

    		last = tf::Pose(tf::createQuaternionFromYaw(current->getTheta()),
                            tf::Point(current->getX(), current->getY(), 0.0));
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
}

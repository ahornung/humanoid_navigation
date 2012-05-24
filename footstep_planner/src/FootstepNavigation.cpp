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
          ivFootstepsExecution("footsteps_execution", true)
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

        nh_private.param("feedback_rate", ivFeedbackRate, 5.0);
    }


	FootstepNavigation::~FootstepNavigation()
    {}


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
    			from.leg = RIGHT;
    		}
    		else // support_foot = LLEG
    		{
    			support_foot_id = ivIdFootLeft;
    			from.leg = LEFT;
    		}
    		{
    			boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    			// get real placement of the support foot
    			getFootTransform(support_foot_id, ivIdMapFrame,
    			                 ros::Time::now(), from_transform);
    		}
			from.x = from_transform.getOrigin().x();
			from.y = from_transform.getOrigin().y();
			from.theta = tf::getYaw(from_transform.getRotation());
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

    	humanoid_nav_msgs::StepTarget footstep;
    	humanoid_nav_msgs::ExecFootstepsGoal goal;

    	// get footsteps from the calculated path
    	state_iter_t current = ivPlanner.getPathBegin();
    	State last = *current;
    	current++;
    	bool performable = false;
    	for (; current != ivPlanner.getPathEnd(); current++)
    	{
    		ROS_INFO("(%f, %f, %f, %i)", current->x, current->y, current->theta,
			         current->leg);

    		performable = getFootstep(last, *current, footstep);
    		if (!performable)
    		{
	        	ROS_ERROR("Calculated path cannot be performed!");
    			return;
    		}
    		else
    		{
    			goal.footsteps.push_back(footstep);
    		}
    		last = *current;
    	}
    	goal.feedback_rate = ivFeedbackRate;

    	ROS_INFO("Start walking towards the goal.");

    	// start the execution via action
    	// NOTE: _1, _2 are place holders for function arguments (see boost doc)
    	ivFootstepsExecution.sendGoal(
    			goal,
    			boost::bind(&FootstepNavigation::doneCallback, this, _1, _2),
    			boost::bind(&FootstepNavigation::activeCallback, this),
    			boost::bind(&FootstepNavigation::feedbackCallback, this, _1));
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
    	else
    		ROS_INFO("Failed walking to the goal.");

    	// free the lock
    	ivExecutingFootsteps = false;
    }


    void
    FootstepNavigation::feedbackCallback(
    		const humanoid_nav_msgs::ExecFootstepsFeedbackConstPtr& fb)
    {}


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
	FootstepNavigation::run()
	{
		// lock the planning and execution process
		ivExecutingFootsteps = true;
		// calculate path
        if (ivPlanner.plan())
        {
			// execution thread
			boost::thread footstepExecutionThread(
					&FootstepNavigation::executeFootsteps, this);

//        	// ALTERNATIVE:
//        	executeFootsteps_alt();
        }
        else
        {
        	// free the lock if the planning failed
        	ivExecutingFootsteps = false;
        }
	}


	void
    FootstepNavigation::mapCallback(
            const nav_msgs::OccupancyGridConstPtr& occupancy_map)
    {
        boost::shared_ptr<GridMap2D> gridMap(new GridMap2D(occupancy_map));
        ivIdMapFrame = gridMap->getFrameID();
        ivPlanner.setMap(gridMap);
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
        get_footstep(from.x, from.y, from.theta, from.leg, to.x, to.y, to.theta,
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
			ivTransformListener.lookupTransform(world_frame_id, foot_id, ros::Time(0),
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


    // TODO: remove after debug
    void
    FootstepNavigation::debugFootstepExecution(const tf::Transform& from,
	                                           const State& from_planned,
	                                           const State& to_planned)
    {
    	ROS_INFO("--- compare calculated state and actual footstep placement");
		ROS_INFO("calculated state (%f, %f, %f, %i)", from_planned.x,
                 from_planned.y, from_planned.theta, from_planned.leg);
		ROS_INFO("actual state (%f, %f, %f, %i)", from.getOrigin().x(),
                 from.getOrigin().y(), tf::getYaw(from.getRotation()),
                 from_planned.leg);

		ROS_INFO("--- actual footstep placement and next (planned) footstep "
				 "placement");
		ROS_INFO("current (%f, %f, %f, %i)", from.getOrigin().x(),
				 from.getOrigin().y(), tf::getYaw(from.getRotation()),
				 from_planned.leg);
		ROS_INFO("next (%f, %f, %f, %i)", to_planned.x, to_planned.y,
		                                  to_planned.theta, to_planned.leg);

		ROS_INFO("--- relative footstep and practicability");
		double fs_x, fs_y, fs_theta;
		get_footstep(from.getOrigin().x(), from.getOrigin().y(),
		             tf::getYaw(from.getRotation()), from_planned.leg,
		             to_planned.x, to_planned.y, to_planned.theta,
		             fs_x, fs_y, fs_theta);
		ROS_INFO("footstep 'fs' (%f, %f, %f)", fs_x, fs_y, fs_theta);

		ROS_INFO("--- footstep clipping");
		humanoid_nav_msgs::ClipFootstep step_srv;
        step_srv.request.step.pose.x = fs_x;
        if (from_planned.leg == LEFT)
        {
            step_srv.request.step.pose.y = -fs_y;
            step_srv.request.step.pose.theta = -fs_theta;
            step_srv.request.step.leg = humanoid_nav_msgs::StepTarget::right;
        }
        else
        {
            step_srv.request.step.pose.y = fs_y;
            step_srv.request.step.pose.theta = fs_theta;
            step_srv.request.step.leg = humanoid_nav_msgs::StepTarget::left;
        }
        ivClipFootstepSrv.call(step_srv);
        bool performable_b = performable(step_srv);
        ROS_INFO("original footstep 'fs' (%f, %f, %f)",
                 step_srv.request.step.pose.x,
                 step_srv.request.step.pose.y,
                 step_srv.request.step.pose.theta);
        ROS_INFO("clipped footstep 'fs' (%f, %f, %f)",
                 step_srv.response.step.pose.x,
                 step_srv.response.step.pose.y,
                 step_srv.response.step.pose.theta);
        ROS_INFO("performable? %i", performable_b);
		ROS_INFO("---------------------------------------------------------\n");
    }
}

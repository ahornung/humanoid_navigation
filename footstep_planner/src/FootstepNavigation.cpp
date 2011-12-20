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
          ivFootIDRight("/RFoot_link"), ivFootIDLeft("/LFoot_link"),
          ivMapFrameID("map"),
          ivExecutingFootsteps(false)
    {
        // private NodeHandle for parameters and private messages (debug / info)
        ros::NodeHandle nh_private("~");
        ros::NodeHandle nh_public;

        // service
        ivFootstepService = nh_public.serviceClient<humanoid_nav_msgs::StepTargetService>("cmd_step_srv");

        // subscribers
        ivGridMapSub = nh_public.subscribe<nav_msgs::OccupancyGrid>("map", 1, &FootstepNavigation::mapCallback, this);
		ivGoalPoseSub = nh_public.subscribe<geometry_msgs::PoseStamped>("goal", 1, &FootstepNavigation::goalPoseCallback, this);
        // subscribe to robot pose to get latest time
        ivRobotPoseSub = nh_public.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 5, &FootstepNavigation::robotPoseCallback, this);

        // read parameters from config file:
        nh_private.param("rfoot_frame_id", ivFootIDRight, ivFootIDRight);
        nh_private.param("lfoot_frame_id", ivFootIDLeft, ivFootIDLeft);

        nh_private.param("accuracy/footstep/x", ivFootstepAccuracyX, 0.01);
        nh_private.param("accuracy/footstep/y", ivFootstepAccuracyY, 0.01);
        nh_private.param("accuracy/footstep/theta", ivFootstepAccuracyTheta, 0.15);

        double max_footstep_x, max_footstep_y, max_footstep_theta;
        double max_inv_footstep_x, max_inv_footstep_y, max_inv_footstep_theta;
        nh_private.param("foot/separation", ivFootSeparation, 0.095);
        nh_private.param("foot/max/step/x", max_footstep_x, 0.04);
        nh_private.param("foot/max/step/y", max_footstep_y, 0.04);
        nh_private.param("foot/max/step/theta", max_footstep_theta, 0.349);
        nh_private.param("foot/max/inverse/step/x", max_inv_footstep_x, 0.04);
        nh_private.param("foot/max/inverse/step/y", max_inv_footstep_y, 0.01);
        nh_private.param("foot/max/inverse/step/theta", max_inv_footstep_theta, 0.05);
        ivMaxFootstepX = cont_2_disc(max_footstep_x, ivCellSize);
        ivMaxFootstepY = cont_2_disc(max_footstep_y, ivCellSize);
        ivMaxFootstepTheta = angle_cont_2_disc(max_footstep_theta, ivNumAngleBins);
        ivMaxInvFootstepX = cont_2_disc(max_inv_footstep_x, ivCellSize);
        ivMaxInvFootstepY = cont_2_disc(max_inv_footstep_y, ivCellSize);
        ivMaxInvFootstepTheta = angle_cont_2_disc(max_inv_footstep_theta, ivNumAngleBins);

        nh_private.param("accuracy/cell_size", ivCellSize, 0.01);
        nh_private.param("accuracy/num_angle_bins", ivNumAngleBins, 64);
    }


	FootstepNavigation::~FootstepNavigation()
    {}


    void
    FootstepNavigation::executeFootsteps()
    {
    	if (ivPlanner.getPathSize() == 0)
    		return;

    	humanoid_nav_msgs::StepTarget step;
    	humanoid_nav_msgs::StepTargetService step_service;

    	tf::Transform support_foot;
    	std::string support_foot_id;

    	// calculate and perform relative footsteps until goal is reached
    	bool reached = false;
    	state_iter_t current = ivPlanner.getPathBegin();
    	while (current != ivPlanner.getPathEnd())
    	{
    		if (current->leg == RIGHT)
    		{
    			support_foot_id = ivFootIDRight;
    			step.leg = humanoid_nav_msgs::StepTarget::left;
    		}
    		else // support_foot = LLEG
    		{
    			support_foot_id = ivFootIDLeft;
    			step.leg = humanoid_nav_msgs::StepTarget::right;
    		}

    		{
    			boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    			// get real placement of the support foot
    			getFootTransform(support_foot_id, ivMapFrameID, ivLastRobotTime,
    			                 support_foot);
    		}
    		// get next state to calculate relative footstep
    		current++;

    		// calculate relative step
    		reached = getFootstep(support_foot, *current, step);
    		// if step cannot be performed initialize replanning..
    		if (!reached)
    		{
    			ROS_INFO("Footstep cannot be performed: new path planning "
    					 "necessary");
    			// TODO: remove later
    			return;

                if (ivPlanner.replan())
                {
                    // if replanning was successful start new execution thread
                    boost::thread footstepExecutionThread(
                            &FootstepNavigation::executeFootsteps, this);
                }
                // leave this thread
                return;
    		}
    		// ..otherwise perform step
    		else
    		{
    			step_service.request.step = step;
    			ivFootstepService.call(step_service);
    		}
    	}

    	// perform last step (0,0,0) so the feet are again parallel
    	if (current->leg == RIGHT)
    		step.leg = humanoid_nav_msgs::StepTarget::left;
    	else // supportLeg == LLEG
    		step.leg = humanoid_nav_msgs::StepTarget::right;
    	step.pose.x = 0;
    	step.pose.y = 0;
    	step_service.request.step = step;
    	ivFootstepService.call(step_service);

    	ivExecutingFootsteps = false;
    }


    bool
    FootstepNavigation::getFootstep(
            const tf::Transform& support_foot,
            const State& foot_placement,
            humanoid_nav_msgs::StepTarget& footstep)
    {
    	// calculate the necessary footstep to reach the foot placement
    	tf::Transform footstep_transform;
    	Leg support_leg;
    	if (footstep.leg == humanoid_nav_msgs::StepTarget::right)
    		support_leg = LEFT;
    	else // footstep.leg == LEFT
    		support_leg = RIGHT;
    	double x, y, theta;
    	get_footstep(support_leg, ivFootSeparation,
                     support_foot.getOrigin().x(),
                     support_foot.getOrigin().y(),
                     tf::getYaw(support_foot.getRotation()),
    				 foot_placement.x,
    				 foot_placement.y,
    				 foot_placement.theta,
    				 x, y, theta);

    	int disc_x = cont_2_disc(x, ivCellSize);
        int disc_y = cont_2_disc(y, ivCellSize);
        int disc_theta = angle_cont_2_disc(theta, ivNumAngleBins);
        bool reached = reachable(disc_x, disc_y, disc_theta,
                                 ivMaxFootstepX, ivMaxFootstepY,
                                 ivMaxFootstepTheta,
                                 ivMaxInvFootstepX, ivMaxInvFootstepY,
                                 ivMaxInvFootstepTheta,
                                 support_leg);
        if (reached)
        {
            footstep.pose.x = x;
            footstep.pose.y = y;
            footstep.pose.theta = theta;

            return true;
        }
        else
        {
            footstep.pose.x = 0;
            footstep.pose.y = 0;
            footstep.pose.theta = 0;

            return false;
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
    FootstepNavigation::goalPoseCallback(
            const geometry_msgs::PoseStampedConstPtr& goal_pose)
    {
        if (ivExecutingFootsteps)
        {
            ROS_INFO("currently walking down a footstep path; no planning "
                     "possible");
            return;
        }

    	if (setGoal(goal_pose))
    	{
    		tf::Transform foot_left, foot_right;
    		{
				boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
				// get real placement of the feet
				getFootTransform(ivFootIDLeft, ivMapFrameID, ivLastRobotTime,
                                 foot_left);
				getFootTransform(ivFootIDRight, ivMapFrameID, ivLastRobotTime,
                                 foot_right);
    		}
    		State left, right;
    		left.x = foot_left.getOrigin().x();
    		left.y = foot_left.getOrigin().y();
    		left.leg = LEFT;
    		left.theta = tf::getYaw(foot_left.getRotation());

    		right.x = foot_right.getOrigin().x();
    		right.y = foot_right.getOrigin().y();
    		right.leg = RIGHT;
    		right.theta = tf::getYaw(foot_right.getRotation());

    		if (ivPlanner.setStart(right, left))
    		{
    	        // calculate plan
    	        if (ivPlanner.plan())
    	        {
    	            // execution thread
    	            ivExecutingFootsteps = true;
    	            boost::thread footstepExecutionThread(
    	                    &FootstepNavigation::executeFootsteps, this);
    	        }
    		}
    		else
    		{
    			ROS_ERROR("start pose not accessible; odometry and map "
    					  "information do not match!");
    		}
    	}
    }


    bool
    FootstepNavigation::setGoal(
            const geometry_msgs::PoseStampedConstPtr& goal_pose)
    {
        return setGoal(goal_pose->pose.position.x,
                       goal_pose->pose.position.y,
                       tf::getYaw(goal_pose->pose.orientation));
    }


    void
    FootstepNavigation::mapCallback(
            const nav_msgs::OccupancyGridConstPtr& occupancy_map)
    {
        boost::shared_ptr<GridMap2D> gridMap(new GridMap2D(occupancy_map));
        ivMapFrameID = gridMap->getFrameID();
        ivPlanner.setMap(gridMap);
    }


    bool
    FootstepNavigation::setGoal(float x, float y, float theta)
    {
    	return ivPlanner.setGoal(x,y,theta);
    }


    void
    FootstepNavigation::getFootTransform(const std::string& foot_id,
    		const std::string& world_frame_id,
    		const ros::Time& time,
    		tf::Transform& foot)
    {
    	tf::StampedTransform stamped_foot_transform;
    	// TODO: try/catch if tf not available?
    	ivTransformListener.waitForTransform(
    	        world_frame_id, foot_id, time, ros::Duration(0.1));
    	ivTransformListener.lookupTransform(
    	        world_frame_id, foot_id, time, stamped_foot_transform);

    	foot.setOrigin(stamped_foot_transform.getOrigin());
    	foot.setRotation(stamped_foot_transform.getRotation());
    }
}

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
          ivRFootID("/RFoot_link"), ivLFootID("/LFoot_link"),
          ivMapFrameID("map")
    {
        // private NodeHandle for parameters and private messages (debug / info)
        ros::NodeHandle nh_private("~");
        ros::NodeHandle nh_public;

        // service
        ivFootstepService = nh_public.serviceClient<humanoid_nav_msgs::StepTargetService>("cmd_step_srv");

		ivGoalPoseSub = nh_public.subscribe<geometry_msgs::PoseStamped>("goal", 1, &FootstepNavigation::goalPoseCallback, this);
        // subscribe to robot pose to get latest time
        ivRobotPoseSub = nh_public.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 5, &FootstepNavigation::robotPoseCallback, this);

        // read parameters from config file:
        nh_private.param("rfoot_frame_id", ivRFootID, ivRFootID);
        nh_private.param("lfoot_frame_id", ivLFootID, ivLFootID);

        nh_private.param("accuracy/footstep/x", ivFootstepAccuracyX, 0.01);
        nh_private.param("accuracy/footstep/y", ivFootstepAccuracyY, 0.01);
        nh_private.param("accuracy/footstep/theta", ivFootstepAccuracyTheta, 0.15);

        nh_private.param("foot/separation", ivFootSeparation, 0.095);
        nh_private.param("foot/max/step/x", ivMaxFootstepX, 0.04);
        nh_private.param("foot/max/step/y", ivMaxFootstepY, 0.04);
        nh_private.param("foot/max/step/theta", ivMaxFootstepTheta, 0.349);
        nh_private.param("foot/max/inverse/step/x", ivMaxInvFootstepX, 0.04);
        nh_private.param("foot/max/inverse/step/y", ivMaxInvFootstepY, 0.01);
        nh_private.param("foot/max/inverse/step/theta", ivMaxInvFootstepTheta, 0.05);

        nh_private.param("accuracy/cell_size", ivCellSize, 0.01);
        nh_private.param("accuracy/num_angle_bins", ivNumAngleBins, 64);
    }


	FootstepNavigation::~FootstepNavigation()
    {}


    void
    FootstepNavigation::executeFootsteps()
    {
    	humanoid_nav_msgs::StepTarget step;
    	humanoid_nav_msgs::StepTargetService footstep_service;

    	state_iter_t current = ivPlanner.getPathBegin();
    	std::string support_foot_link;
    	if (current->leg == RIGHT)
    	{
    		support_foot_link = ivLFootID;
    		step.leg = humanoid_nav_msgs::StepTarget::right;
    	}
    	else    // supportLeg == LLEG
    	{
    		support_foot_link = ivRFootID;
    		step.leg = humanoid_nav_msgs::StepTarget::left;
    	}

    	tf::Transform support_foot;
    	{
    		boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    		getFootTransform(support_foot_link, ivMapFrameID, ivLastRobotTime, support_foot);
    	}

    	tf::Transform footPlacement(
    	        tf::createQuaternionFromYaw(current->theta),
    			tf::Point(current->x, current->y, 0));

    	// perform a greedy footstep adjustment to place the robot's feet on the
    	// first foot placement calculated by the planner
    	bool reached = getGreedyFootstep(support_foot, footPlacement, step);
    	footstep_service.request.step = step;
    	ivFootstepService.call(footstep_service);

    	// calculate and perform relative footsteps until goal is reached
    	while (current != ivPlanner.getPathEnd())
    	{
    		if (current->leg == RIGHT)
    		{
    			support_foot_link = ivRFootID;
    			step.leg = humanoid_nav_msgs::StepTarget::left;
    		}
    		else // supportLeg = LLEG
    		{
    			support_foot_link = ivLFootID;
    			step.leg = humanoid_nav_msgs::StepTarget::right;
    		}

    		{
    			boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    			// get real placement of the support foot
    			getFootTransform(support_foot_link, ivMapFrameID, ivLastRobotTime, support_foot);
    		}
    		// get next state:
    		current++;

    		footPlacement = tf::Transform(tf::createQuaternionFromYaw(current->theta),
    		    			tf::Point(current->x, current->y, 0));

    		// NOTE: if method returns false then perform a replanning
    		reached = getGreedyFootstep(support_foot, footPlacement, step);
    		if (!reached)
    		{
                if (ivPlanner.replan())
                {
                    // if replanning was successful start new execution thread
                    boost::thread footstepExecutionThread(
                            &FootstepNavigation::executeFootsteps, this);
                }
                // leave this thread
                return;
    		}
    		else
    		{
    			footstep_service.request.step = step;
    			ivFootstepService.call(footstep_service);
    		}
    	}

    	if (current->leg == RIGHT)
    		step.leg = humanoid_nav_msgs::StepTarget::left;
    	else // supportLeg == LLEG
    		step.leg = humanoid_nav_msgs::StepTarget::right;
    	step.pose.x = 0;
    	step.pose.y = 0;
    	footstep_service.request.step = step;
    	ivFootstepService.call(footstep_service);

    	ivExecutingFootsteps = false;
    }


    bool
    FootstepNavigation::getGreedyFootstep(
            const tf::Transform& support_foot,
            const tf::Transform& foot_placement,
            humanoid_nav_msgs::StepTarget& footstep)
    {
    	// calculate the necessary footstep to reach the foot placement
    	tf::Transform footstep_transform;
    	Leg support_leg;
    	if (footstep.leg == humanoid_nav_msgs::StepTarget::right)
    		support_leg = RIGHT;
    	else // leg == LEFT
    		support_leg = LEFT;
    	double x, y, theta;
    	get_footstep(support_leg, ivFootSeparation,
                     support_foot.getOrigin().x(),
                     support_foot.getOrigin().y(),
                     tf::getYaw(support_foot.getRotation()),
    				 foot_placement.getOrigin().x(),
    				 foot_placement.getOrigin().y(),
    				 tf::getYaw(foot_placement.getRotation()),
    				 x, y, theta);

    	int diff_x = cont_2_disc(x, ivCellSize);
        int diff_y = cont_2_disc(y, ivCellSize);
        int diff_theta = angle_cont_2_disc(theta, ivNumAngleBins);
        return reachable(diff_x, diff_y, diff_theta,
                         ivMaxFootstepX, ivMaxFootstepY, ivMaxFootstepTheta,
                         ivMaxInvFootstepX, ivMaxInvFootstepY,
                         ivMaxInvFootstepTheta,
                         support_leg);
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
				getFootTransform(ivLFootID, ivMapFrameID, ivLastRobotTime,
                                 foot_left);
				getFootTransform(ivRFootID, ivMapFrameID, ivLastRobotTime,
                                 foot_right);
    		}
    		State left, right;
    		left.x = foot_left.getOrigin().x();
    		left.y = foot_left.getOrigin().y();
    		left.leg = LEFT;
    		left.theta = tf::getYaw(foot_left.getRotation());

    		right.x = foot_right.getOrigin().x();
    		right.y = foot_right.getOrigin().y();
    		right.leg = LEFT;
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
    	}
    }


    bool
    FootstepNavigation::setGoal(
            const geometry_msgs::PoseStampedConstPtr& goal_pose)
    {
    	ROS_INFO("check2");

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

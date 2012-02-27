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
        ivFootstepService = nh_public.serviceClient<
				humanoid_nav_msgs::StepTargetService>("cmd_step_srv");

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
        nh_private.param("rfoot_frame_id", ivFootIDRight, ivFootIDRight);
        nh_private.param("lfoot_frame_id", ivFootIDLeft, ivFootIDLeft);

        nh_private.param("accuracy/footstep/x", ivAccuracyX, 0.005);
        nh_private.param("accuracy/footstep/y", ivAccuracyY, 0.005);
        nh_private.param("accuracy/footstep/theta", ivAccuracyTheta, 0.05);

        nh_private.param("foot/separation", ivFootSeparation, 0.095);
        nh_private.param("foot/max/step/x", ivContMaxFootstepX, 0.04);
        nh_private.param("foot/max/step/y", ivContMaxFootstepY, 0.04);
        nh_private.param("foot/max/step/theta", ivContMaxFootstepTheta, 0.349);
        nh_private.param("foot/max/inverse/step/x", ivContMaxInvFootstepX,
		                 0.04);
        nh_private.param("foot/max/inverse/step/y", ivContMaxInvFootstepY,
		                 0.01);
        nh_private.param("foot/max/inverse/step/theta",
                         ivContMaxInvFootstepTheta, 0.05);

        nh_private.param("accuracy/cell_size", ivCellSize, 0.01);
        nh_private.param("accuracy/num_angle_bins", ivNumAngleBins, 64);

        // discretization
        ivMaxFootstepX = discretize(ivContMaxFootstepX, ivCellSize);
        ivMaxFootstepY = discretize(ivContMaxFootstepY, ivCellSize);
        ivMaxFootstepTheta = angle_state_2_cell(ivContMaxFootstepTheta,
                                                ivNumAngleBins);
        ivMaxInvFootstepX = discretize(ivContMaxInvFootstepX, ivCellSize);
        ivMaxInvFootstepY = discretize(ivContMaxInvFootstepY, ivCellSize);
        ivMaxInvFootstepTheta = angle_state_2_cell(ivContMaxInvFootstepTheta,
                                                   ivNumAngleBins);

//        // TODO: remove after debug
//        ROS_INFO("--- discrete accuracies ---");
//        ROS_INFO("max footstep (%i, %i, %i)", ivMaxFootstepX, ivMaxFootstepY,
//        		ivMaxFootstepTheta);
//        ROS_INFO("max inv footstep (%i, %i, %i)", ivMaxInvFootstepX,
//        		ivMaxInvFootstepY, ivMaxInvFootstepTheta);
//        ROS_INFO("--- continuous accuracies ---");
//        ROS_INFO("max footstep (%f, %f, %f)",
//        		ivContMaxFootstepX, ivContMaxFootstepY, ivContMaxFootstepTheta);
//        ROS_INFO("max inv footstep (%f, %f, %f)\n", ivContMaxInvFootstepX,
//        		ivContMaxInvFootstepY, ivContMaxInvFootstepTheta);
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
    	bool performable = false;
    	state_iter_t next_foot_placement = ivPlanner.getPathBegin();
    	while (next_foot_placement != ivPlanner.getPathEnd())
    	{
    		if (next_foot_placement->leg == RIGHT)
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

//    		// TODO: remove later
//    		current_state = *next_foot_placement;
//
//    		// TODO: remove later
//    		ROS_INFO("--- difference between calculated state and actual "
//    				 "state ---");
//    		ROS_INFO("calculated state (%f, %f, %f", next_foot_placement->x,
//    				 next_foot_placement->y, next_foot_placement->theta);
//    		ROS_INFO("actual state (%f, %f, %f)",
//    				 support_foot.getOrigin().x(),
//    				 support_foot.getOrigin().y(),
//    				 tf::getYaw(support_foot.getRotation()));

			// get next state to calculate relative footstep
    		next_foot_placement++;

//    		// TODO: remove later
//    		ROS_INFO("--- calculated footstep between current state and next "
//    				 "state ---");
//    		ROS_INFO("current state (%f, %f, %f, %s)",
//    		         support_foot.getOrigin().x(),
//    		         support_foot.getOrigin().y(),
//    		         tf::getYaw(support_foot.getRotation()),
//    		         support_foot_id.c_str());
//    		ROS_INFO("next (planned) state (%f, %f, %f, %i)",
//    		         next_foot_placement->x, next_foot_placement->y,
//    		         next_foot_placement->theta, next_foot_placement->leg);

    		// calculate relative step
    		performable = getFootstep(support_foot, *next_foot_placement, step);

//    		// TODO: remove later
//    		ROS_INFO("--- calculated relative footstep and performability ---");
//    		ROS_INFO("step (%f, %f, %f)", step.pose.x, step.pose.y,
//    		         step.pose.theta);
//    		ROS_INFO("performable? %i", performablee);
//
//    		// TODO: remove later
//    		ROS_INFO("--- calculate relative foostep from both planning states "
//    				 "---");
//    		double step_x, step_y, step_theta;
//    		Leg leg;
//    		if (step.leg == RIGHT) leg = LEFT;
//    		else leg = RIGHT;
//    		get_footstep(
//    				leg, ivFootSeparation,
//    				current_state.x, current_state.y, current_state.theta,
//    				next_foot_placement->x, next_foot_placement->y,
//    				next_foot_placement->theta,
//    				step_x, step_y, step_theta);
//    		int disc_step_x = discretize(step_x, ivCellSize);
//			int disc_step_y = discretize(step_y, ivCellSize);
//			int disc_step_theta = angle_state_2_cell(
//					step_theta, ivNumAngleBins);
//    		performable = performable(
//    				disc_step_x, disc_step_y, disc_step_theta,
//    				ivMaxFootstepX, ivMaxFootstepY, ivMaxFootstepTheta,
//    				ivMaxInvFootstepX, ivMaxInvFootstepY,
//    				ivMaxInvFootstepTheta,
//    				ivNumAngleBins);
//    		ROS_INFO("step (%f, %f, %f)", step_x, step_y, step_theta);
//    		ROS_INFO("disc step (%i, %i, %i)",
//    				disc_step_x, disc_step_y, disc_step_theta);
//    		ROS_INFO("performable? %i", performablee);
//    		exit(0);

    		// if step cannot be performed initialize replanning..
    		if (!performable)
    		{
    			ROS_INFO("Footstep cannot be performed: new path planning "
    					 "necessary");

    			// TODO: uncomment this later
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
    			step_service.request.step = step;
    			ivFootstepService.call(step_service);
    		}
    	}
    	// perform last step (0,0,0) so the feet are again parallel
    	if (next_foot_placement->leg == RIGHT)
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
    	Leg support_foot_leg;
    	if (footstep.leg == humanoid_nav_msgs::StepTarget::right)
    		support_foot_leg = LEFT;
    	else // footstep.leg == LEFT
    		support_foot_leg = RIGHT;

    	double footstep_x, footstep_y, footstep_theta;
    	get_footstep(support_foot_leg, ivFootSeparation,
                     support_foot.getOrigin().x(),
                     support_foot.getOrigin().y(),
                     tf::getYaw(support_foot.getRotation()),
    				 foot_placement.x, foot_placement.y, foot_placement.theta,
    				 footstep_x, footstep_y, footstep_theta);

        bool performable = performable_cont(
                footstep_x, footstep_y, footstep_theta,
                ivContMaxFootstepX, ivContMaxFootstepY, ivContMaxFootstepTheta,
                ivContMaxInvFootstepX, ivContMaxInvFootstepY,
                ivContMaxInvFootstepTheta,
                ivAccuracyX, ivAccuracyY, ivAccuracyTheta);

        if (performable)
        {
        	// clip footsteps if necessary (i.e. clip accuracies)
        	if (footstep_x > ivContMaxFootstepX)
        		footstep_x = ivContMaxFootstepX;
        	else if (footstep_x < ivContMaxInvFootstepX)
        		footstep_x = ivContMaxInvFootstepX;
        	if (footstep_y > ivContMaxFootstepY)
        		footstep_y = ivContMaxFootstepY;
        	else if (footstep_y < ivContMaxInvFootstepY)
        		footstep_y = ivContMaxInvFootstepY;
        	if (footstep_theta > ivContMaxFootstepTheta)
        		footstep_theta = ivContMaxFootstepTheta;
        	else if (footstep_theta < ivContMaxInvFootstepTheta)
        		footstep_theta = ivContMaxInvFootstepTheta;

            footstep.pose.x = footstep_x;
            footstep.pose.y = footstep_y;
            footstep.pose.theta = footstep_theta;

            return true;
        }
        else
        	return false;
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
    		if (updateStart())
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
    			ROS_ERROR("start pose not accessible: check your odometry");
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


    bool
    FootstepNavigation::setGoal(float x, float y, float theta)
    {
    	return ivPlanner.setGoal(x,y,theta);
    }


    bool
    FootstepNavigation::updateStart()
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

        return ivPlanner.setStart(right, left);
    }


    void
    FootstepNavigation::mapCallback(
            const nav_msgs::OccupancyGridConstPtr& occupancy_map)
    {
        boost::shared_ptr<GridMap2D> gridMap(new GridMap2D(occupancy_map));
        ivMapFrameID = gridMap->getFrameID();
        ivPlanner.setMap(gridMap);
    }


    void
    FootstepNavigation::getFootTransform(
            const std::string& foot_id, const std::string& world_frame_id,
    		const ros::Time& time, tf::Transform& foot)
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

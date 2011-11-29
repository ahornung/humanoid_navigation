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
        ivFootstepService = nh_public.serviceClient<humanoid_nav_msgs::StepTargetService>("cmd_step_srv");


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

        // subscribe to robot pose to get latest time
        ivRobotPoseSub = nh_public.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 5, &FootstepNavigation::robotPoseCallback, this);

    }


	FootstepNavigation::~FootstepNavigation()
    {}

    bool
    FootstepNavigation::run()
    {

    	// TODO: Initial plan
    	humanoid_nav_msgs::StepTarget step;
    	humanoid_nav_msgs::StepTargetService footstepService;


    	state_iter_t currentState = ivPlanner.getPathBegin();

    	tf::Transform supportFoot;
    	std::string supportFootLink;
    	tf::Transform footPlacement(tf::createQuaternionFromYaw(currentState->theta),
    			tf::Point(currentState->x, currentState->y, 0));

    	int firstStepLeg;
    	int secondStepLeg;
    	if (currentState->leg == RIGHT)
    	{
    		supportFootLink = ivLFootID;
    		step.leg = humanoid_nav_msgs::StepTarget::right;
    		firstStepLeg = humanoid_nav_msgs::StepTarget::right;
    		secondStepLeg = humanoid_nav_msgs::StepTarget::left;
    	}
    	else    // supportLeg == LLEG
    	{
    		supportFootLink = ivRFootID;
    		step.leg = humanoid_nav_msgs::StepTarget::left;
    		firstStepLeg = humanoid_nav_msgs::StepTarget::left;
    		secondStepLeg = humanoid_nav_msgs::StepTarget::right;
    	}

    	{
    		boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    		getFootTransform(supportFootLink, ivMapFrameID, ivLastRobotTime, supportFoot);
    	}
    	// perform a greedy footstep adjustment to place the robot's feet on the
    	// first foot placement calculated by the planner
    	bool reached = getGreedyFootstep(supportFoot, footPlacement, step);
    	footstepService.request.step = step;
    	ivFootstepService.call(footstepService);

    	// this loop tries to reach the first "real" footstep in the plan...
    	// still necessary?
    	while (!reached)
    	{
    		step.leg = secondStepLeg;
    		step.pose.x = 0;
    		step.pose.y = 0;
    		footstepService.request.step = step;
    		ivFootstepService.call(footstepService);
    		{
    			// TODO: check mutex locking
    			boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    			getFootTransform(supportFootLink, ivMapFrameID, ivLastRobotTime, supportFoot);
    		}
    		reached = getGreedyFootstep(supportFoot, footPlacement, step);
    		step.leg = firstStepLeg;
    		footstepService.request.step = step;
    		ivFootstepService.call(footstepService);
    	}

    	// calculate and perform relative footsteps until goal is reached
    	while (currentState != ivPlanner.getPathEnd())
    	{
    		if (currentState->leg == RIGHT)
    		{
    			supportFootLink = ivRFootID;
    			step.leg = humanoid_nav_msgs::StepTarget::left;
    		}
    		else // supportLeg = LLEG
    		{
    			supportFootLink = ivLFootID;
    			step.leg = humanoid_nav_msgs::StepTarget::right;
    		}

    		{
    			boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    			// get real placement of the support foot
    			getFootTransform(supportFootLink, ivMapFrameID, ivLastRobotTime, supportFoot);
    		}
    		// get next state:
    		currentState++;

    		footPlacement = tf::Transform(tf::createQuaternionFromYaw(currentState->theta),
    		    			tf::Point(currentState->x, currentState->y, 0));


    		// NOTE: if method returns false then perform a replanning
    		reached = getGreedyFootstep(supportFoot, footPlacement, step);
    		if (!reached)
    		{
    			// TODO: start replanning
    			ROS_ERROR("Replanning on robot not implemented yet!");
    		}
    		else
    		{
    			footstepService.request.step = step;
    			ivFootstepService.call(footstepService);
    		}

    	}

    	if (currentState->leg == RIGHT)
    		step.leg = humanoid_nav_msgs::StepTarget::left;
    	else // supportLeg == LLEG
    		step.leg = humanoid_nav_msgs::StepTarget::right;
    	step.pose.x = 0;
    	step.pose.y = 0;
    	footstepService.request.step = step;
    	ivFootstepService.call(footstepService);

    	ivExecutingFootsteps = false;

    	return true;
    }

    bool
    FootstepNavigation::getGreedyFootstep(const tf::Transform& supportFoot,
    									  const tf::Transform& footPlacement,
    									  humanoid_nav_msgs::StepTarget& footstep)
    {

    	bool xInRange = false;
    	bool yInRange = false;
    	bool thetaInRange = false;

    	// calculate the necessary footstep to reach the foot placement
    	tf::Transform footstepTransform;
    	Leg supportLeg;
    	if (footstep.leg == humanoid_nav_msgs::StepTarget::right)
    		supportLeg = RIGHT;
    	else // leg == LEFT
    		supportLeg = LEFT;
    	double x, y, theta;
    	get_footstep(supportLeg, ivFootSeparation,
    				 supportFoot.getOrigin().x(), supportFoot.getOrigin().y(), tf::getYaw(supportFoot.getRotation()),
    				 footPlacement.getOrigin().x(), footPlacement.getOrigin().y(), tf::getYaw(footPlacement.getRotation()),
    				 x, y, theta);

    	// TODO: cleanup below (see "reachable" in environment?)
    	if (x <= ivMaxFootstepX + ivFootstepAccuracyX &&
    			x >= -ivMaxFootstepX - ivFootstepAccuracyX)
    	{
    		xInRange = true;
    	}
    	else
    	{
    		if (x > ivMaxFootstepX )
    			footstep.pose.x = ivMaxFootstepX;
    		else if (x < -ivMaxFootstepX)
    			footstep.pose.x = -ivMaxFootstepX;
    		else
    			footstep.pose.x = x;
    	}
    	if (footstep.leg == humanoid_nav_msgs::StepTarget::right)
    	{
    		if (y >= -ivMaxFootstepY - ivFootstepAccuracyY &&
    				y <= ivMaxInvFootstepY + ivFootstepAccuracyY)
    		{
    			yInRange = true;
    		}
    		else
    		{
    			if (y < -ivMaxFootstepY)
    				footstep.pose.y = -ivMaxFootstepY;
    			else if (y > ivMaxInvFootstepY)
    				footstep.pose.y = ivMaxInvFootstepY;
    			else
    				footstep.pose.y = y;
    		}
    		if (theta >= -ivMaxFootstepTheta - ivFootstepAccuracyTheta &&
    				theta <= ivMaxInvFootstepTheta + ivFootstepAccuracyTheta)
    		{
    			thetaInRange = true;
    		}
    		else
    		{
    			if (theta < -ivMaxFootstepTheta)
    				footstep.pose.theta = -ivMaxFootstepTheta;
    			else if (theta > ivMaxInvFootstepTheta)
    				footstep.pose.theta = ivMaxInvFootstepTheta;
    			else
    				footstep.pose.theta = theta;
    		}
    	}
    	else // leg =left
    			{
    		if (y <= ivMaxFootstepY + ivFootstepAccuracyY &&
    				y >= -ivMaxInvFootstepY - ivFootstepAccuracyY)
    		{
    			yInRange = true;
    		}
    		else
    		{
    			if (y > ivMaxFootstepY)
    				footstep.pose.y = ivMaxFootstepY;
    			else if (y < -ivMaxInvFootstepTheta)
    				footstep.pose.y = -ivMaxInvFootstepTheta;
    			else
    				footstep.pose.y = y;
    		}
    		if (theta <= ivMaxFootstepTheta + ivFootstepAccuracyTheta &&
    				theta >= -ivMaxInvFootstepTheta - ivFootstepAccuracyTheta)
    		{
    			thetaInRange = true;
    		}
    		else
    		{
    			if (theta > ivMaxFootstepTheta)
    				footstep.pose.theta = ivMaxFootstepTheta;
    			else if (theta < -ivMaxInvFootstepTheta)
    				footstep.pose.theta = -ivMaxInvFootstepTheta;
    			else
    				footstep.pose.theta = theta;
    		}
    			}

    	return xInRange && yInRange && thetaInRange;

    }

    void
    FootstepNavigation::robotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& robotPose)
    {
    	boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
    	ivLastRobotTime = robotPose->header.stamp;
    }

    void
    FootstepNavigation::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose)
    {
    	// TODO: check if already executing footsteps, request stop

    	if (setGoal(goal_pose))
    	{
    		tf::Transform leftFoot, rightFoot;
    		{
				boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
				// get real placement of the feet
				getFootTransform(ivLFootID, ivMapFrameID, ivLastRobotTime, leftFoot);
				getFootTransform(ivRFootID, ivMapFrameID, ivLastRobotTime, rightFoot);
    		}
    		State left, right;
    		left.x = leftFoot.getOrigin().x();
    		left.y = leftFoot.getOrigin().y();
    		left.leg = LEFT;
    		left.theta = tf::getYaw(leftFoot.getRotation());

    		right.x = rightFoot.getOrigin().x();
    		right.y = rightFoot.getOrigin().y();
    		right.leg = LEFT;
    		right.theta = tf::getYaw(rightFoot.getRotation());

    		if (ivPlanner.setStart(right, left)){
				// execution thread
				ivExecutingFootsteps = true;
				boost::thread footstepExecutionThread(&FootstepNavigation::run, this);
    		}
    	}
    }

    bool
    FootstepNavigation::setGoal(const geometry_msgs::PoseStampedConstPtr& goal_pose)
    {

        return setGoal(goal_pose->pose.position.x,
                       goal_pose->pose.position.y,
                       tf::getYaw(goal_pose->pose.orientation));
    }

    void
    FootstepNavigation::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map)
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
    FootstepNavigation::getFootTransform(const std::string& footID,
    		const std::string& worldFrameID,
    		const ros::Time& time,
    		tf::Transform& foot)
    {

    	tf::StampedTransform stampedFootTransform;
    	// TODO: try/catch if tf not available?
    	ivTransformListener.waitForTransform(worldFrameID, footID, time, ros::Duration(0.1));
    	ivTransformListener.lookupTransform(worldFrameID, footID, time, stampedFootTransform);

    	foot.setOrigin(stampedFootTransform.getOrigin());
    	foot.setRotation(stampedFootTransform.getRotation());

    }



}

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

namespace footstep_planner{

	FootstepPlanner::FootstepPlanner()
		: ivDstarSetUp(false),
		  ivStartPoseSet(false),
		  ivGoalPoseSet(false),
		  ivRobotPoseSet(false),
		  ivRFootID("/RFoot_link"),
		  ivLFootID("/LFoot_link"),
		  ivMarkerNamespace("")
	{

		// private NodeHandle for parameters and private messages (debug / info)
		ros::NodeHandle privateNh("~");

		ivExecutingFootsteps = false;
		ivLastMarkerMsgSize  = 0;

		// ..publishers
		ivExpandedStatesVisPub = privateNh.advertise<sensor_msgs::PointCloud>("expanded_states", 1);
		ivFootstepPathVisPub = privateNh.advertise<visualization_msgs::MarkerArray>("footsteps_array", 1);
		ivStartPoseVisPub = privateNh.advertise<geometry_msgs::PoseStamped>("start", 1);
		ivPathVisPub = privateNh.advertise<nav_msgs::Path>("path", 1);
		ivHeuristicPathVisPub = privateNh.advertise<nav_msgs::Path>("heuristic_path", 1);
		// ..and services
		ivFootstepService = ivNh.serviceClient<humanoid_nav_msgs::StepTargetService>("cmd_step_srv");


		int    mode;
		int    heuristic;
		double subgoalDistance;
		double stepCosts;
		int    roundingThreshold;
		int    plannerMaxSteps;

		// read parameters from config file:
		// - planner settings
		privateNh.param("planning_mode", mode, 0);
		privateNh.param("heuristic", heuristic, 0);
		privateNh.param("subgoal_distance", subgoalDistance, 3.0);
		privateNh.param("planner_max_steps", plannerMaxSteps, 180000);
		privateNh.param("step_costs", stepCosts, 0.05);
		privateNh.param("accuracy/rounding_threshold", roundingThreshold, 2);
		privateNh.param("accuracy/collision_check", ivCollisionCheckAccuracy, 2);
		privateNh.param("accuracy/footstep/x", ivFootstepAccuracyX, 0.01);
		privateNh.param("accuracy/footstep/y", ivFootstepAccuracyY, 0.01);
		privateNh.param("accuracy/footstep/theta", ivFootstepAccuracyTheta, 0.15);
		privateNh.param("rfoot_frame_id", ivRFootID, ivRFootID);
		privateNh.param("lfoot_frame_id", ivLFootID, ivLFootID);

		// - footstep settings
		privateNh.param("foot/size/x", ivFootsizeX, 0.16);
		privateNh.param("foot/size/y", ivFootsizeY, 0.06);
		privateNh.param("foot/size/z", ivFootsizeZ, 0.015);
		privateNh.param("foot/separation", ivFootSeparation, 0.095);
		privateNh.param("foot/origin_shift/x", ivFootOriginShiftX, 0.02);
		privateNh.param("foot/origin_shift/y", ivFootOriginShiftY, 0.0);
		privateNh.param("foot/max/step/x", ivFootMaxStepX, 0.04);
		privateNh.param("foot/max/step/y", ivFootMaxStepY, 0.04);
		privateNh.param("foot/max/step/theta", ivFootMaxStepTheta, 0.349);
		privateNh.param("foot/max/inverse/step/x", ivFootMaxInverseStepX, 0.04);
		privateNh.param("foot/max/inverse/step/y", ivFootMaxInverseStepY, 0.01);
		privateNh.param("foot/max/inverse/step/theta", ivFootMaxInverseStepTheta, 0.05);
		// - footstep discretisation
		XmlRpc::XmlRpcValue xDiscretizationList;
		XmlRpc::XmlRpcValue yDiscretizationList;
		XmlRpc::XmlRpcValue thetaDiscretizationList;
		privateNh.getParam("footsteps/x", xDiscretizationList);
		privateNh.getParam("footsteps/y", yDiscretizationList);
		privateNh.getParam("footsteps/theta", thetaDiscretizationList);
		if (xDiscretizationList.getType() != XmlRpc::XmlRpcValue::TypeArray)
			ROS_ERROR("Error reading footsteps/x from config file.");
		if (yDiscretizationList.getType() != XmlRpc::XmlRpcValue::TypeArray)
			ROS_ERROR("Error reading footsteps/y from config file.");
		if (thetaDiscretizationList.getType() != XmlRpc::XmlRpcValue::TypeArray)
			ROS_ERROR("Error reading footsteps/theta from config file.");
		// check if received footstep discretization is valid
		int size, sizeY, sizeT;
		try
		{
			size = xDiscretizationList.size();
			sizeY = yDiscretizationList.size();
			sizeT = thetaDiscretizationList.size();

			if (size != sizeY || size != sizeT)
			{
				ROS_ERROR("Footstep parameterization has different sizes for x/y/theta, exiting.");
				exit(0);
			}
		} catch (const XmlRpc::XmlRpcException e)
		{
			ROS_ERROR("No footstep parameterization available, exiting.");
			exit(0);
		}

		// create footstep set
		std::vector<Footstep> footstepSet;
		float maxX = 0, maxY = 0;
		for(int i=0; i < size; i++)
		{
			float x = (double)xDiscretizationList[i];
			float y = (double)yDiscretizationList[i];
			float theta = (double)thetaDiscretizationList[i];

			Footstep f(x, y, theta, NOLEG);
			footstepSet.push_back(f);

			if (x > maxX)
				maxX = x;
			if (y > maxY)
				maxY = y;
		}
		float maxStepWidth = sqrt(maxX*maxX + maxY*maxY);

		// initialize the heuristic
		boost::shared_ptr<Heuristic> h;
		float footWidth;
		switch (heuristic)
		{
		case Heuristic::EUCLIDEAN:
			// euclidean distance
			h.reset(new EuclideanHeuristic(Heuristic::EUCLIDEAN, roundingThreshold));
			ROS_INFO("FootstepPlanner heuristic: euclidean distance");
			break;
		case Heuristic::EUCLIDEAN_STEPCOST:
			// euclidean distance + step costs estimation
			h.reset(new EuclStepCostHeuristic(Heuristic::EUCLIDEAN_STEPCOST,
			                                  roundingThreshold,
			                                  stepCosts,
			                                  maxStepWidth));
			ROS_INFO("FootstepPlanner heuristic: euclidean distance w. step costs");
			break;
		case Heuristic::ASTAR_PATH:
			// euclidean distance + step costs estimation based on precalculated subgoals
			footWidth = ivFootsizeY; // this width should be the smaller of the foot's dimensions
			h.reset(new AstarHeuristic(Heuristic::ASTAR_PATH,
                    stepCosts,
                    maxStepWidth,
                    footWidth,
                    subgoalDistance,
                    roundingThreshold));

			// keep a local ptr for visualization
			ivAstarHeuristic = boost::dynamic_pointer_cast<AstarHeuristic>(h);

			ROS_INFO("FootstepPlanner heuristic: 2D path euclidean distance w. step costs");
			break;
		default:
			ROS_ERROR("No heuristic available, exiting.");
			exit(0);
		}

		// initialize planning algorithm
		ivDstarPtr.reset(new Dstar(footstepSet,
								   ivFootSeparation,
								   ivFootOriginShiftX,
								   ivFootOriginShiftY,
								   ivFootsizeX,
								   ivFootsizeY,
								   ivFootMaxStepX,
								   ivFootMaxStepY,
								   ivFootMaxStepTheta,
								   ivFootMaxInverseStepY,
								   ivFootMaxInverseStepTheta,
								   maxStepWidth,
								   stepCosts,
								   ivCollisionCheckAccuracy,
								   roundingThreshold,
								   plannerMaxSteps,
								   h));

		ivMode = (PlanningMode)mode;

	}


	FootstepPlanner::~FootstepPlanner()
	{}


	void
	FootstepPlanner::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goalPose)
	{

		bool success = setGoal(goalPose);
		if (success)
		{
			// if the planning algorithm has been set up the new goal state can be
			// set directly (the planner's previously collected information will be
			// reseted)
			if (ivDstarSetUp)
				// NOTE: goal is set to the right foot of the desired robot's pose
				ivDstarPtr->setGoal(ivGoalFootRight);

			// make sure either a manually set pose or a robot pose has been set
			if (ivStartPoseSet || ivRobotPoseSet)
				navigate();
		}

	}


	void
	FootstepPlanner::startPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& startPose)
	{

		if (ivMode != MERE_PLANNING)
		{
			ROS_INFO("Start pose is ignored since planning mode is set to robot navigation.");
			return;
		}

		bool success = setStart(startPose->pose.pose.position.x,
								startPose->pose.pose.position.y,
								tf::getYaw(startPose->pose.pose.orientation));
		if (success)
		{
			// make sure a goal pose has been set before starting the navigation
			if (ivGoalPoseSet)
				navigate();
		}

	}


	void
	FootstepPlanner::robotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& robotPose)
	{

		if (ivMode != ROBOT_NAVIGATION)
		{
			ROS_INFO("Robot pose is ignored since planning mode is set to mere planning.");
			return;
		}

		boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
		ivRobotHeader = robotPose->header;
		ivRobotPoseSet = true;

	}


	void
	FootstepPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancyMap)
	{

		boost::shared_ptr<GridMap2D> gridMap(new GridMap2D(occupancyMap));
		setMap(gridMap);

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
		if (occupied(goal))
		{
			ROS_ERROR("Goal pose at (%f %f %f) not accessible.", x, y, theta);
			return false;
		}

		getFootPositions(goal, ivGoalFootLeft, ivGoalFootRight);
		if (occupied(ivGoalFootLeft)  || occupied(ivGoalFootRight))
		{
			ROS_ERROR("Goal pose not accessible.");
			ivGoalFootLeft = State();
			ivGoalFootRight = State();
			return false;
		}

		ivGoalPoseSet = true;
		ROS_INFO("Goal pose set to (%f %f %f)", x, y, theta);

		return true;

	}


	bool
	FootstepPlanner::setGoal(const geometry_msgs::PoseStampedConstPtr& goalPose)
	{

		return setGoal(goalPose->pose.position.x,
					   goalPose->pose.position.y,
					   tf::getYaw(goalPose->pose.orientation));

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
		if (occupied(start))
		{
			ROS_ERROR("Start pose (%f %f %f) not accessible.", x, y, theta);
			return false;
		}

		getFootPositions(start, ivStartFootLeft, ivStartFootRight);
		if (occupied(ivStartFootLeft) || occupied(ivStartFootRight))
		{
			ROS_ERROR("Starting foot positions not accessible.");
			ivStartFootLeft = State();
			ivStartFootRight = State();
			return false;
		}

		ivStartPoseSet = true;
		ROS_INFO("Start pose set to (%f %f %f)", x, y, theta);

		// publish visualization:
		geometry_msgs::PoseStamped startPose;
		startPose.pose.position.x = x;
		startPose.pose.position.y = y;
		startPose.pose.position.z = 0.025;
		startPose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
		startPose.header.frame_id = ivMapPtr->getFrameID();
		startPose.header.stamp = ros::Time::now();
		ivStartPoseVisPub.publish(startPose);

		return true;

	}



	bool
	FootstepPlanner::setStart(const geometry_msgs::PoseStampedConstPtr& startPose)
	{

		return setStart(startPose->pose.position.x,
						startPose->pose.position.y,
						tf::getYaw(startPose->pose.orientation));

	}


	void
	FootstepPlanner::setMap(boost::shared_ptr<GridMap2D> gridMap)
	{

		ivMapPtr.reset();
		ivMapPtr = gridMap;
		ivDstarPtr->updateDistanceMap(ivMapPtr);

	}


	bool
	FootstepPlanner::plan()
	{

		if (ivMode != MERE_PLANNING)
		{
			ROS_INFO("Planning mode is set to robot navigation.");
			return false;
		}

		// all checks should have been performed before
		assert(ivGoalPoseSet);
		assert(ivStartPoseSet);
		assert(ivMapPtr);

		ivDstarPtr->reset();
		// deactivate set up flag to reinitialize the planning algorithm with the
		// new start and goal poses
		ivDstarSetUp = false;
		// start the planning
		bool success = dstarPlanning();

		// return success
		if (success)
		{
			// broadcast calculated path
			broadcastExpandedNodesVis();
			broadcastFootstepPathVis();
			broadcastPathVis();

			return true;
		}
		else
		{
			broadcastExpandedNodesVis();
			return false;
		}

	}

    bool
    FootstepPlanner::planService(PlanFootsteps::Request &req, PlanFootsteps::Response &resp)
    {
    	bool result = plan(req.start.x, req.start.y, req.start.theta,
    						req.goal.x, req.goal.y, req.goal.theta);

    	resp.costs = ivDstarPtr->getPathCosts();
    	resp.footsteps.reserve(ivDstarPtr->getNumFootsteps());


    	for (Dstar::stateIterator it = ivDstarPtr->getPathBegin(); it != ivDstarPtr->getPathEnd(); ++it){
    		// TODO: differentiate between left and right here? => add foot to footstep srv!
    		geometry_msgs::Pose2D foot;
    		foot.x = it->getX();
    		foot.y = it->getY();
    		foot.theta = it->getTheta();

    		resp.footsteps.push_back(foot);
    	}

    	resp.result = result;

    	return result;
    }


	bool
	FootstepPlanner::plan(float startX, float startY, float startTheta,
						  float goalX, float goalY, float goalTheta)
	{

		if (ivMode != MERE_PLANNING)
		{
			ROS_INFO("Planning mode is set to robot navigation.");
			return false;
		}

		if (!(setStart(startX, startY, startTheta) && setGoal(goalX, goalY, goalTheta)))
			return false;

		return plan();

	}


	bool
	FootstepPlanner::plan(const geometry_msgs::PoseStampedConstPtr& startPose,
						  const geometry_msgs::PoseStampedConstPtr& goalPose)
	{

		return plan(startPose->pose.position.x, startPose->pose.position.y, tf::getYaw(startPose->pose.orientation),
					goalPose->pose.position.x, goalPose->pose.position.y, tf::getYaw(goalPose->pose.orientation));

	}


	void
	FootstepPlanner::navigate()
	{

		// if the robot is currently executing footsteps then start no new planning
		if (ivMode == ROBOT_NAVIGATION && ivExecutingFootsteps)
			return;

		// all checks should have been performed before
		assert(!(ivMode == MERE_PLANNING && !ivStartPoseSet));
		assert(!(ivMode == ROBOT_NAVIGATION && !ivRobotPoseSet));
		assert(ivGoalPoseSet);
		assert(ivMapPtr);

		// start the planning task
		if (!dstarPlanning())
		{
			broadcastExpandedNodesVis();
			return;
		}

		broadcastExpandedNodesVis();
		broadcastFootstepPathVis();
		broadcastPathVis();

		// if a robot (or a simulator) is used start executing the calculated footsteps
		if (ivMode == ROBOT_NAVIGATION)
		{
			ivExecutingFootsteps = true;
			boost::thread footstepExecutionThread(&FootstepPlanner::executeFootsteps, this);
		}

	}


	bool
	FootstepPlanner::dstarPlanning()
	{

		// check if D* lite has been set up (this has to be done only once)
		if (!ivDstarSetUp)
		{
			if (ivMode == ROBOT_NAVIGATION)
			{
				tf::Transform leftFoot;
				tf::Transform rightFoot;
				getFootTransform(ivLFootID, ivMapPtr->getFrameID(), ivRobotHeader.stamp, leftFoot);
				getFootTransform(ivRFootID, ivMapPtr->getFrameID(), ivRobotHeader.stamp, rightFoot);
				ivStartFootLeft = State(leftFoot.getOrigin().x(),
										leftFoot.getOrigin().y(),
										tf::getYaw(leftFoot.getRotation()),
										LEFT);
				ivStartFootRight = State(rightFoot.getOrigin().x(),
										 rightFoot.getOrigin().y(),
										 tf::getYaw(rightFoot.getRotation()),
										 RIGHT);
				// NOTE: goal is set to the right foot of the desired robot's pose
				ivDstarPtr->setUp(ivStartFootLeft, ivStartFootRight, ivGoalFootRight);
			}
			else if (ivMode == MERE_PLANNING)
			{
				// NOTE: goal is set to the right foot of the desired robot's pose
				ivDstarPtr->setUp(ivStartFootLeft, ivStartFootRight, ivGoalFootRight);
			}
			ivDstarSetUp = true;
		}
		// D* lite has been set up previously so the start pose of the planner has
		// to be updated via its update method
		else
		{
			// if a robot (or a simulator) is used update it's position
			if (ivMode == ROBOT_NAVIGATION)
			{
				tf::Transform leftFoot;
				tf::Transform rightFoot;
				getFootTransform(ivLFootID, ivMapPtr->getFrameID(), ivRobotHeader.stamp, leftFoot);
				getFootTransform(ivRFootID, ivMapPtr->getFrameID(), ivRobotHeader.stamp, rightFoot);
				ivStartFootLeft = State(leftFoot.getOrigin().x(),
										leftFoot.getOrigin().y(),
										tf::getYaw(leftFoot.getRotation()),
										LEFT);
				ivStartFootRight = State(rightFoot.getOrigin().x(),
										 rightFoot.getOrigin().y(),
										 tf::getYaw(rightFoot.getRotation()),
										 RIGHT);
				ivDstarPtr->updateStart(ivStartFootLeft, ivStartFootRight);
			}
			// update the manually set (e.g. via rviz) start pose
			else if (ivMode == MERE_PLANNING)
			{
				ivDstarPtr->updateStart(ivStartFootLeft, ivStartFootRight);
			}
		}

		broadcastHeuristicPathVis();

		// start the planning task
		bool success = ivDstarPtr->replan();

		// return the result
		return success;

	}


	void
	FootstepPlanner::executeFootsteps()
	{

		humanoid_nav_msgs::StepTarget step;
		humanoid_nav_msgs::StepTargetService footstepService;

		State current = *(ivDstarPtr->getPathBegin());

		tf::Transform supportFoot;
		std::string supportFootLink;
		tf::Transform footPlacement(tf::createQuaternionFromYaw(current.getTheta()),
									tf::Point(current.getX(), current.getY(), 0));

		int firstStepLeg;
		int secondStepLeg;
		if (current.getLeg() == RIGHT)
		{
			supportFootLink = ivLFootID;
			step.leg = humanoid_nav_msgs::StepTarget::right;
			firstStepLeg = humanoid_nav_msgs::StepTarget::right;
			secondStepLeg = humanoid_nav_msgs::StepTarget::left;
		}
		else	// supportLeg == LLEG
		{
			supportFootLink = ivRFootID;
			step.leg = humanoid_nav_msgs::StepTarget::left;
			firstStepLeg = humanoid_nav_msgs::StepTarget::left;
			secondStepLeg = humanoid_nav_msgs::StepTarget::right;
		}

		{
			boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
			getFootTransform(supportFootLink,
							 ivMapPtr->getFrameID(),
							 ivRobotHeader.stamp,
							 supportFoot);
		}
		// perform a greedy footstep adjustment to place the robot's feed on the
		// first foot placement calculated by the planner
		bool reached = getGreedyFootstep(supportFoot, footPlacement, step);
		footstepService.request.step = step;
		ivFootstepService.call(footstepService);

		while (!reached)
		{
			step.leg = secondStepLeg;
			step.pose.x = 0;
			step.pose.y = 0;
			footstepService.request.step = step;
			ivFootstepService.call(footstepService);
			{
				boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
				getFootTransform(supportFootLink,
								 ivMapPtr->getFrameID(),
								 ivRobotHeader.stamp,
								 supportFoot);
			}
			reached = getGreedyFootstep(supportFoot, footPlacement, step);
			step.leg = firstStepLeg;
			footstepService.request.step = step;
			ivFootstepService.call(footstepService);
		}

		// calculate and perform relative footsteps until goal is reached
		State next;
		while (!ivDstarPtr->isCloseToGoal(current))
		{
			if (current.getLeg() == RIGHT)
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
				getFootTransform(supportFootLink,
								 ivMapPtr->getFrameID(),
								 ivRobotHeader.stamp,
								 supportFoot);
			}
			// get next foot placement
			ivDstarPtr->getMinSucc(current, &next);
			footPlacement = tf::Transform(tf::createQuaternionFromYaw(next.getTheta()),
										  tf::Point(next.getX(), next.getY(), 0));
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
			current = next;
		}

		if (current.getLeg() == RIGHT)
			step.leg = humanoid_nav_msgs::StepTarget::left;
		else // supportLeg == LLEG
			step.leg = humanoid_nav_msgs::StepTarget::right;
		step.pose.x = 0;
		step.pose.y = 0;
		footstepService.request.step = step;
		ivFootstepService.call(footstepService);

		ivExecutingFootsteps = false;

	}


	bool
	FootstepPlanner::getGreedyFootstep(const tf::Transform& supportFoot,
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
		getFootstep(supportLeg, ivFootSeparation, supportFoot, footPlacement, &footstepTransform);
		float x = footstepTransform.getOrigin().x();
		float y = footstepTransform.getOrigin().y();
		float theta = tf::getYaw(footstepTransform.getRotation());

		if (x <= ivFootMaxStepX + ivFootstepAccuracyX &&
			x >= -ivFootMaxStepX - ivFootstepAccuracyX)
		{
			xInRange = true;
		}
		else
		{
			if (x > ivFootMaxStepX )
				footstep.pose.x = ivFootMaxStepX;
			else if (x < -ivFootMaxStepX)
				footstep.pose.x = -ivFootMaxStepX;
			else
				footstep.pose.x = x;
		}
		if (footstep.leg == humanoid_nav_msgs::StepTarget::right)
			{
			if (y >= -ivFootMaxStepY - ivFootstepAccuracyY &&
				y <= ivFootMaxInverseStepY + ivFootstepAccuracyY)
			{
				yInRange = true;
			}
			else
			{
				if (y < -ivFootMaxStepY)
					footstep.pose.y = -ivFootMaxStepY;
				else if (y > ivFootMaxInverseStepY)
					footstep.pose.y = ivFootMaxInverseStepY;
				else
					footstep.pose.y = y;
			}
			if (theta >= -ivFootMaxStepTheta - ivFootstepAccuracyTheta &&
				theta <= ivFootMaxInverseStepTheta + ivFootstepAccuracyTheta)
			{
				thetaInRange = true;
			}
			else
			{
				if (theta < -ivFootMaxStepTheta)
					footstep.pose.theta = -ivFootMaxStepTheta;
				else if (theta > ivFootMaxInverseStepTheta)
					footstep.pose.theta = ivFootMaxInverseStepTheta;
				else
					footstep.pose.theta = theta;
			}
		}
		else // leg =left
		{
			if (y <= ivFootMaxStepY + ivFootstepAccuracyY &&
				y >= -ivFootMaxInverseStepY - ivFootstepAccuracyY)
			{
				yInRange = true;
			}
			else
			{
				if (y > ivFootMaxStepY)
					footstep.pose.y = ivFootMaxStepY;
				else if (y < -ivFootMaxInverseStepY)
					footstep.pose.y = -ivFootMaxInverseStepY;
				else
					footstep.pose.y = y;
			}
			if (theta <= ivFootMaxStepTheta + ivFootstepAccuracyTheta &&
				theta >= -ivFootMaxInverseStepTheta - ivFootstepAccuracyTheta)
			{
				thetaInRange = true;
			}
			else
			{
				if (theta > ivFootMaxStepTheta)
					footstep.pose.theta = ivFootMaxStepTheta;
				else if (theta < -ivFootMaxInverseStepTheta)
					footstep.pose.theta = -ivFootMaxInverseStepTheta;
				else
					footstep.pose.theta = theta;
			}
		}

		return xInRange && yInRange && thetaInRange;

	}


	void
	FootstepPlanner::getFootPositions(const State& robot, State& footLeft, State& footRight)
	{

		float theta = robot.getTheta();
		float xShift = -sin(theta)*ivFootSeparation/2;
		float yShift =  cos(theta)*ivFootSeparation/2;

		footLeft.setX(robot.getX() + xShift);
		footLeft.setY(robot.getY() + yShift);
		footLeft.setTheta(theta);
		footLeft.setLeg(LEFT);

		footRight.setX(robot.getX() - xShift);
		footRight.setY(robot.getY() - yShift);
		footRight.setTheta(theta);
		footRight.setLeg(RIGHT);

	}


	void
	FootstepPlanner::getFootTransform(const std::string& from,
									  const std::string& to,
									  const ros::Time& time,
									  tf::Transform& foot)
	{

		tf::StampedTransform stampedFootTransform;
		ivTransformListener.waitForTransform(to, from, time, ros::Duration(0.1));
		ivTransformListener.lookupTransform(to, from, time, stampedFootTransform);

		foot.setOrigin(stampedFootTransform.getOrigin());
		foot.setRotation(stampedFootTransform.getRotation());

	}


	bool
	FootstepPlanner::occupied(const State& u)
	{

		float cosTheta = cos(u.getTheta());
		float sinTheta = sin(u.getTheta());
		float xShift = cosTheta*ivFootOriginShiftX - sinTheta*ivFootOriginShiftY;
		float yShift;

		if (u.getLeg() == LEFT)
			yShift = sinTheta*ivFootOriginShiftX + cosTheta*ivFootOriginShiftY;
		else // leg == RLEG
			yShift = sinTheta*ivFootOriginShiftX - cosTheta*ivFootOriginShiftY;

		return collisionCheck(u.getX() + xShift,
							  u.getY() + yShift,
							  u.getTheta(),
							  ivFootsizeX,
							  ivFootsizeY,
							  ivCollisionCheckAccuracy,
							  *ivMapPtr);

	}


	void
	FootstepPlanner::broadcastFootstepPathVis()
	{

		// checks should have been performed before
		assert(!(ivMode == MERE_PLANNING && !ivStartPoseSet));
		assert(!(ivMode == ROBOT_NAVIGATION && !ivRobotPoseSet));

		visualization_msgs::Marker marker;
		visualization_msgs::MarkerArray markerMsg;
		std::vector<visualization_msgs::Marker> markerVector;

		int markerCounter = 0;

		if (ivMode == MERE_PLANNING)
		{
			marker.header.stamp = ros::Time::now();
			marker.header.frame_id = ivMapPtr->getFrameID();
		}
		else if (ivMode == ROBOT_NAVIGATION)
			marker.header = ivRobotHeader;

		// add the left start foot to the publish vector
		footstepToMarker(ivStartFootLeft, marker);
		marker.id = markerCounter++;
		markerVector.push_back(marker);

		// add the right start foot to the publish vector
		footstepToMarker(ivStartFootRight, marker);
		marker.id = markerCounter++;
		markerVector.push_back(marker);

		// add the footsteps of the path to the publish vector
		Dstar::stateIterator pathIter = ivDstarPtr->getPathBegin();
		for(; pathIter != ivDstarPtr->getPathEnd(); pathIter++)
		{
			footstepToMarker(*pathIter, marker);
			marker.id = markerCounter++;
			markerVector.push_back(marker);
		}
		if (markerCounter < ivLastMarkerMsgSize)
		{
			for(int j = markerCounter; j < ivLastMarkerMsgSize; j++)
			{
				marker.ns = ivMarkerNamespace;
				marker.id = j;
				marker.action = visualization_msgs::Marker::DELETE;

				markerVector.push_back(marker);
			}
		}
		markerMsg.markers = markerVector;
		ivLastMarkerMsgSize = markerVector.size();

		ivFootstepPathVisPub.publish(markerMsg);

	}

	void
	FootstepPlanner::clearFootstepPathVis(unsigned numFootsteps){
		visualization_msgs::Marker marker;
		visualization_msgs::MarkerArray markerMsg;

		if (ivMode == MERE_PLANNING)
		{
			marker.header.stamp = ros::Time::now();
			marker.header.frame_id = ivMapPtr->getFrameID();
		}
		else if (ivMode == ROBOT_NAVIGATION)
			marker.header = ivRobotHeader;

		if (numFootsteps < 1){
			numFootsteps = ivLastMarkerMsgSize;
		}

		for (unsigned i = 0; i < numFootsteps; i++){
			marker.ns = ivMarkerNamespace;
			marker.id = i;
			marker.action = visualization_msgs::Marker::DELETE;

			markerMsg.markers.push_back(marker);

		}

		ivFootstepPathVisPub.publish(markerMsg);
	}


	void
	FootstepPlanner::broadcastHeuristicPathVis()
	{

		if (!ivAstarHeuristic)
			return;

		nav_msgs::Path pathMsg;
		geometry_msgs::PoseStamped state;

		if (ivMode == MERE_PLANNING)
		{
			state.header.stamp = ros::Time::now();
			state.header.frame_id = ivMapPtr->getFrameID();
		}
		else if (ivMode == ROBOT_NAVIGATION)
			state.header = ivRobotHeader;

		AstarHeuristic::subgoal_iter pathIter = ivAstarHeuristic->getPathBegin();
		for(; pathIter != ivAstarHeuristic->getPathEnd(); pathIter++)
		{
			state.pose.position.x = pathIter->first.first;
			state.pose.position.y = pathIter->first.second;
			pathMsg.poses.push_back(state);
		}
		pathMsg.header = state.header;
		ivHeuristicPathVisPub.publish(pathMsg);

	}


	void
	FootstepPlanner::broadcastPathVis()
	{

		// checks should have been performed before
		assert(!(ivMode == MERE_PLANNING && !ivStartPoseSet));
		assert(!(ivMode == ROBOT_NAVIGATION && !ivRobotPoseSet));

		nav_msgs::Path pathMsg;
		geometry_msgs::PoseStamped state;

		if (ivMode == MERE_PLANNING)
		{
			state.header.stamp = ros::Time::now();
			state.header.frame_id = ivMapPtr->getFrameID();
		}
		else if (ivMode == ROBOT_NAVIGATION)
			state.header = ivRobotHeader;

		Dstar::stateIterator pathIter = ivDstarPtr->getPathBegin();

		if (pathIter->getLeg() == RIGHT)
		{
			state.pose.position.x = ivStartFootRight.getX();
			state.pose.position.y = ivStartFootRight.getY();
			pathMsg.poses.push_back(state);
			state.pose.position.x = ivStartFootLeft.getX();
			state.pose.position.y = ivStartFootLeft.getY();
			pathMsg.poses.push_back(state);
		}
		else // leg == LEFT
		{
			state.pose.position.x = ivStartFootLeft.getX();
			state.pose.position.y = ivStartFootLeft.getY();
			pathMsg.poses.push_back(state);
			state.pose.position.x = ivStartFootRight.getX();
			state.pose.position.y = ivStartFootRight.getY();
			pathMsg.poses.push_back(state);
		}

		for(; pathIter != ivDstarPtr->getPathEnd(); pathIter++)
		{
			state.pose.position.x = pathIter->getX();
			state.pose.position.y = pathIter->getY();
			pathMsg.poses.push_back(state);
		}
		pathMsg.header = state.header;
		ivPathVisPub.publish(pathMsg);

	}


	void
	FootstepPlanner::broadcastExpandedNodesVis()
	{

		// checks should have been performed before
		assert(!(ivMode == MERE_PLANNING && !ivStartPoseSet));
		assert(!(ivMode == ROBOT_NAVIGATION && !ivRobotPoseSet));

		sensor_msgs::PointCloud cloudMsg;
		geometry_msgs::Point32 point;
		std::vector<geometry_msgs::Point32> points;

		Dstar::stateIterator expandedStatesIter = ivDstarPtr->getExpandedBegin();
		for(; expandedStatesIter != ivDstarPtr->getExpandedEnd(); expandedStatesIter++)
		{
			point.x = expandedStatesIter->getX();
			point.y = expandedStatesIter->getY();
			point.z = 0.01;
			points.push_back(point);
		}
		if (ivMode == MERE_PLANNING){
			cloudMsg.header.stamp = ros::Time::now();
			cloudMsg.header.frame_id = ivMapPtr->getFrameID();
		}
		else if (ivMode == ROBOT_NAVIGATION)
			cloudMsg.header = ivRobotHeader;
		cloudMsg.points = points;

		ivExpandedStatesVisPub.publish(cloudMsg);

	}


	void
	FootstepPlanner::footstepToMarker(const State& footstep, visualization_msgs::Marker& marker)
	{

		// checks should have been performed before
		assert(!(ivMode == MERE_PLANNING && !ivStartPoseSet));
		assert(!(ivMode == ROBOT_NAVIGATION && !ivRobotPoseSet));

		if (ivMode == MERE_PLANNING)
		{
			marker.header.stamp = ros::Time::now();
			marker.header.frame_id = ivMapPtr->getFrameID();
		}
		else if (ivMode == ROBOT_NAVIGATION)
			marker.header = ivRobotHeader;
		marker.ns = ivMarkerNamespace;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;

		float cosTheta = cos(footstep.getTheta());
		float sinTheta = sin(footstep.getTheta());
		float xShift = cosTheta*ivFootOriginShiftX - sinTheta*ivFootOriginShiftY;
		float yShift;
		if (footstep.getLeg() == LEFT)
			yShift = sinTheta*ivFootOriginShiftX + cosTheta*ivFootOriginShiftY;
		else // leg == RLEG
			yShift = sinTheta*ivFootOriginShiftX - cosTheta*ivFootOriginShiftY;
		marker.pose.position.x = footstep.getX() + xShift;
		marker.pose.position.y = footstep.getY() + yShift;
		tf::quaternionTFToMsg(tf::createQuaternionFromYaw(footstep.getTheta()), marker.pose.orientation);

		marker.scale.x = ivFootsizeX; // - 0.01;
		marker.scale.y = ivFootsizeY; // - 0.01;
		marker.scale.z = ivFootsizeZ;

		// TODO: make color configurable?
		if (footstep.getLeg() == RIGHT)
		{
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
		}
		else // leg == LEFT
		{
			marker.color.r = 1.0f;
			marker.color.g = 0.0f;
		}
		marker.color.b = 0.0;
		marker.color.a = 0.25;

		marker.lifetime = ros::Duration();

	}

} // end of namespace

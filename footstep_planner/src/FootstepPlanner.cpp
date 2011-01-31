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

#include "footstep_planner/FootstepPlanner.h"


/*
 * FootstepPlanner::FootstepPlanner()
 * --------------------------
 * Constructor.
 */
FootstepPlanner::FootstepPlanner()
	: ivDstarSetUp(false),
	  ivStartPoseSet(false),
	  ivGoalPoseSet(false),
	  ivRobotPoseSet(false),
	  ivRFootID("/RFoot_link"),
	  ivLFootID("/LFoot_link")
{

	ivExecutingFootsteps = false;
	ivLastMarkerMsgSize  = 0;

	// ..publishers
	// TODO: move to private namespace
	ivExpandedStatesVisPub = ivNh.advertise<sensor_msgs::PointCloud>("footstep_planning/expanded_states", 1);
	ivFootstepPathVisPub = ivNh.advertise<visualization_msgs::MarkerArray>("footstep_planning/footsteps_array", 1);
	ivStartPoseVisPub = ivNh.advertise<geometry_msgs::PoseStamped>("footstep_planning/start", 1);
	ivPathVisPub = ivNh.advertise<nav_msgs::Path>("footstep_planning/path", 1);
	// ..and services
	ivFootstepService = ivNh.serviceClient<humanoid_nav_msgs::StepTargetService>("cmd_step_srv");

	// private NodeHandle for parameters:
	ros::NodeHandle privateNh("~");

	int    mode;
	int    heuristic;
	double stepCosts;
	int    roundingThreshold;
	int    plannerMaxSteps;

	// read parameters from config file:
	// - planner settings
	privateNh.param("planning_mode", mode, 0);
	privateNh.param("heuristic", heuristic, 0);
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
	switch (heuristic)
	{
	case 0: // euclidean distance
		h.reset(new EuclideanHeuristic(roundingThreshold));
		break;
	case 1: // euclidean distance + step costs estimation
		h.reset(new EuclStepCostHeuristic(roundingThreshold, stepCosts, maxStepWidth));
		break;
	case 2: // euclidean distance + step costs estimation based on precalculated subgoals
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


/*
 * FootstepPlanner::~FootstepPlanner()
 * --------------------------
 * Destructor.
 */
FootstepPlanner::~FootstepPlanner()
{}


/*
 * void FootstepPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& goal)
 * --------------------------
 * Callback function for updating the goal pose.
 */
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


/*
 * void FootstepPlanner::robotFakePoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& initialpose)
 * --------------------------
 * Callback function for updating the robot's pose if the pose is set manually
 * (e.g. via rviz).
 */
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


/*
 * void FootstepPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& gridMap)
 * --------------------------
 * Callback function for updating the robots pose (received for example via 2D laser orientation).
 */
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


/*
 * void FootstepPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& gridMap)
 * --------------------------
 * Callback function for updating the map.
 */
void
FootstepPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancyMap)
{

	boost::shared_ptr<GridMap2D> gridMap(new GridMap2D(occupancyMap));
	setMap(gridMap);


//	geometry_msgs::PoseStampedPtr start(new geometry_msgs::PoseStamped);
//	start->pose.position.x = 0.2;
//	start->pose.position.y = 0.9;
//	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0), start->pose.orientation);
//	start->header = occupancyMap->header;
//	geometry_msgs::PoseStampedPtr goal(new geometry_msgs::PoseStamped);
//	goal->pose.position.x = 0.655;
//	goal->pose.position.y = 0.9;
//	goal->header = occupancyMap->header;
//	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0), goal->pose.orientation);
//	plan(start, goal);
////	ROS_INFO("---------------------------------------------------------------");
////	start->pose.position.x = 0.2;
////	start->pose.position.y = 0.9;
////	setStart(start);
//	exit(0);

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
	if (!ivGoalPoseSet and !ivStartPoseSet)
	{
		ROS_ERROR("Either no start pose or no robot pose has been received yet.");
		return false;
	}
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
		return false;
	}

}


bool
FootstepPlanner::plan(float startX, float startY, float startTheta,
                      float goalX, float goalY, float goalTheta)
{

	// TODO: Also add a service (wrapper) for this functionality

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


/*
 * void FootstepPlanner::navigate()
 * --------------------------
 * Navigates the robot from his basic pose to the goal pose.
 */
void
FootstepPlanner::navigate()
{

	// if the robot is currently executing footsteps then start no new planning
	if (ivMode == ROBOT_NAVIGATION && ivExecutingFootsteps)
		return;

	// check if all necessary information for the planning task has been received
	if (ivMode == MERE_PLANNING && !ivStartPoseSet)
	{
		ROS_ERROR("No start pose received yet.");
		return;
	}
	else if (ivMode == ROBOT_NAVIGATION && !ivRobotPoseSet)
	{
		ROS_ERROR("No robot pose received yet.");
		return;
	}
	if (!ivGoalPoseSet)
	{
		ROS_ERROR("No goal pose set yet.");
		return;
	}
	assert(ivMapPtr);

	// start the planning task
	if (!dstarPlanning())
	{
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
			getFootTransform(leftFoot, ivLFootID, ivMapPtr->getFrameID(), ivRobotHeader.stamp);
			getFootTransform(rightFoot, ivRFootID, ivMapPtr->getFrameID(), ivRobotHeader.stamp);
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
			getFootTransform(leftFoot, ivLFootID, ivMapPtr->getFrameID(), ivRobotHeader.stamp);
			getFootTransform(rightFoot, ivRFootID, ivMapPtr->getFrameID(), ivRobotHeader.stamp);
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

	// start the planning task
	bool success = ivDstarPtr->replan();

	// return the result
	return success;

}


/*
 * void FootstepPlanner::executeFootsteps()
 * --------------------------
 * Perform the calculated footsteps and initialize a replanning if the
 * environment changes or deviation to the calculated path due to unclean
 * performed footsteps is too big.
 */
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
		step.leg = (int)RIGHT;
		firstStepLeg = (int)RIGHT;
		secondStepLeg = (int)LEFT;
	}
	else	// supportLeg == LLEG
	{
		supportFootLink = ivRFootID;
		step.leg = (int)LEFT;
		firstStepLeg = (int)LEFT;
		secondStepLeg = (int)RIGHT;
	}

	{
		boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
		getFootTransform(supportFoot,
						 supportFootLink,
						 ivMapPtr->getFrameID(),
						 ivRobotHeader.stamp);
	}
	// perform a greedy footstep adjustment to place the robot's feed on the
	// first footstep placement calculated by the planner
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
			getFootTransform(supportFoot,
							 supportFootLink,
							 ivMapPtr->getFrameID(),
							 ivRobotHeader.stamp);
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
			step.leg = (int)LEFT;
		}
		else // supportLeg = LLEG
		{
			supportFootLink = ivLFootID;
			step.leg = (int)RIGHT;
		}

		{
			boost::mutex::scoped_lock lock(ivRobotPoseUpdateMutex);
			// get real placement of the support foot
			getFootTransform(supportFoot,
							 supportFootLink,
							 ivMapPtr->getFrameID(),
							 ivRobotHeader.stamp);
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
		}
		else
		{
			footstepService.request.step = step;
			ivFootstepService.call(footstepService);
		}
		current = next;
	}

	if (current.getLeg() == RIGHT)
		step.leg = (int)LEFT;
	else // supportLeg == LLEG
		step.leg = (int)RIGHT;
	step.pose.x = 0;
	step.pose.y = 0;
	footstepService.request.step = step;
	ivFootstepService.call(footstepService);

	ivExecutingFootsteps = false;

}


///*
// * bool FootstepPlanner::getGreadyFootstep(tf::Transform& supportFoot,
// *                                         tf::Transform& footPlacement,
// *                                         humanoid_nav_msgs::StepTarget& footstep)
// * --------------------------
// * Check if 'footPlacement' can be reached from 'supportFoot' by any footstep
// * (i.e. not only the discretized footsteps).
// */
//bool
//FootstepPlanner::getGreedyFootstep(tf::Transform& supportFoot,
//                                   tf::Transform& footPlacement,
//                                   humanoid_nav_msgs::StepTarget& footstep)
//{
//
//	bool xInRange = false;
//	bool yInRange = false;
//	bool thetaInRange = false;
//
//	tf::Transform current;
//	tf::Vector3 translation;
//
//	tf::Quaternion rotation = footPlacement.getRotation().inverse() * supportFoot.getRotation();
//	if (footstep.leg == humanoid_nav_msgs::StepTarget::right)
//		translation = tf::Vector3(0, ivFootSeparation/2, 0);
//	else // leg == rightLeg
//		translation = tf::Vector3(0, -ivFootSeparation/2, 0);
//	current = footPlacement * tf::Transform(rotation, translation);
//	current *= tf::Transform(tf::createQuaternionFromYaw(0.0), translation);
//	current = supportFoot.inverse() * current;
//
//	float diffX = current.getOrigin().x();
//	float diffY = current.getOrigin().y();
//	float diffAngle = -tf::getYaw(rotation);
//
//	if (diffX <= ivFootMaxStepX + ivFootstepAccuracyX &&
//		diffX >= -ivFootMaxStepX - ivFootstepAccuracyX)
//	{
//		xInRange = true;
//	}
//	else
//	{
//		if (diffX > ivFootMaxStepX )
//			footstep.pose.x = ivFootMaxStepX;
//		else if (diffX < -ivFootMaxStepX)
//			footstep.pose.x = -ivFootMaxStepX;
//		else
//			footstep.pose.x = diffX;
//	}
//	if (footstep.leg == humanoid_nav_msgs::StepTarget::right)
//		{
//		if (diffY >= -ivFootMaxStepY - ivFootstepAccuracyY &&
//			diffY <= ivFootMaxInverseStepY + ivFootstepAccuracyY)
//		{
//			yInRange = true;
//		}
//		else
//		{
//			if (diffY < -ivFootMaxStepY)
//				footstep.pose.y = -ivFootMaxStepY;
//			else if (diffY > ivFootMaxInverseStepY)
//				footstep.pose.y = ivFootMaxInverseStepY;
//			else
//				footstep.pose.y = diffY;
//		}
//		if (diffAngle >= -ivFootMaxStepTheta - ivFootstepAccuracyTheta &&
//			diffAngle <= ivFootMaxInverseStepTheta + ivFootstepAccuracyTheta)
//		{
//			thetaInRange = true;
//		}
//		else
//		{
//			if (diffAngle < -ivFootMaxStepTheta)
//				footstep.pose.theta = -ivFootMaxStepTheta;
//			else if (diffAngle > ivFootMaxInverseStepTheta)
//				footstep.pose.theta = ivFootMaxInverseStepTheta;
//			else
//				footstep.pose.theta = diffAngle;
//		}
//	}
//	else // leg =left
//	{
//		if (diffY <= ivFootMaxStepY + ivFootstepAccuracyY &&
//			diffY >= -ivFootMaxInverseStepY - ivFootstepAccuracyY)
//		{
//			yInRange = true;
//		}
//		else
//		{
//			if (diffY > ivFootMaxStepY)
//				footstep.pose.y = ivFootMaxStepY;
//			else if (diffY < -ivFootMaxInverseStepY)
//				footstep.pose.y = -ivFootMaxInverseStepY;
//			else
//				footstep.pose.y = diffY;
//		}
//		if (diffAngle <= ivFootMaxStepTheta + ivFootstepAccuracyTheta &&
//			diffAngle >= -ivFootMaxInverseStepTheta - ivFootstepAccuracyTheta)
//		{
//			thetaInRange = true;
//		}
//		else
//		{
//			if (diffAngle > ivFootMaxStepTheta)
//				footstep.pose.theta = ivFootMaxStepTheta;
//			else if (diffAngle < -ivFootMaxInverseStepTheta)
//				footstep.pose.theta = -ivFootMaxInverseStepTheta;
//			else
//				footstep.pose.theta = diffAngle;
//		}
//	}
//
//	return xInRange && yInRange && thetaInRange;
//
//}


/*
 * bool FootstepPlanner::getGreadyFootstep(humanoid_nav_msgs::StepTarget& footstep,
 *                                         tf::Transform& supportFoot,
 *                                         tf::Transform& footPlacement)
 * --------------------------
 * Check if 'footPlacement' can be reached from 'supportFoot' by any footstep
 * (i.e. not only the discretized footsteps).
 */
bool
FootstepPlanner::getGreedyFootstep(const tf::Transform& supportFoot,
                                   const tf::Transform& footPlacement,
                                   humanoid_nav_msgs::StepTarget& footstep)
{

	bool xInRange = false;
	bool yInRange = false;
	bool thetaInRange = false;

	// calculate the necessary footstep to reach the footstep placement
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


/*
 * bool FootstepPlanner::getFootPositions(const State& robot, State& leftFoot, State& rightFoot)
 * --------------------------
 * ...
 */
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
FootstepPlanner::getFootTransform(tf::Transform& foot,
                                  const std::string& from,
                                  const std::string& to,
                                  const ros::Time& time)
{

	tf::StampedTransform stampedFootTransform;
	ivTransformListener.waitForTransform(to, from, time, ros::Duration(0.1));
	ivTransformListener.lookupTransform(to, from, time, stampedFootTransform);

	foot.setOrigin(stampedFootTransform.getOrigin());
	foot.setRotation(stampedFootTransform.getRotation());

}


/*
 * bool FootstepPlanner::occupied(const State& u)
 * --------------------------
 * Check whether this state collides with an obstacle or not.
 */
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


/*
 * void FootstepPlanner::broadcastFootstepPathVis()
 * --------------------------
 * Publishes the footstep path.
 */
void
FootstepPlanner::broadcastFootstepPathVis()
{

	if (!ivStartPoseSet and !ivRobotPoseSet)
	{
		ROS_ERROR("Either no start pose or no robot pose has been received yet.");
		return;
	}

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
			marker.ns = "footstep_planning";
			marker.id = j;
			marker.action = visualization_msgs::Marker::DELETE;

			markerVector.push_back(marker);
		}
	}
	markerMsg.markers = markerVector;
	ivLastMarkerMsgSize = markerVector.size();

	ivFootstepPathVisPub.publish(markerMsg);

}


/*
 * void FootstepPlanner::broadcastPathVis()
 * --------------------------
 * Publishes the calculated zig-zag state path.
 */
void
FootstepPlanner::broadcastPathVis()
{

	if (!ivStartPoseSet and !ivRobotPoseSet)
	{
		ROS_ERROR("Either no start pose or no robot pose has been received yet.");
		return;
	}

	nav_msgs::Path pathMsg;
	geometry_msgs::PoseStamped state;
	std::vector<geometry_msgs::PoseStamped> poses;

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
		poses.push_back(state);
		state.pose.position.x = ivStartFootLeft.getX();
		state.pose.position.y = ivStartFootLeft.getY();
		poses.push_back(state);
	}
	else // leg == LEFT
	{
		state.pose.position.x = ivStartFootLeft.getX();
		state.pose.position.y = ivStartFootLeft.getY();
		poses.push_back(state);
		state.pose.position.x = ivStartFootRight.getX();
		state.pose.position.y = ivStartFootRight.getY();
		poses.push_back(state);
	}

	for(; pathIter != ivDstarPtr->getPathEnd(); pathIter++)
	{
		state.pose.position.x = pathIter->getX();
		state.pose.position.y = pathIter->getY();
		poses.push_back(state);
	}
	pathMsg.header = state.header;
	pathMsg.poses = poses;

	ivPathVisPub.publish(pathMsg);

}


/*
 * void FootstepPlanner::broadcastExpandedNodesVis()
 * --------------------------
 * Publishes the expanded states of the search as point cloud.
 */
void
FootstepPlanner::broadcastExpandedNodesVis()
{

	if (!ivStartPoseSet and !ivRobotPoseSet)
	{
		ROS_ERROR("Either no start pose or no robot pose has been received yet.");
		return;
	}

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

	if (!ivStartPoseSet and !ivRobotPoseSet)
	{
		ROS_ERROR("Either no start pose or no robot pose has been received yet.");
		return;
	}

	if (ivMode == MERE_PLANNING)
	{
		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = ivMapPtr->getFrameID();
	}
	else if (ivMode == ROBOT_NAVIGATION)
		marker.header = ivRobotHeader;
	marker.ns = "footstep_planning";
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
	marker.color.a = 0.2;

	marker.lifetime = ros::Duration();

}

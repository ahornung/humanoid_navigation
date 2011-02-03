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

#ifndef FOOTSTEPPLANNER_H_
#define FOOTSTEPPLANNER_H_

#include <assert.h>
#include <footstep_planner/Astar.h>
#include <footstep_planner/Dstar.h>
#include <footstep_planner/helper.h>
#include <footstep_planner/Heuristic.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <humanoid_nav_msgs/StepTargetService.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>


namespace footstep_planner
{

	/**
	 * @brief The Footstep Planner class, offering the public interface to
	 * use for (re)planning. Internally uses Dstar for planning.
	 *
	 */
	class FootstepPlanner
	{

	public:

		enum PlanningMode { MERE_PLANNING=0, ROBOT_NAVIGATION=1 };
		enum HeuristicType { EUCLIDEAN=0, EUCLIDEAN_STEPCOST=1, ASTAR_PATH=2 };

		FootstepPlanner();
		virtual ~FootstepPlanner();

		/**
		 * @brief Plan a single footstep sequence from a (static) start to a (static) goal location.
		 *
		 * @return success of planning
		 */
		bool plan(const geometry_msgs::PoseStampedConstPtr& start, const geometry_msgs::PoseStampedConstPtr& goal);

		/**
		 * @brief Plan a single footstep sequence from a (static) start to a (static) goal location.
		 *
		 * @return success of planning
		 */
		bool plan(float startX, float startY, float startTheta, float goalX, float goalY, float goalTheta);

		/**
		 * @brief Plan a single footstep sequence for a (static) start and goal location.
		 *
		 * @return success of planning
		 */
		bool plan();

		/**
		 * @brief Sets a new (static) goal location. The coordinates need to be in the map coordinate frame.
		 *
		 * @return false if the goalPose is invalid
		 */
		bool setGoal(const geometry_msgs::PoseStampedConstPtr& goalPose);

		/**
		 * @brief Sets a new (static) goal location. The coordinates need to be in the map coordinate frame.
		 *
		 * @return false if the goalPose is invalid
		 */
		bool setGoal(float x, float y, float theta);

		/**
		 * @brief Sets a new (static) start location. The coordinates need to be in the map coordinate frame.
		 *
		 * @return false if the startPose is invalid
		 */
		bool setStart(const geometry_msgs::PoseStampedConstPtr& startPose);

		/**
		 * @brief Sets a new (static) start location. The coordinates need to be in the map coordinate frame.
		 *
		 * @return false if the startPose is invalid
		 */
		bool setStart(float x, float y, float theta);

		/**
		 * @brief Sets a new 2D grid and updates the map of the underlying Dstar.
		 */
		void setMap(boost::shared_ptr<GridMap2D> gridMap);

		/**
		 * @brief Callback method to set the desired goal pose (e.g. via rviz).
		 */
		void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goalPose);

		/**
		 * @brief Callback method to set the robot's pose if the pose is set manually (e.g. via rviz).
		 * Be sure to have set the parameter planning_mode to 0 if you want to use this callback.
		 */
		void startPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& startPose);

		/**
		 * @brief Callback method to update the robot's pose. Be sure to have set the parameter planning_mode
		 * to 1 if you want to use this callback.
		 */
		void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& robotPose);

		/**
		 * @brief Callback method to set a new grid map. Also updates the map of the underlying Dstar.
		 */
		void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancyMap);

		/**
		 * @return the current planning mode that has been defined in the parameter settings
		 */
		PlanningMode getPlanningMode() const { return ivMode; };


	private:

		boost::shared_ptr<Dstar>     ivDstarPtr;
		boost::shared_ptr<GridMap2D> ivMapPtr;
		boost::mutex ivRobotPoseUpdateMutex;

		State ivStartFootLeft;
		State ivStartFootRight;
		State ivGoalFootLeft;
		State ivGoalFootRight;

		int    ivCollisionCheckAccuracy;
		bool   ivDstarSetUp;
		bool   ivStartPoseSet;
		bool   ivGoalPoseSet;
		bool   ivRobotPoseSet;
		bool   ivExecutingFootsteps;
		double ivFootsizeX;
		double ivFootsizeY;
		double ivFootsizeZ;
		double ivFootOriginShiftX;
		double ivFootOriginShiftY;
		double ivFootMaxStepX;
		double ivFootMaxStepY;
		double ivFootMaxStepTheta;
		double ivFootMaxInverseStepX;
		double ivFootMaxInverseStepY;
		double ivFootMaxInverseStepTheta;
		double ivFootSeparation;
		double ivFootstepAccuracyX;
		double ivFootstepAccuracyY;
		double ivFootstepAccuracyTheta;
		int    ivLastMarkerMsgSize;

		PlanningMode ivMode;

		std::string ivRFootID;
		std::string ivLFootID;

		roslib::Header ivRobotHeader;

		ros::NodeHandle    ivNh;
		ros::Publisher     ivExpandedStatesVisPub;
		ros::Publisher	   ivFootstepPathVisPub;
		ros::Publisher     ivStartPoseVisPub;
		ros::Publisher	   ivPathVisPub;
		ros::ServiceClient ivFootstepService;

		tf::TransformListener ivTransformListener;

		/**
		 * @brief Publishes the calculated footstep path under "footstep_planning/path" as visualization_msgs::MarkerArray.
		 */
		void broadcastFootstepPathVis();

		/**
		 * @brief Publishes the expanded states of the planner under "footstep_planning/expanded_states" as sensor_msgs::PointCloud.
		 */
		void broadcastExpandedNodesVis();

		/**
		 * @brief Publishes the calculated path under "footstep_planning/footsteps_array" as nav_msgs::Path.
		 */
		void broadcastPathVis();

		/**
		 * @brief Administrates the underlying Dstar. Initiates the (re)planning.
		 *
		 * @return success of planning
		 */
		bool dstarPlanning();

		/**
		 * @brief Performs the calculated footsteps. Initializes a replanning if the
		 * environment changes or the deviation to the calculated path due to unclean
		 * performed footsteps is too large.
		 */
		void executeFootsteps();

		/**
		 * @brief Transforms a footstep into a visualization message.
		 */
		void footstepToMarker(const State& footstep, visualization_msgs::Marker& marker);

		/**
		 * @brief Determines the position of the robot's left and right foot. Only used if parameter
		 * planning_mode is set to 0.
		 */
		void getFootPositions(const State& robot, State& footLeft, State& footRight);

		/**
		 * @brief Receives the pose of the robot's foot.
		 */
		void getFootTransform(const std::string& from,
							  const std::string& to,
							  const ros::Time& time,
							  tf::Transform& foot);

		/**
		 * @brief Calculates for a given support foot and the desired footstep placement the most
		 * applicable footstep. If the robot (limited by its step size) could not perform the
		 * necessary step, choose the step that gets closest to the placement.
		 *
		 * @param supportFoot the robot's current support foot
		 * @param footPlacement the placement where to put the other foot (referred to the support foot)
		 * @param footstep the most applicable footstep to be used
		 *
		 * @return false if the calculated footstep comes up to the defined limitations of the robot
		 */
		bool getGreedyFootstep(const tf::Transform& supportFoot,
							   const tf::Transform& footPlacement,
							   humanoid_nav_msgs::StepTarget& footstep);

		/**
		 * @brief Initiates the footstep planning. If the parameter planning_mode is set 1
		 * also controls the navigation of the robot to the given goal pose.
		 */
		void navigate();

		/**
		 * @return true if a footstep placement in state u collides with an obstacle
		 */
		bool occupied(const State& u);


	};

}


#endif /* FootstepPlanner_H_ */

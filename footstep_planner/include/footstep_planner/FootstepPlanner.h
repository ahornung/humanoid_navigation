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
#include <list>
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
		 * @brief Plan a single footstep sequence from start to goal pose
		 *
		 * @return success of planning
		 */
		bool plan(const geometry_msgs::PoseStampedConstPtr& start, const geometry_msgs::PoseStampedConstPtr& goal);

		/**
		 * @brief Plan a single footstep sequence from start to goal pose
		 *
		 * @return success of planning
		 */
		bool plan(float startX, float startY, float startTheta, float goalX, float goalY, float goalTheta);

		/**
		 * @brief Plan a single footstep sequence where start and goal have been set by the appropriate methods
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

		/// Sets a new 2D grid, also updates the map of its Dstar-planner
		void setMap(boost::shared_ptr<GridMap2D> gridMap);

		void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goalPose);
		void startPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& startPose);
		void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& robotPose);

		/// Callback on a map message, calls setMap() accordingly
		void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancyMap);

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

		void broadcastFootstepPathVis();
		void broadcastExpandedNodesVis();
		void broadcastPathVis();
		bool dstarPlanning();
		void executeFootsteps();
		void footstepToMarker(const State& step, visualization_msgs::Marker& printOut);
		void getFootPositions(const State& robot, State& footLeft, State& footRight);
		void getFootTransform(tf::Transform& footstep,
							  const std::string& from,
							  const std::string& to,
							  const ros::Time& time);
		bool getGreedyFootstep(const tf::Transform& supportFoot,
							   const tf::Transform& footPlacement,
							   humanoid_nav_msgs::StepTarget& footstep);
		void navigate();

		/// return true if the foot in state u would collide with an obstacle
		bool occupied(const State& u);

		void performPathAdjustment();


	};

}

#endif /* FootstepPlanner_H_ */

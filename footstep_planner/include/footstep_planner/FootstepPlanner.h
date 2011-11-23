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


#ifndef HUMANOID_SBPL_FOOTSTEPPLANNER_H_
#define HUMANOID_SBPL_FOOTSTEPPLANNER_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <humanoid_nav_msgs/StepTargetService.h>
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <footstep_planner/helper.h>
#include <footstep_planner/PathCostHeuristic.h>
#include <footstep_planner/FootstepPlannerEnvironment.h>
#include <footstep_planner/PlanningStateChangeQuery.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sbpl/headers.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <assert.h>


namespace footstep_planner
{
	typedef std::vector<State>::const_iterator state_iter_t;

    class FootstepPlanner
    {
    public:

        FootstepPlanner();
        virtual ~FootstepPlanner();

        bool plan();
        bool plan(const geometry_msgs::PoseStampedConstPtr& start,
                  const geometry_msgs::PoseStampedConstPtr& goal);
        bool plan(float start_x, float start_y, float start_theta,
                  float goal_x, float goal_y, float goal_theta);

		/// Service handle to plan footsteps
		bool planService(humanoid_nav_msgs::PlanFootsteps::Request &req, humanoid_nav_msgs::PlanFootsteps::Response &resp);

        bool setGoal(const geometry_msgs::PoseStampedConstPtr& goal_pose);
        bool setGoal(float x, float y, float theta);
        bool setStart(const geometry_msgs::PoseStampedConstPtr& start_pose);
        bool setStart(float x, float y, float theta);
        void setMap(GridMap2DPtr gridMap);
        void setMarkerNamespace(const std::string& ns) { ivMarkerNamespace = ns; };
        void setMaxSearchTime(int search_time) { ivMaxSearchTime = search_time; };

        void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose);
        void startPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose);
        void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map);

        void clearFootstepPathVis(unsigned num_footsteps=0);

		/**
		 * @return Costs of the planned footstep path.
		 */
		double getPathCosts() const { return ivPathCost; };

		/**
		 * @return Number of expanded states.
		 */
		size_t getNumExpandedStates() const { return ivPlannerPtr->get_n_expands(); };

		/**
		 * @return Number of planned footsteps.
		 */
		size_t getNumFootsteps() const { return ivPath.size(); };

		state_iter_t getPathBegin() const { return ivPath.begin(); };
		state_iter_t getPathEnd() const { return ivPath.end(); };

    private:
        void broadcastExpandedNodesVis();
        void broadcastFootstepPathVis();
        void broadcastHeuristicPathVis();
        void broadcastPathVis();
        bool extractSolution(const std::vector<int>& state_ids);
        void footstepToMarker(const State& footstep,
                              visualization_msgs::Marker* marker);
        bool occupied(const State& u);
        bool run();

        void getFootPositionLeft(const State& robot, State* foot_left);
        void getFootPositionRight(const State& robot, State* foot_right);

        void setupPlanner();

        boost::shared_ptr<Heuristic> ivHeuristicPtr;
        boost::shared_ptr<FootstepPlannerEnvironment> ivPlannerEnvironmentPtr;
        boost::shared_ptr<GridMap2D> ivMapPtr;
        boost::shared_ptr<SBPLPlanner> ivPlannerPtr;
        boost::shared_ptr<PathCostHeuristic> ivPathCostHeuristicPtr;

        std::vector<Footstep> ivFootstepSet;

        std::vector<State> ivPath;

        State ivStartFootLeft;
        State ivStartFootRight;
        State ivGoalFootLeft;
        State ivGoalFootRight;

        ros::Publisher  ivExpandedStatesVisPub;
        ros::Publisher  ivFootstepPathVisPub;
        ros::Subscriber ivGridMapSub;
        ros::Publisher  ivHeuristicPathVisPub;
        ros::Publisher  ivPathVisPub;
        ros::Publisher  ivStartPoseVisPub;
        ros::ServiceServer ivFootstepPlanService;

        double ivFootSeparation;
        double ivOriginFootShiftX, ivOriginFootShiftY;
        double ivFootsizeX, ivFootsizeY, ivFootsizeZ;
        double ivMaxFootstepX, ivMaxFootstepY, ivMaxFootstepTheta;
        double ivMaxInvFootstepX, ivMaxInvFootstepY, ivMaxInvFootstepTheta;
        double ivMaxStepWidth;
        int    ivCollisionCheckAccuracy;
        double ivFootstepAccuracyX, ivFootstepAccuracyY, ivFootstepAccuracyTheta;
        bool   ivStartPoseSetUp, ivGoalPoseSetUp;
        bool   ivPlanExists;
        int    ivLastMarkerMsgSize;
        double ivPathCost;
        double ivCellSize;
        bool   ivSearchUntilFirstSolution;
        double ivMaxSearchTime;
        double ivInitialEpsilon;
        bool   ivForwardSearch;
        int    ivNumAngleBins;

        unsigned int ivChangedCellsLimit;

        std::string ivPlannerType;
        std::string ivRFootID;
        std::string ivLFootID;
        std::string ivMarkerNamespace;
    };
}

#endif  // HUMANOID_SBPL_HUMANOID_SBPL_

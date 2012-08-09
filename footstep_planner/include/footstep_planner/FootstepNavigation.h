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

#ifndef FOOTSTEP_PLANNER_FOOTSTEPNAVIGATION_H_
#define FOOTSTEP_PLANNER_FOOTSTEPNAVIGATION_H_

#include <actionlib/client/simple_action_client.h>
#include <footstep_planner/FootstepPlanner.h>
#include <footstep_planner/State.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <humanoid_nav_msgs/ClipFootstep.h>
#include <humanoid_nav_msgs/ExecFootstepsAction.h>
#include <humanoid_nav_msgs/ExecFootstepsFeedback.h>
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <humanoid_nav_msgs/StepTargetService.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <assert.h>


namespace footstep_planner
{
	/**
	 * @brief A class to control the performance of a planned footstep path on
	 * the NAO robot.
	 */
    class FootstepNavigation
    {
    public:
        FootstepNavigation();
        virtual ~FootstepNavigation();

        /// @brief Wrapper for FootstepPlanner::setGoal.
        bool setGoal(const geometry_msgs::PoseStampedConstPtr& goal_pose);
        /// @brief Wrapper for FootstepPlanner::setGoal.
        bool setGoal(float x, float y, float theta);

        /**
         * @brief Callback to retreive the robot's position.
         *
         * Subscribed to 'amcl_pose'.
         */
        void robotPoseCallback(
			const geometry_msgs::PoseWithCovarianceStampedConstPtr& robotPose);

        /**
         * @brief Callback to set a simulated robot at a certain pose.
		 *
		 * Subscribed to 'goal'.
         */
        void goalPoseCallback(
        		const geometry_msgs::PoseStampedConstPtr& goal_pose);

        /**
         * @brief Callback to set the map.
         *
         * Subscribed to 'map'.
         */
        void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map);

    protected:
        /// @brief Starts the planning task via FootstepPlanner::plan().
        void run();

        /**
         * @brief Obtains the pose of the robot's foot from tf.
         */
        void getFootTransform(const std::string& foot_id,
		                      const std::string& world_frame_id,
		                      const ros::Time& time,
		                      tf::Transform& foot);

        /**
         * @brief Calculates the footstep necessary to reach 'to' from within
         * 'from'.
         *
         * @return True if an performable footstep has been found.
         */
        bool getFootstep(const tf::Pose& from, const State& to,
                         humanoid_nav_msgs::StepTarget& footstep);

        bool getFootstepsFromPath(
        		const State& current_support_leg, int starting_step_num,
        		std::vector<humanoid_nav_msgs::StepTarget>& footsteps);

        /// @brief Updates the robot's current pose.
        bool updateStart();

        /// @brief Executes footsteps as boost::thread.
        void executeFootsteps();

        /**
         * @brief Alternative (and more fluid) execution of footsteps using
         * ROS's actionlib. (NOTE: not fully implemented so far!)
         */
        void executeFootsteps_alt();

        void activeCallback();
        void doneCallback(
        		const actionlib::SimpleClientGoalState& state,
                const humanoid_nav_msgs::ExecFootstepsResultConstPtr& result);
        void feedbackCallback(
				const humanoid_nav_msgs::ExecFootstepsFeedbackConstPtr& fb);

        /**
         * @param footstep The response from the clip footstep service (i.e. it
         * contains the unclipped (calculated) step and the clipped
         * (performable) step).
         *
         * @return True if the footstep can be performed by the robot (i.e. it
         * is within the robot's max ranges).
         */
        bool performanceValid(const humanoid_nav_msgs::ClipFootstep& footstep);

        /// @return True if both states are equal upon some accuracy.
        bool performanceValid(const State& planned, const State& executed);

        /// @return True if both states are equal upon some accuracy.
        bool performanceValid(float a_x, float a_y, float a_theta,
                              float b_x, float b_y, float b_theta);

        void restartFootstepExecution();

        FootstepPlanner ivPlanner;

        ros::Subscriber ivGridMapSub, ivRobotPoseSub, ivGoalPoseSub;

        ros::Publisher  ivPathVisPub;

        ros::ServiceClient ivFootstepSrv;
        ros::ServiceClient ivClipFootstepSrv;

        tf::TransformListener ivTransformListener;

        boost::mutex ivRobotPoseUpdateMutex;

        ros::Time ivLastRobotTime;

        std::string ivIdFootRight;
        std::string ivIdFootLeft;
        std::string ivIdMapFrame;

        double ivAccuracyX;
        double ivAccuracyY;
        double ivAccuracyTheta;
        double ivCellSize;
        int    ivNumAngleBins;
        /// Used to lock the calculation and execution of footsteps.
        bool   ivExecutingFootsteps;
        /// The rate the action server sends its feedback.
        double ivFeedbackFrequency;

        /// @brief Simple action client to control a footstep execution.
    	actionlib::SimpleActionClient<
				humanoid_nav_msgs::ExecFootstepsAction> ivFootstepsExecution;

    	int ivExecutionShift;
    	int ivControlStepIdx;
    	int ivResetStepIdx;

    	bool ivProtectiveExecution;
    };
}
#endif  // FOOTSTEP_PLANNER_FOOTSTEPNAVIGATION_H_

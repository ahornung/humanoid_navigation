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

    class FootstepNavigation
    {
    public:

        FootstepNavigation();
        virtual ~FootstepNavigation();

        /// calls setGoal(x,y,theta)
        bool setGoal(const geometry_msgs::PoseStampedConstPtr& goal_pose);

        /**
         * @brief Sets the goal of the underlying planner
         * @return success of operation (goal valid)
         */
        bool setGoal(float x, float y, float theta);

        void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& robotPose);

        /**
         * @brief Set goal of planner and start planning from the current foot configuration
         * @param goal_pose
         */
        void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose);
        void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map);

    protected:
        void debugFootstepExecution(const tf::Transform& from,
		                            const State& from_planned,
		                            const State& to_planned);

        void run();

        /// @brief Obtains the pose of the robot's foot from tf
        void getFootTransform(const std::string& footID, const std::string& worldFrameID,
            		const ros::Time& time, tf::Transform& foot);
        /**
         * @brief Calculates for a given support foot and the desired foot placement
         * the most applicable footstep. If the robot (limited by its step size)
         * could not perform the necessary step, choose the step that gets closest
         * to the placement.
         *
         * @param supportFoot the robot's current support foot (global coordinates)
         * @param footPlacement where to put the other foot (global coordinates)
         * @param footstep the most applicable relative footstep to be used (within bounds)
         *
         * @return false if the calculated footstep does not reach footPlacement (out of limits)
         */
        bool getFootstep(const State& supportFoot,
                         const State& footPlacement,
                         humanoid_nav_msgs::StepTarget& footstep);

        bool updateStart();

        /// Main execution loop, will be called from a boost::thread
        void executeFootsteps();
        void executeFootsteps2();

        void activeCallback();
        void doneCallback(
        		const actionlib::SimpleClientGoalState& state,
                const humanoid_nav_msgs::ExecFootstepsResultConstPtr& result);
        void feedbackCallback(
        		const humanoid_nav_msgs::ExecFootstepsFeedbackConstPtr& fb);

        bool performable(const humanoid_nav_msgs::ClipFootstep& step);


        FootstepPlanner ivPlanner;

        ros::Subscriber ivGridMapSub, ivRobotPoseSub, ivGoalPoseSub;
        ros::Publisher  ivPathVisPub;
        ros::ServiceClient ivFootstepSrv;
        ros::ServiceClient ivClipFootstepSrv;
        tf::TransformListener ivTransformListener;
        boost::mutex ivRobotPoseUpdateMutex;

        ros::Time ivLastRobotTime;

        std::string ivFootIDRight;
        std::string ivFootIDLeft;
        std::string ivMapFrameID;

        double ivAccuracyX, ivAccuracyY, ivAccuracyTheta;
        double ivCellSize;
        int    ivNumAngleBins;
        bool   ivExecutingFootsteps;

        double ivFeedbackRate;

    	actionlib::SimpleActionClient<
				humanoid_nav_msgs::ExecFootstepsAction> ivFootstepsExecution;
    };
}

#endif  // HUMANOID_SBPL_HUMANOID_SBPL_

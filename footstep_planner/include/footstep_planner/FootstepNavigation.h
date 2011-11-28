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

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <humanoid_nav_msgs/StepTargetService.h>
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <footstep_planner/FootstepPlanner.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <assert.h>


namespace footstep_planner
{

    class FootstepNavigation
    {
    public:

        FootstepNavigation();
        virtual ~FootstepNavigation();

        bool setGoal(const geometry_msgs::PoseStampedConstPtr& goal_pose);
        bool setGoal(float x, float y, float theta);

        void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose);
        void startPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose);
        void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map);

    protected:
        void getFootTransform(const std::string& from, const std::string& to,
            		const ros::Time& time, tf::Transform& foot);
        bool run();
        FootstepPlanner ivPlanner;

        ros::Subscriber ivGridMapSub;
        ros::Publisher  ivPathVisPub;

        std::string ivRFootID;
        std::string ivLFootID;
    };
}

#endif  // HUMANOID_SBPL_HUMANOID_SBPL_

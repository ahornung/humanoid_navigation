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
        : ivRFootID("/RFoot_link"),
          ivLFootID("/LFoot_link")
    {
        // private NodeHandle for parameters and private messages (debug / info)
        ros::NodeHandle nh_private("~");
        ros::NodeHandle nh_public;

        nh_private.param("rfoot_frame_id", ivRFootID, ivRFootID);
        nh_private.param("lfoot_frame_id", ivLFootID, ivLFootID);
    }


	FootstepNavigation::~FootstepNavigation()
    {}

    bool
    FootstepNavigation::run()
    {
        // TODO: planning /replanning loop, will be called from a boost thread

    	return true;
    }


    void
    FootstepNavigation::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose)
    {
    	bool success = setGoal(goal_pose);
    	if (success)
    	{
    		run();
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
        ivPlanner.setMap(gridMap);
    }




    bool
    FootstepNavigation::setGoal(float x, float y, float theta)
    {
    	return ivPlanner.setGoal(x,y,theta);

    }

    void
    FootstepNavigation::getFootTransform(const std::string& from,
    		const std::string& to,
    		const ros::Time& time,
    		tf::Transform& foot)
    {

    	tf::StampedTransform stampedFootTransform;
    	// TODO: try/catch?
    	ivTransformListener.waitForTransform(to, from, time, ros::Duration(0.1));
    	ivTransformListener.lookupTransform(to, from, time, stampedFootTransform);

    	foot.setOrigin(stampedFootTransform.getOrigin());
    	foot.setRotation(stampedFootTransform.getRotation());

    }



}

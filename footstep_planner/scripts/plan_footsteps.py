#!/usr/bin/env python

#
# A simple service client calling a running footstep_planner node.
# Using this program you can request footstep plans from the command line.
#
# Author: Armin Hornung, University of Freiburg
#
# This is program is part of the ROS footstep planner:
# http://www.ros.org/wiki/footstep_planner
# License: GPL 3
#

import roslib
roslib.load_manifest('footstep_planner')
import rospy

from humanoid_nav_msgs.srv import *
from geometry_msgs.msg import Pose2D

import sys


if __name__ == '__main__':
    if len(sys.argv) != 7:
        sys.exit('\nUSAGE: %s <start> <goal>\n  where <start> and <goal> consist of "x y theta" in world coordinates\n\n' % sys.argv[0])

    rospy.init_node('plan_footsteps')
    planSrv = rospy.ServiceProxy("plan_footsteps", PlanFootsteps)
    start = Pose2D()
    goal = Pose2D()

    start.x = float(sys.argv[1])
    start.y = float(sys.argv[2])
    start.theta = float(sys.argv[3])

    goal.x = float(sys.argv[4])
    goal.y = float(sys.argv[5])
    goal.theta = float(sys.argv[6])

    rospy.loginfo("Calling footstep planner service from (%f %f %f) to (%f %f %f)...",
                  start.x, start.y, start.theta, goal.x, goal.y, goal.theta)
    resp = planSrv(start, goal)

    if resp.result == True:
        rospy.loginfo("Planning succeeded with %d steps, path costs: %f" % (len(resp.footsteps), resp.costs))
    else:
        rospy.logerr("Service call failed")

    exit(0)

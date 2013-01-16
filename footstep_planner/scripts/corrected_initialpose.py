#!/usr/bin/env python

# Maps the initialpose pointing to Nao's torso to a corrected pose with correct
# height.
#
# Author: Johannes Garimort
# License: BSD

import roslib
roslib.load_manifest('footstep_planner')
import rospy
import geometry_msgs.msg as geo_msgs


def callback(pose, pub):
    newpose = pose
    newpose.pose.pose.position.z += 0.315
    pub.publish(newpose)


def listener():
    rospy.init_node('corrected_initialpose')
    pub = rospy.Publisher('nao_corrected_initialpose',
                          geo_msgs.PoseWithCovarianceStamped)
    rospy.Subscriber("initialpose", geo_msgs.PoseWithCovarianceStamped,
                     callback, pub)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

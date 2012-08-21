// SVN $HeadURL$
// SVN $Id$

/*
 * hrl_kinematics - a kinematics library for humanoid robots based on KDL
 *
 * Copyright 2011-2012 Armin Hornung, University of Freiburg
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <hrl_kinematics/TestStability.h>
#include <ros/ros.h>


namespace hrl_kinematics {
  
/**
 * Simple node to use the Kinematics and TestStability classes.
 * Based on the current kinematic state, it will output wheather
 * the state is stable and publish some visualization
 */
class TestStabilityNode {
public:
  TestStabilityNode(Kinematics::FootSupport support = Kinematics::SUPPORT_DOUBLE);
  virtual ~TestStabilityNode();
  void jointStateCb(const sensor_msgs::JointStateConstPtr& state);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher visualization_pub_, support_polygon_pub_, pcom_pub_;
  TestStability test_stability_;
  Kinematics::FootSupport support_mode_;
};

TestStabilityNode::TestStabilityNode(Kinematics::FootSupport support)
: support_mode_ (support)
{
  ros::NodeHandle nh_private("~");
  visualization_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("stability_visualization", 1);
  support_polygon_pub_ = nh_private.advertise<geometry_msgs::PolygonStamped>("support_polygon", 10);
  pcom_pub_ = nh_private.advertise<visualization_msgs::Marker>("projected_com", 10);

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &TestStabilityNode::jointStateCb, this);
}

TestStabilityNode::~TestStabilityNode(){

}


void TestStabilityNode::jointStateCb(const sensor_msgs::JointStateConstPtr& state){
  size_t num_joints = state->position.size();
  ROS_DEBUG("Received JointState with %zu joints", num_joints);

  // TODO: need to obtain current support mode

  // get joint positions from state message
  std::map<std::string, double> joint_positions;
  for (unsigned int i=0; i<state->name.size(); i++)
    joint_positions.insert(make_pair(state->name[i], state->position[i]));

  tf::Vector3 normal_vector(0.0, 0.0, 1.0); // planar support
  // for testing, normal vector of arbitrary plane:
  //tf::Vector3 normal_vector(0.5, 0.0, 1.0);
  normal_vector.normalize();

  bool stable = test_stability_.isPoseStable(joint_positions, support_mode_, normal_vector);

  // print info
  tf::Point com = test_stability_.getCOM();
  if (stable)
    ROS_INFO("Pose is stable, pCOM at %f %f", com.x(), com.y());
  else
    ROS_INFO("Pose is NOT stable, pCOM at %f %f", com.x(), com.y());

  // publish visualization:

  // TODO link coms:
//  visualization_msgs::MarkerArray com_vis_markers;
//
//  visualization_pub_.publish(vis_markers);
//  visualization_msgs::Marker marker;
//  createCoGMarker(kdl_tree_.getRootSegment()->first, "torso", 0.04, com, marker);
//
//  com_vis_markers_.markers.push_back(marker);
//
//
//  visualization_pub_.publish(com_vis_markers_);

  support_polygon_pub_.publish(test_stability_.getSupportPolygon());
  pcom_pub_.publish(test_stability_.getCOMMarker());

}


}

using namespace hrl_kinematics;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_stability");

  Kinematics::FootSupport support = Kinematics::SUPPORT_DOUBLE;

  if (argc == 2){
    int i = atoi(argv[1]);
    if (i >= 0 && i <= 2)
      support = Kinematics::FootSupport(i);

  }

  try
  {
    ros::spinOnce();
    hrl_kinematics::TestStabilityNode StabilityNode(support);

    ros::spin();
  }
  catch(hrl_kinematics::Kinematics::InitFailed &e)
  {
    std::cerr << "Could not initialize kinematics node: " << e.what() << std::endl;
    // TODO: does not work?
    //ROS_INFO("Could not initialize kinematics node: %s\n", e.what());
    return 1;
  }

  return 0;

}


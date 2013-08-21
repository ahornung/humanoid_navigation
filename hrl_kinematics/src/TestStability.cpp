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

#include "hrl_kinematics/TestStability.h"
#include <pcl/surface/convex_hull.h>
#include <pcl/point_types.h>

using robot_state_publisher::SegmentPair;
using boost::shared_ptr;

namespace hrl_kinematics {

TestStability::TestStability()
: Kinematics(), rfoot_mesh_link_name("RAnkleRoll_link")
{
  //Build support polygon with default scale 1.0
  initFootPolygon(1.0);
}

TestStability::~TestStability() {

}

bool TestStability::isPoseStable(const std::map<std::string, double>& joint_positions, FootSupport support_mode){
  tf::Vector3 normal(0.0, 0.0, 1.0);
  return isPoseStable(joint_positions, support_mode, normal);
}

bool TestStability::isPoseStable(const std::map<std::string, double>& joint_positions,
                                 FootSupport support_mode, const tf::Vector3& normal_vector)
{

  tf::Vector3 upright_vector(0.0, 0.0, 1.0);
  double angle = acos(upright_vector.dot(normal_vector.normalized()));

  tf::Quaternion q;
  if (std::abs(angle) < FLT_EPSILON)
    q=tf::createIdentityQuaternion();
  else{
    tf::Vector3 axis = upright_vector.cross(normal_vector).normalized();
    q = tf::Quaternion(axis, angle);
  }
  tf::Transform rotate_plane(q, tf::Vector3(0,0,0));

  tf::Point com; // center of mass in root frame
  double m;

  // transforms from root to left and right foot:
  tf::Transform tf_right_foot, tf_left_foot;
  computeCOM(joint_positions, com, m, tf_right_foot, tf_left_foot);

  tf::Transform tf_to_support;

  if (support_mode == SUPPORT_SINGLE_LEFT){
    support_polygon_ = foot_support_polygon_left_;
    tf_to_support_ = tf_left_foot;
  } else { // RIGHT or DOUBLE
    support_polygon_ = foot_support_polygon_right_;
    tf_to_support_ = tf_right_foot;

  }

  // rotate and project down
  for (unsigned i = 0; i < support_polygon_.size(); ++i){
    support_polygon_[i] = rotate_plane * support_polygon_[i];
  }

  // append left if double support:
  if (support_mode == SUPPORT_DOUBLE){
    tf::Transform tf_right_to_left = tf_right_foot.inverseTimes(tf_left_foot);
    for (unsigned i = 0; i < foot_support_polygon_left_.size(); ++i){
      support_polygon_.push_back(rotate_plane * tf_right_to_left * foot_support_polygon_left_[i]);
    }
    support_polygon_ = convexHull(support_polygon_);
  }
  if (support_polygon_.size() <= 2)
    return false;

  // projected com in support frame, rotated around support plane:
  p_com_ = rotate_plane * tf_to_support_.inverse() * com;
  p_com_.setZ(0.0);

  return pointInConvexHull(p_com_, support_polygon_);
}

geometry_msgs::PolygonStamped TestStability::getSupportPolygon() const{
  geometry_msgs::PolygonStamped footprint_poly;
  footprint_poly.header.frame_id = root_link_name_;
  footprint_poly.header.stamp = ros::Time::now();
  for (unsigned i=0; i < support_polygon_.size(); ++i){
    geometry_msgs::Point32 p;
    tf::Point tfP = tf_to_support_ * support_polygon_[i];
    p.x = tfP.x();
    p.y = tfP.y();
    p.z = tfP.z();
    footprint_poly.polygon.points.push_back(p);
  }

  return footprint_poly;
}

visualization_msgs::Marker TestStability::getCOMMarker() const{

  visualization_msgs::Marker com_marker;
  com_marker.header.stamp = ros::Time::now();
  com_marker.header.frame_id = root_link_name_;
  com_marker.action = visualization_msgs::Marker::ADD;
  com_marker.type = visualization_msgs::Marker::SPHERE;
  tf::pointTFToMsg(tf_to_support_ * p_com_, com_marker.pose.position);
  tf::quaternionTFToMsg(tf_to_support_.getRotation(), com_marker.pose.orientation);
  com_marker.scale.x = 0.03;
  com_marker.scale.y = 0.03;
  com_marker.scale.z = 0.005;
  com_marker.color.a = 1.0;
  com_marker.color.g = 1.0;
  com_marker.color.r = 0.0;


  return com_marker;
}

std::vector<tf::Point> TestStability::convexHull(const std::vector<tf::Point>& points) const{
  std::vector<tf::Point> hull;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> chull_points;
  pcl::ConvexHull<pcl::PointXYZ> chull;

  if (points.empty()){
    ROS_ERROR("convexHull on empty set of points!");
    return hull;
  }

  for (unsigned i = 0; i < points.size(); ++i){
    pcl_points->points.push_back(pcl::PointXYZ(points[i].x(), points[i].y(), 0.0));
  }

#if ROS_VERSION_MINIMUM(1,8,0) // test for Fuerte (newer PCL)
  chull.setDimension(2);
#else

#endif

  chull.setInputCloud(pcl_points);
  std::vector<pcl::Vertices> polygons;
  chull.reconstruct(chull_points, polygons);

  if (polygons.size() == 0){
    ROS_ERROR("Convex hull polygons are empty");
    return hull;
  } else if (polygons.size() > 1){
    ROS_WARN("Convex hull polygons are larger than 1");
  }

  for (unsigned i = 0; i < polygons[0].vertices.size(); ++i){
    int idx = polygons[0].vertices[i];
    tf::Point p(chull_points.points[idx].x,
                chull_points.points[idx].y,
                chull_points.points[idx].z);
    hull.push_back(p);
  }

  return hull;
}

bool TestStability::pointInConvexHull(const tf::Point& point, const std::vector<tf::Point>& polygon) const{
  assert(polygon.size() >=3);
  int positive_direction = 0;
  for (unsigned i = 0; i < polygon.size(); ++i){
    int i2 = (i+1)% (polygon.size());
    double dx = polygon[i2].getX() - polygon[i].getX();
    double dy = polygon[i2].getY() - polygon[i].getY();
    if (dx == 0.0 && dy == 0.0){
      ROS_DEBUG("Skipping polygon connection [%d-%d] (identical points)", i, i2);
      continue;
    }
    double line_test = (point.y() - polygon[i].getY())*dx - (point.x() - polygon[i].getX())*dy;
    if (i == 0)
      positive_direction = (line_test > 0.0);
    ROS_DEBUG("Line test [%d-%d] from (%f,%f) to (%f,%f): %f", i, i2, polygon[i].getX(), polygon[i].getY(),
              polygon[i2].getX(), polygon[i2].getY(), line_test);
    if ((line_test > 0.0) != positive_direction)
      return false;

  }

  return true;
}

void TestStability::initFootPolygon(double scale_convex_hull){
  
  //Init convex hull scaling factor
  scale_convex_hull_ = scale_convex_hull;

  // TODO: param?
  if (!loadFootPolygon()){
    ROS_WARN("Could not load foot mesh, using default points");

    foot_support_polygon_right_.push_back(tf::Point(0.07f, 0.023f, 0.0));
    foot_support_polygon_right_.push_back(tf::Point(0.07f, -0.03f, 0.0));
    foot_support_polygon_right_.push_back(tf::Point(-0.03f, -0.03f, 0.0));
    foot_support_polygon_right_.push_back(tf::Point(-0.03f, 0.02, 0.0));
  }

  // mirror for left:
  foot_support_polygon_left_ = foot_support_polygon_right_;
  for (unsigned i=0; i < foot_support_polygon_left_.size(); ++i){
    foot_support_polygon_left_[i] *= tf::Point(1.0, -1.0, 1.0);
  }

  // restore order of polygon
  foot_support_polygon_left_ = convexHull(foot_support_polygon_left_);
}


bool TestStability::loadFootPolygon(){
  boost::shared_ptr<const urdf::Link> foot_link =  urdf_model_.getLink(rfoot_mesh_link_name);
  assert(foot_link);
  boost::shared_ptr<const urdf::Geometry> geom;
  urdf::Pose geom_pose;
  if (foot_link->collision && foot_link->collision->geometry){
    geom = foot_link->collision->geometry;
    geom_pose = foot_link->collision->origin;
  } else if (foot_link->visual && foot_link->visual->geometry){
    geom = foot_link->visual->geometry;
    geom_pose = foot_link->visual->origin;
  } else{
    ROS_ERROR_STREAM("No geometry for link "<< rfoot_mesh_link_name << " available");
    return false;
  }

  tf::Pose geom_origin = tf::Pose(tf::Transform(tf::Quaternion(geom_pose.rotation.x, geom_pose.rotation.y, geom_pose.rotation.z, geom_pose.rotation.w),
                                              tf::Vector3(geom_pose.position.x, geom_pose.position.y, geom_pose.position.z)));

  if (geom->type != urdf::Geometry::MESH){
    ROS_ERROR_STREAM("Geometry for link "<< rfoot_mesh_link_name << " is not a mesh");
    return false;
  } else {
    shared_ptr<const urdf::Mesh> mesh = boost::dynamic_pointer_cast<const urdf::Mesh>(geom);

    //// arm_navigation (Fuerte / Groovy)
   tf::Vector3 scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
   shapes::Mesh* shape_mesh = shapes::createMeshFromFilename(mesh->filename, &scale);
   size_t vertex_count = shape_mesh->vertexCount;

    //// MoveIt:
//     const Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
//     shapes::Mesh* shape_mesh = shapes::createMeshFromResource(mesh->filename, scale);
//     size_t vertex_count = shape_mesh->vertex_count;

    //Vector storing the original foot points
    std::vector<tf::Point> foot_SP_right;
    for (unsigned int i = 0 ; i < vertex_count ; ++i)
    {
      unsigned int i3 = i * 3;

      tf::Point p(shape_mesh->vertices[i3], shape_mesh->vertices[i3 + 1], shape_mesh->vertices[i3 + 2]); // proj down (z=0)
      tf::Point projectedP = geom_origin*p;
      projectedP.setZ(0.0);
      // transform into local foot frame:
      //foot_support_polygon_right_.push_back(projectedP);
      foot_SP_right.push_back(projectedP);
    }

    //Compute foot center point w.r.t local frame
    float sum_x_coord = 0.0;
    float sum_y_coord = 0.0;
    tf::Point r_foot_center;
    for (unsigned int i = 0 ; i < foot_SP_right.size(); ++i)
    {
    	sum_x_coord = sum_x_coord + foot_SP_right[i].x();
    	sum_y_coord = sum_y_coord + foot_SP_right[i].y();
    }
    //X and Y of right foot center
    r_foot_center.setX(sum_x_coord/foot_SP_right.size());
    r_foot_center.setY(sum_y_coord/foot_SP_right.size());

    //Vector storing foot points w.r.t foot center
    std::vector<tf::Point> foot_SP_right_center;
    tf::Point foot_point;
    for (unsigned int i = 0 ; i < foot_SP_right.size(); ++i){
      //Express point w.r.t foot center and directly apply scaling
      foot_point.setX( (foot_SP_right[i].x() - r_foot_center.x()) * scale_convex_hull_ );
      foot_point.setY( (foot_SP_right[i].y() - r_foot_center.y()) * scale_convex_hull_ );
      foot_SP_right_center.push_back(foot_point);
    }

    //Express new(scaled) coordinates in local frame
    std::vector<tf::Point> scaled_SP_right;
    for (unsigned int i = 0 ; i < foot_SP_right_center.size() ; ++i){
      //Express point w.r.t foot center and directly apply scaling
      foot_point.setX( foot_SP_right_center[i].x() + r_foot_center.x() ) ;
      foot_point.setY( foot_SP_right_center[i].y() + r_foot_center.y() ) ;
      scaled_SP_right.push_back(foot_point);
    }


    //std::cout<<"Num points"<<scaled_SP_right.size()<<std::endl;

    //Without scaling
    //foot_support_polygon_right_ = convexHull(foot_support_polygon_right_);
    //foot_support_polygon_right_ = convexHull(foot_SP_right);

    //With scaling
    foot_support_polygon_right_ = convexHull(scaled_SP_right);

  }

  ROS_INFO("Foot polygon loaded with %zu points", foot_support_polygon_right_.size());

  return true;

}


//Function to guarantee correct double support
void TestStability::scaleConvexHull(double scaling_factor)
{
	//Reinit convex hull with new scaling factor
	initFootPolygon(scaling_factor);
}



} /* namespace hrl_kinematics */

// SVN $HeadURL$
// SVN $Id$

/*
 * 6D localization for humanoid robots
 *
 * Copyright 2009-2012 Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/humanoid_localization
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

#include <humanoid_localization/MotionModel.h>

using namespace tf;
using namespace std;

namespace humanoid_localization{

MotionModel::MotionModel(ros::NodeHandle* nh, EngineT* rngEngine, tf::TransformListener* tf,
                         const std::string& odomFrameId, const std::string& baseFrameId)
: m_tfListener(tf),
  m_rngNormal(*rngEngine, NormalDistributionT(0.0, 1.0)),
  m_rngUniform(*rngEngine, UniformDistributionT(0.0, 1.0)),
  m_odomFrameId(odomFrameId), m_baseFrameId(baseFrameId),
  m_firstOdometryReceived(false)

{

  // motion model noise parameters:
  double xx, yy, zz, RR, PP, YY;
  nh->param("motion_noise/x", xx, 0.01);
  nh->param("motion_noise/y", yy, 0.01);
  nh->param("motion_noise/z", zz, 0.01);
  nh->param("motion_noise/roll", RR, 0.05);
  nh->param("motion_noise/pitch", PP, 0.1);
  nh->param("motion_noise/yaw", YY, 0.5);
  m_motionNoise(0) = xx; // x
  m_motionNoise(1) = yy; // y
  m_motionNoise(2) = zz; // z
  m_motionNoise(3) = RR; // roll
  m_motionNoise(4) = PP; // pitch
  m_motionNoise(5) = YY; // yaw

  m_odomCalibration2D = Eigen::Matrix3d::Identity();
  nh->param("motion_calib/xx", m_odomCalibration2D(0,0), 1.0);
  nh->param("motion_calib/xy", m_odomCalibration2D(0,1), 0.0);
  nh->param("motion_calib/xt", m_odomCalibration2D(0,2), 0.0);
  nh->param("motion_calib/yx", m_odomCalibration2D(1,0), 0.0);
  nh->param("motion_calib/yy", m_odomCalibration2D(1,1), 1.0);
  nh->param("motion_calib/yt", m_odomCalibration2D(1,2), 0.0);
  nh->param("motion_calib/tx", m_odomCalibration2D(2,0), 0.0);
  nh->param("motion_calib/ty", m_odomCalibration2D(2,1), 0.0);
  nh->param("motion_calib/tt", m_odomCalibration2D(2,2), 1.0);

  // not used right now:
// covariance matrix, contains squared std.devs on diagonal
//  Matrix6f motionNoise = Matrix6f::Zero();
//  motionNoise.diagonal() = m_motionNoise.cwiseProduct( m_motionNoise);

//  if (motionNoise.isZero()){
//    m_motionNoiseL = Matrix6f::Zero();
//  } else{
//    m_motionNoiseL = motionNoise.llt().matrixL();
//  }

  reset();

}

MotionModel::~MotionModel() {

}

void MotionModel::transformPose(tf::Pose& particlePose, const tf::Transform& odomTransform){


  // apply transform to particle:
  particlePose *= odomTransform * odomTransformNoise(odomTransform);

  // HACK for testing: fix roll & pitch at 0:
//  	double roll, pitch, yaw;
//  	tf::Matrix3x3 basis = particlePose.getBasis();
//  	basis.getRPY(roll, pitch, yaw);
//  	basis.setRPY(0.0, 0.0, yaw);
//  	particlePose.setBasis(basis);


}

tf::Transform MotionModel::odomTransformNoise(const tf::Transform& odomTransform){
  // absolute amount of translation, used to scale odom noise:
  // (about 1-2cm for each update step)
  const double d = odomTransform.getOrigin().length();
  return tf::Transform(tf::createQuaternionFromRPY(
        d * m_motionNoise(3) * m_rngNormal(),
        d * m_motionNoise(4) * m_rngNormal(),
        d * m_motionNoise(5) * m_rngNormal()),
      tf::Vector3(
        d * m_motionNoise(0) * m_rngNormal(),
        d * m_motionNoise(1) * m_rngNormal(),
        d * m_motionNoise(2) * m_rngNormal()));
}

void MotionModel::reset(){
  m_firstOdometryReceived = false;
}

void MotionModel::applyOdomTransform(Particles& particles, const tf::Transform& odomTransform){
  tf::Transform calibratedOdomTransform = calibrateOdometry(odomTransform);

  for (unsigned i=0; i < particles.size(); ++i){
    transformPose(particles[i].pose, calibratedOdomTransform);
  }
}

bool MotionModel::applyOdomTransformTemporal(Particles& particles,const ros::Time& t, double dt){
  ros::WallTime startTime = ros::WallTime::now();

  // first see if default time is available
  tf::Transform odomTransform;
  if (!lookupOdomTransform(t, odomTransform))
    return false;


  tf::Transform timeSampledTransform;
  ros::Time maxTime;
  std::string errorString;
  m_tfListener->getLatestCommonTime(m_odomFrameId, m_baseFrameId, maxTime, &errorString);
  ros::Duration maxDuration = maxTime - t;

  for (unsigned i=0; i < particles.size(); ++i){
    if (dt > 0.0){
      ros::Duration duration(m_rngUniform()*dt -dt/2.0);
      // TODO: time t is time of first measurement in scan!
      if (duration > maxDuration)
        duration = maxDuration;
      if (lookupOdomTransform(t + duration, timeSampledTransform))
        transformPose(particles[i].pose, timeSampledTransform);
      else{
        ROS_WARN("Could not lookup temporal odomTransform");
        transformPose(particles[i].pose, odomTransform);
      }
    } else{
      transformPose(particles[i].pose, odomTransform);
    }
  }

  double dwalltime = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO_STREAM("OdomTransformTemporal took " << dwalltime << "s ");


  return true;
}

tf::Transform MotionModel::calibrateOdometry(const tf::Transform& odomTransform) const {
  Eigen::Vector3d odomPose2D;
  double roll, pitch;
  odomPose2D(0) = odomTransform.getOrigin().getX();
  odomPose2D(1) = odomTransform.getOrigin().getY();
  odomPose2D(2) = tf::getYaw(odomTransform.getRotation());
  odomTransform.getBasis().getRPY(roll, pitch, odomPose2D(2));

  odomPose2D = m_odomCalibration2D * odomPose2D;

  return tf::Transform(tf::createQuaternionFromRPY(roll, pitch, odomPose2D(2)),
                       tf::Vector3(odomPose2D(0), odomPose2D(1), odomTransform.getOrigin().getZ()));


}

bool MotionModel::lookupOdomTransform(const ros::Time& t, tf::Transform& odomTransform) const{
  tf::Stamped<tf::Pose> odomPose;

  if (t <= m_lastOdomPose.stamp_){
    ROS_WARN("Looking up OdomTransform that is %f ms older than the last odomPose!",
             (m_lastOdomPose.stamp_ - t).toSec()/1000.0);
  }

  if (!lookupOdomPose(t, odomPose))
    return false;

  odomTransform = computeOdomTransform(odomPose);
  return true;
}

tf::Transform MotionModel::computeOdomTransform(const tf::Transform currentPose) const{
  if (m_firstOdometryReceived){
    return m_lastOdomPose.inverse() * currentPose;
  } else{
    return tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0));
  }

}

void MotionModel::storeOdomPose(const tf::Stamped<tf::Pose>& odomPose){
  m_firstOdometryReceived = true;
  if (odomPose.stamp_ <= m_lastOdomPose.stamp_){
    ROS_WARN("Trying to store an OdomPose that is older or equal than the current in the MotionModel, ignoring!");
  } else {
    m_lastOdomPose = odomPose;
  }

}


bool MotionModel::lookupOdomPose(const ros::Time& t, tf::Stamped<tf::Pose>& odomPose) const
{
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)), t, m_baseFrameId);

  try
  {
    m_tfListener->transformPose(m_odomFrameId, ident, odomPose);
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }

  return true;
}


bool MotionModel::lookupLocalTransform(const std::string& targetFrame, const ros::Time& t,
                                       tf::StampedTransform& localTransform) const
{
  try
  {
    m_tfListener->lookupTransform(targetFrame, m_baseFrameId, t, localTransform);
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Failed to lookup local transform (%s)", e.what());
    return false;
  }

  return true;
}

bool MotionModel::getLastOdomPose(tf::Stamped<tf::Pose>& lastOdomPose) const{
  if (m_firstOdometryReceived){
    lastOdomPose = m_lastOdomPose;
    return true;
  } else{
    return false;
  }

}

}


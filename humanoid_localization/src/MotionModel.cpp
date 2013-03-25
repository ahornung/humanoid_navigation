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
  m_odomNoise2D = Eigen::Matrix3d::Zero();
  // noise affecting x direction (sq. / variance)
  nh->param("motion_noise/xx", m_odomNoise2D(0,0), 0.01);
  nh->param("motion_noise/xy", m_odomNoise2D(0,1), 0.01);
  nh->param("motion_noise/xt", m_odomNoise2D(0,2), 0.0001);
  // noise affecting y direction (sq. / variance)
  nh->param("motion_noise/yx", m_odomNoise2D(1,0), 0.01);
  nh->param("motion_noise/yy", m_odomNoise2D(1,1), 0.01);
  nh->param("motion_noise/yt", m_odomNoise2D(1,2), 0.0001);
  // noise affecting orientation (sq. / variance)
  nh->param("motion_noise/tx", m_odomNoise2D(2,0), 0.5);
  nh->param("motion_noise/ty", m_odomNoise2D(2,1), 0.5);
  nh->param("motion_noise/tt", m_odomNoise2D(2,2), 0.01);

  // std. devs for z, roll & pitch (depend on amount of transl.)
  nh->param("motion_noise/z", m_odomNoiseZ, 0.01);
  nh->param("motion_noise/roll", m_odomNoiseRoll, 0.05);
  nh->param("motion_noise/pitch", m_odomNoisePitch, 0.1);

  // old parameters, warn that renamed:
  if (nh->hasParam("motion_noise/x"))
    ROS_WARN("Parameter motion_noise/x is no longer used, use variances motion_noise/[xx|xy|xt] instead");

  if (nh->hasParam("motion_noise/y"))
    ROS_WARN("Parameter motion_noise/y is no longer used, use variances motion_noise/[yx|yy|yt] instead");

  if (nh->hasParam("motion_noise/yaw"))
    ROS_WARN("Parameter motion_noise/yaw is no longer used, use variances motion_noise/[tx|ty|tt] instead");

  // odometry calibration (systematic drift correction)
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


  reset();

}

MotionModel::~MotionModel() {

}


tf::Transform MotionModel::odomTransformNoise(const tf::Transform& odomTransform){
  // vectors (x,y,theta) in 2D for squared motion and variance
  Eigen::Vector3d motion2D_sq, motion_variance;
  double yaw = tf::getYaw(odomTransform.getRotation());
  motion2D_sq(0) = odomTransform.getOrigin().x() * odomTransform.getOrigin().x();
  motion2D_sq(1) = odomTransform.getOrigin().y() * odomTransform.getOrigin().y();
  motion2D_sq(2) = yaw * yaw;

  motion_variance = m_odomNoise2D * motion2D_sq;

  // use std.normal dev as basis:
  // X is normally distributed (mean 0, dev 1)
  // => Y = aX + b is also normally distributed with mean b and deviation a

  // absolute amount of translation, used to scale noise in z, roll & pitch
  // (about 1-2cm for each update step)
  const double d = odomTransform.getOrigin().length();
  return tf::Transform(tf::createQuaternionFromRPY(
        m_rngNormal() * d * m_odomNoiseRoll,      // roll
        m_rngNormal() * d * m_odomNoisePitch,     // pitch
        m_rngNormal() * sqrt(motion_variance(2))),// yaw
      tf::Vector3(
        m_rngNormal() * sqrt(motion_variance(0)), // x
        m_rngNormal() * sqrt(motion_variance(1)), // y
        m_rngNormal() * d * m_odomNoiseZ));       // z
}

void MotionModel::reset(){
  m_firstOdometryReceived = false;
}

void MotionModel::applyOdomTransform(tf::Pose& particlePose, const tf::Transform& odomTransform){
  particlePose *= calibrateOdometry(odomTransform) * odomTransformNoise(odomTransform);
}

void MotionModel::applyOdomTransform(Particles& particles, const tf::Transform& odomTransform){
  const tf::Transform calibratedOdomTransform = calibrateOdometry(odomTransform);

  for (unsigned i=0; i < particles.size(); ++i){
    particles[i].pose *= calibratedOdomTransform * odomTransformNoise(odomTransform);
  }
}

bool MotionModel::applyOdomTransformTemporal(Particles& particles,const ros::Time& t, double dt){
  ros::WallTime startTime = ros::WallTime::now();

  // first see if default time is available
  tf::Transform odomTransform;
  if (!lookupOdomTransform(t, odomTransform))
    return false;

  tf::Transform timeSampledTransform;
  ros::Duration maxDuration;
  if (dt > 0.0){
    ros::Time maxTime;
    std::string errorString;
    m_tfListener->getLatestCommonTime(m_odomFrameId, m_baseFrameId, maxTime, &errorString);
    maxDuration = maxTime - t;
  }

  for (unsigned i=0; i < particles.size(); ++i){
    if (dt > 0.0){
      ros::Duration duration(m_rngUniform()*dt -dt/2.0);
      // TODO: time t is time of first measurement in scan!
      if (duration > maxDuration)
        duration = maxDuration;

      if (lookupOdomTransform(t + duration, timeSampledTransform))
        applyOdomTransform(particles[i].pose, timeSampledTransform);
      else{
        ROS_WARN("Could not lookup temporal odomTransform");
        applyOdomTransform(particles[i].pose, odomTransform);
      }
    } else{
      applyOdomTransform(particles[i].pose, odomTransform);
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


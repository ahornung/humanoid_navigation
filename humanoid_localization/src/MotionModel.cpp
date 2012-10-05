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

MotionModel::MotionModel(ros::NodeHandle* nh, EngineT* rngEngine, tf::TransformListener* tf)
: m_tfListener(tf),
  m_rngNormal(*rngEngine, NormalDistributionT(0.0, 1.0)),
  m_rngUniform(*rngEngine, UniformDistributionT(0.0, 1.0)),
  m_odomFrameId("odom"), m_baseFrameId("torso"),
  m_footprintFrameId("base_footprint"),
  m_firstOdometryReceived(false)

{

  nh->param("odom_frame_id", m_odomFrameId, m_odomFrameId);
  nh->param("base_frame_id", m_baseFrameId, m_baseFrameId);

  // motion model noise parameters:
  // covariance matrix, contains squared std.devs on diagonal
  Matrix6f motionNoise = Matrix6f::Zero();
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

  motionNoise.diagonal() = m_motionNoise.cwiseProduct( m_motionNoise);

  if (motionNoise.isZero()){
    m_motionNoiseL = Matrix6f::Zero();
  } else{
    m_motionNoiseL = motionNoise.llt().matrixL();
  }

  reset();

}

MotionModel::~MotionModel() {

}

void MotionModel::transformPose(tf::Pose& particlePose, const tf::Transform& odomTransform){

  // about ~ 0.01 / 0.02 m
  double dTrans = odomTransform.getOrigin().length();

  Vector6f poseNoise = m_motionNoise;
  for (unsigned i = 0; i < 6; ++i){
    poseNoise(i) *= dTrans * m_rngNormal();
  }
  //Vector6f poseCovNoise = m_motionNoiseL * poseNoise;


  tf::Transform odomTransformNoise(tf::createQuaternionFromRPY(poseNoise(3), poseNoise(4),poseNoise(5)),
                                   tf::Vector3(poseNoise(0), poseNoise(1), poseNoise(2)));

  // apply transform to particle:
  particlePose *= odomTransform * odomTransformNoise;

  // HACK for testing: fix roll & pitch at 0:
  //	double roll, pitch, yaw;
  //	btMatrix3x3 basis = particlePose.getBasis();
  //	basis.getRPY(roll, pitch, yaw);
  //	basis.setRPY(0.0, 0.0, yaw);
  //	particlePose.setBasis(basis);
  //

}

void MotionModel::reset(){
  m_firstOdometryReceived = false;
}

void MotionModel::applyOdomTransform(Particles& particles, const tf::Transform& odomTransform){
  for(Particles::iterator it = particles.begin(); it != particles.end(); ++it){
    transformPose(it->pose, odomTransform);
  }

}

bool MotionModel::applyOdomTransformTemporal(Particles& particles,const ros::Time& t, double dt){

  // the actual, non time-sampled transform:
  tf::Transform odomTransform;
  if (!lookupOdomTransform(t, odomTransform))
    return false;


  tf::Stamped<tf::Pose> timeSampledTransform;
  ros::Time maxTime;
  std::string errorString;
  m_tfListener->getLatestCommonTime(m_odomFrameId, m_baseFrameId, maxTime, &errorString);
  ros::Duration maxDuration = maxTime - t;
  for(Particles::iterator it = particles.begin(); it != particles.end(); ++it){
    if (dt > 0.0){
      ros::Duration duration(m_rngUniform()*dt -dt/2.0);
      // TODO: time t is time of first measurement in scan!
      if (duration > maxDuration)
        duration = maxDuration;

      lookupOdomTransform(t + duration, timeSampledTransform);
    } else{
      lookupOdomTransform(t, timeSampledTransform);

    }

    transformPose(it->pose, timeSampledTransform);
  }

  return true;
}

bool MotionModel::lookupOdomTransform(const ros::Time& t, tf::Transform& odomTransform) const{
  tf::Stamped<tf::Pose> odomPose;

  if (t <= m_lastOdomPose.stamp_){
    ROS_WARN("Looking up OdomTransform that is %f ms older than the last odomPose!",
             (m_lastOdomPose.stamp_ - t).toSec()/1000.0);
  }

  if (!lookupOdomPose(t, odomPose))
    return false;

  if (m_firstOdometryReceived){
    odomTransform = m_lastOdomPose.inverse() * odomPose;
  } else{
    odomTransform = tf::Transform(tf::createIdentityQuaternion(), btVector3(0,0,0));
  }


  return true;
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
#if ROS_VERSION_MINIMUM(1, 8, 7) // fuerte
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)), t, m_baseFrameId);
#else // electric or older
  tf::Stamped<tf::Pose> ident (btTransform(tf::createIdentityQuaternion(),
                                           btVector3(0,0,0)), t, m_baseFrameId);
#endif


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

bool MotionModel::lookupLaserTransform(const std::string& laserFrameId, const ros::Time& t,
                                       tf::StampedTransform& torsoToLaser) const
{
  try
  {
    m_tfListener->lookupTransform(m_baseFrameId, laserFrameId, t, torsoToLaser);
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Failed to lookup laser transform, skipping scan (%s)", e.what());
    return false;
  }

  return true;
}

bool MotionModel::lookupFootprintTf(const ros::Time& t, tf::StampedTransform& footprintToTorso) const
{
  try {
    m_tfListener->lookupTransform(m_footprintFrameId, m_baseFrameId, t, footprintToTorso);
  } catch (const tf::TransformException& e) {
    ROS_WARN("Failed to lookup pose height, skipping scan (%s)", e.what());
    return false;
  }

  return true;
}

bool MotionModel::lookupPoseHeight(const ros::Time& t, double& poseHeight) const{
  tf::StampedTransform tf;
  if (lookupFootprintTf(t, tf)){
    poseHeight = tf.getOrigin().getZ();
    return true;
  } else
    return false;
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


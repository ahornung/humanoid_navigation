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

#ifndef HUMANOID_LOCALIZATION_MOTIONMODEL_H_
#define HUMANOID_LOCALIZATION_MOTIONMODEL_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Cholesky>

#include <humanoid_localization/humanoid_localization_defs.h>

namespace humanoid_localization{
class MotionModel {
public:
  MotionModel(ros::NodeHandle* nh, EngineT* rngEngine, tf::TransformListener* tf);
  virtual ~MotionModel();
  void reset();
  /// look up the odom pose at a certain time through tf
  bool lookupOdomPose(const ros::Time& t, tf::Stamped<tf::Pose>& odomPose) const;

  /// computes the transform from the last odom pose to the one at time t
  /// odomTransform is an identity TF when there is no previous TF available
  bool lookupOdomTransform(const ros::Time& t, tf::Transform& odomTransform) const;

  /// lookup the transform from torso to the laser at time t
  bool lookupLaserTransform(const std::string& laserFrameId, const ros::Time& t, tf::StampedTransform& torsoToLaser) const;

  /// lookup the tf between the base and footprint frames
  bool lookupFootprintTf(const ros::Time& t, tf::StampedTransform& footprintToTorso) const;

  /// lookup the height of the torso, based on tf between the base and footprint frames
  bool lookupPoseHeight(const ros::Time& t, double& poseHeight) const;



  /// apply odomTransform to all particles (noisy)
  void applyOdomTransform(Particles& particles, const tf::Transform& odomTransform);

  /// apply odomTransform to all particles (noisy), with temporal sampling over the range of dt.
  /// Times are sampled in an interval +-dt/2 around t, iff dt > 0.0.
  bool applyOdomTransformTemporal(Particles& particles, const ros::Time& t, double dt);

  void storeOdomPose(const tf::Stamped<tf::Pose>& odomPose);

  /// get the last stored odomPose
  /// returns false when there is no valid previous pose stored
  bool getLastOdomPose(tf::Stamped<tf::Pose>& lastOdomPose) const;


  // aligns fixed-size members in memory
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void transformPose(tf::Pose& particlePose, const tf::Transform& odomTransform);

  tf::TransformListener* m_tfListener;

  NormalGeneratorT m_rngNormal; // standard normal-distributed noise
  UniformGeneratorT m_rngUniform;
  // parameters:
  Matrix6f m_motionNoiseL;
  Vector6f m_motionNoise;
  //NoiseParams m_motionNoise;


  std::string m_odomFrameId;
  std::string m_baseFrameId;
  std::string m_footprintFrameId;

  bool m_firstOdometryReceived;
  tf::Stamped<tf::Pose> m_lastOdomPose;


};
}

#endif

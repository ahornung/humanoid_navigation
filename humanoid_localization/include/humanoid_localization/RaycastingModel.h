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

#ifndef HUMANOID_LOCALIZATION_RAYCASTINGMODEL_H_
#define HUMANOID_LOCALIZATION_RAYCASTINGMODEL_H_

#include <limits>
#include<cmath>

#include <omp.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <humanoid_localization/ObservationModel.h>

#include <octomap/octomap.h>


namespace humanoid_localization{
class RaycastingModel : public ObservationModel {
public:
  RaycastingModel(ros::NodeHandle* nh, boost::shared_ptr<MapModel> mapModel, EngineT * rngEngine);
  virtual ~RaycastingModel();
  virtual void integrateMeasurement(Particles& particles, const PointCloud& pc, const std::vector<float>& ranges, float max_range, const tf::Transform& baseToSensor);

protected:
  bool getHeightError(const Particle& p, const tf::StampedTransform& footprintToBase, double& heightError) const;
  // laser parameters:
  double m_zHit;
  double m_zRand;
  double m_zShort;
  double m_zMax;
  double m_sigmaHit;
  double m_lambdaShort;

  bool m_filterPointCloudGround;
  double m_groundFilterDistance;
  double m_groundFilterAngle;
  double m_groundFilterPlaneDistance;
  int m_numFloorPoints;
  int m_numNonFloorPoints;
};

}

#endif

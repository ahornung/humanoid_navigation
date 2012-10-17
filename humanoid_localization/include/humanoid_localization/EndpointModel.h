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

#ifndef HUMANOID_LOCALIZATION_ENDPOINTMODEL_H_
#define HUMANOID_LOCALIZATION_ENDPOINTMODEL_H_

#include <limits>
#include<cmath>

#include <omp.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <humanoid_localization/ObservationModel.h>
#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <visualization_msgs/Marker.h>


namespace humanoid_localization{
class EndpointModel : public ObservationModel {
public:
  EndpointModel(ros::NodeHandle* nh, boost::shared_ptr<MapModel> mapModel, EngineT * rngEngine);
  virtual ~EndpointModel();
  virtual void integrateMeasurement(Particles& particles, const PointCloud& pc, const std::vector<float>& ranges, float max_range, const tf::Transform& baseToSensor);

  virtual void setMap(boost::shared_ptr<octomap::OcTree> map);

protected:
  bool getHeightError(const Particle& p, const tf::StampedTransform& footprintToBase, double& heightError) const;
  void initDistanceMap();
  double m_sigma;
  double m_maxObstacleDistance;
  boost::shared_ptr<DynamicEDTOctomap> m_distanceMap;
};

}

#endif

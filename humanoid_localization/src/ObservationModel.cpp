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

#include <humanoid_localization/ObservationModel.h>

using namespace std;
using namespace tf;

namespace humanoid_localization{

ObservationModel::ObservationModel(ros::NodeHandle* nh, boost::shared_ptr<MapModel> mapModel, EngineT* rngEngine )
: m_mapModel(mapModel),
  m_rngNormal(*rngEngine, NormalDistributionT(0.0, 1.0)),
  m_rngUniform(*rngEngine, UniformDistributionT(0.0, 1.0)),
  m_weightRoll(1.0), m_weightPitch(1.0), m_weightZ(1.0),
  m_sigmaZ(0.02), m_sigmaRoll(0.05), m_sigmaPitch(0.05)
{

  m_map = m_mapModel->getMap();

  nh->param("weight_factor_roll", m_weightRoll, m_weightRoll);
  nh->param("weight_factor_pitch", m_weightPitch, m_weightPitch);
  nh->param("weight_factor_z", m_weightZ, m_weightZ);
  nh->param("motion_sigma_z", m_sigmaZ, m_sigmaZ);
  nh->param("motion_sigma_roll", m_sigmaRoll, m_sigmaRoll);
  nh->param("motion_sigma_pitch", m_sigmaPitch, m_sigmaPitch);

  if (m_sigmaZ <= 0.0 || m_sigmaRoll <= 0.0 || m_sigmaPitch <= 0.0){
    ROS_ERROR("Sigma (std.dev) needs to be > 0 in ObservationModel");
  }
}

ObservationModel::~ObservationModel() {
}

void ObservationModel::integratePoseMeasurement(Particles& particles, double poseRoll, double posePitch, double poseHeight){
  // TODO cluster xy of particles => speedup
  for(Particles::iterator it = particles.begin(); it != particles.end(); ++it){
    // integrate IMU meas.:
    double roll, pitch, yaw;
    it->pose.getBasis().getRPY(roll, pitch, yaw);
    it->weight += m_weightRoll * logLikelihood(poseRoll - roll, m_sigmaRoll);
    it->weight += m_weightPitch * logLikelihood(posePitch - pitch, m_sigmaPitch);

    // integrate height measurement (z)
    tf::Vector3 xyz = it->pose.getOrigin();

    std::vector<double> heights;
    m_mapModel->getHeightlist(xyz.getX(), xyz.getY(), 0.6, heights);
    if (heights.size() > 0){
      // find nearest z-level:
      double minDist = std::numeric_limits<double>::max();
      for (unsigned i = 0; i< heights.size(); i++){
        double dist = std::abs((heights[i] + poseHeight) - xyz.getZ());
        if (dist < minDist)
          minDist = dist;

      }

      //double dist = pose.getOrigin().getZ() - it->pose.getOrigin().getZ();
      it->weight += m_weightZ * logLikelihood(minDist, m_sigmaZ);
      //it->weight += -1*( LOG_SQRT_2_PI) - log(m_sigmaZ) - ((minDist * minDist) / (2* m_sigmaZ * m_sigmaZ));
    }
  }

}

void ObservationModel::setMap(boost::shared_ptr<octomap::OcTree> map){
  m_map = map;
}

}

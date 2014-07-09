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

#include <humanoid_localization/MapModel.h>

namespace humanoid_localization{
MapModel::MapModel(ros::NodeHandle* nh)
: m_motionMeanZ(0.0),
  m_motionRangeZ(-1.0), m_motionRangeRoll(-1.0), m_motionRangePitch(-1.0),
  m_motionObstacleDist(0.2)
{

  // motion model max ranges (particles have to stay within range)
  nh->param("motion_mean_z", m_motionMeanZ, m_motionMeanZ);
  nh->param("motion_range_z", m_motionRangeZ, m_motionRangeZ);
  nh->param("motion_range_roll", m_motionRangeRoll, m_motionRangeRoll);
  nh->param("motion_range_pitch", m_motionRangePitch, m_motionRangePitch);
  // this is not correctly used at the moment:
  //nh->param("motion_occupied_radius", m_motionObstacleDist, m_motionObstacleDist);

}

MapModel::~MapModel(){

}


boost::shared_ptr<octomap::OcTree> MapModel::getMap() const{
  return m_map;
}

void MapModel::verifyPoses(Particles& particles){
  double minX, minY, minZ, maxX, maxY, maxZ;
  m_map->getMetricMin(minX, minY, minZ);
  m_map->getMetricMax(maxX, maxY, maxZ);

  // find min. particle weight:
  double minWeight = std::numeric_limits<double>::max();
  for (Particles::iterator it = particles.begin(); it != particles.end(); ++it) {
    if (it->weight < minWeight)
      minWeight = it->weight;

  }

  minWeight -= 200;

  unsigned numWall = 0;
  unsigned numOut = 0;
  unsigned numMotion = 0;


  // TODO possible speedup: cluster particles by grid voxels first?
  // iterate over samples, multi-threaded:
#pragma omp parallel for
  for (unsigned i = 0; i < particles.size(); ++i){

    octomap::point3d position(particles[i].pose.getOrigin().getX(),
                              particles[i].pose.getOrigin().getY(),
                              particles[i].pose.getOrigin().getZ());

    // see if outside of map bounds:
    if (position(0) < minX || position(0) > maxX
        ||	position(1) < minY || position(1) > maxY
        ||	position(2) < minZ || position(2) > maxZ)
    {
      particles[i].weight = minWeight;
#pragma omp atomic
      numOut++;
    } else {

      // see if occupied cell:
      if (this->isOccupied(position)){
        particles[i].weight = minWeight;
#pragma omp atomic
        numWall++;
      } else {
        // see if current pose is has a valid walking height:
        if (m_motionRangeZ >= 0.0 &&
            (std::abs(particles[i].pose.getOrigin().getZ() - getFloorHeight(particles[i].pose) - m_motionMeanZ)
              > m_motionRangeZ))
        {
          particles[i].weight = minWeight;
#pragma omp atomic
          numMotion++;
        } else if (m_motionRangePitch >= 0.0 || m_motionRangeRoll >= 0.0){

          double yaw, pitch, roll;
          particles[i].pose.getBasis().getRPY(roll, pitch, yaw);

          if ((m_motionRangePitch >= 0.0 && std::abs(pitch) > m_motionRangePitch)
              || (m_motionRangeRoll >= 0.0 && std::abs(roll) > m_motionRangeRoll))
          {
            particles[i].weight = minWeight;
#pragma omp atomic
            numMotion++;
          }
        }
      }
    }
  } // end loop over particles

  if (numWall > 0 || numOut > 0 || numMotion > 0){
    ROS_INFO("Particle weights minimized: %d out of map, %d in obstacles, %d out of motion range", numOut, numWall, numMotion);
  }

  if (numOut + numWall >= particles.size()){
    ROS_WARN("All particles are out of the valid map area or in obstacles!");
  }

}


void MapModel::initGlobal(Particles& particles, double z, double roll, double pitch,
                          const Vector6d& initNoise,
                          UniformGeneratorT& rngUniform, NormalGeneratorT& rngNormal){
  double sizeX,sizeY,sizeZ, minX, minY, minZ;
  m_map->getMetricSize(sizeX,sizeY,sizeZ);
  m_map->getMetricMin(minX, minY, minZ);

  double weight = 1.0 / particles.size();
  Particles::iterator it = particles.begin();
  while (true){
    if (it == particles.end())
      break;
    // obtain a pose hypothesis:
    double x = minX + sizeX * rngUniform();
    double y = minY + sizeY * rngUniform();
    std::vector<double> z_list;
    getHeightlist(x, y, 0.6,z_list);

    for (unsigned zIdx = 0; zIdx < z_list.size(); zIdx++){
      if (it == particles.end())
        break;

      // not needed => we already know that z contains valid poses
      // distance map: used distance from obstacles:
      //std::abs(node->getLogOdds()) < 0.1){
      //			if (!isOccupied(octomap::point3d(x, y, z[zIdx]))){

      it->pose.getOrigin().setX(x);
      it->pose.getOrigin().setY(y);
      // TODO: sample z, roll, pitch
      it->pose.getOrigin().setZ(z_list.at(zIdx) + z + rngNormal() * initNoise(2));
      double yaw = rngUniform() * 2 * M_PI  -M_PI;
      it->pose.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
      it->weight = weight;
      it++;
    }
  }

}

void MapModel::getHeightlist(double x, double y, double totalHeight, std::vector<double>& heights){
  double minX, minY, minZ, maxX, maxY, maxZ;
  m_map->getMetricMin(minX, minY, minZ);
  m_map->getMetricMax(maxX, maxY, maxZ);

  double res = m_map->getResolution();

  double z =  maxZ-res/2.0;
  double lastZ = z + res;

  while (z >= minZ){
    if (isOccupied(octomap::point3d(x, y, z))){
      if (lastZ - z >= totalHeight + res){
        heights.push_back(z+ res/2.0);
      }
      lastZ = z;
    }

    z -= res;
  }
}

bool MapModel::isOccupied(const octomap::point3d& position) const{
  octomap::OcTreeNode* mapNode = m_map->search(position);
  if (mapNode)
    return isOccupied(mapNode);
  else return false;
}


///////////////////////////////////////////////////////////////////////
// Distance Map (Endpoint Model)
///////////////////////////////////////////////////////////////////////

DistanceMap::DistanceMap(ros::NodeHandle* nh)
: MapModel(nh)
{
  ROS_ERROR("Distance map implementation is currently not supported");
  std::string mapFileName;
  nh->getParam("map_file_dist", mapFileName);

// TODO: use FileIO, try octree<float>
//  octomap::AbstractOcTree* tree = octomap_msgs::fullMsgDataToMap(resp.map.data);
//  if (tree){
//    ROS_INFO("Received tree type %s",tree->getTreeType().c_str());
//    //octree = dynamic_cast<OcTree*>(tree);
//  }

  octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(mapFileName));
  if (tree){
    m_map.reset(tree);
  }

  if (!m_map|| m_map->size() <= 1){
    ROS_ERROR("Distance map file loaded from \"%s\" is erroneous, exiting...", mapFileName.c_str());
    exit(-1);
  }
  double x,y,z;
  m_map->getMetricSize(x,y,z);
  ROS_INFO("Distance map initialized with %zd nodes (%.2f x %.2f x %.2f m)", m_map->size(), x,y,z);

}

DistanceMap::~DistanceMap(){

}

bool DistanceMap::isOccupied(octomap::OcTreeNode* node) const{
  if (std::abs(node->getLogOdds()) < m_map->getResolution())
    return true;
  else
    return false;
}

double DistanceMap::getFloorHeight(const tf::Transform& pose) const{
  // TODO:
  ROS_ERROR("DistanceMap::getFloorHeight not implemented yet!");

  return 0.0;
}

///////////////////////////////////////////////////////////////////////
// Occupancy Map (Raycasting)
///////////////////////////////////////////////////////////////////////


OccupancyMap::OccupancyMap(ros::NodeHandle* nh)
: MapModel(nh)
{
  std::string servname = "octomap_binary";
  ROS_INFO("Requesting the map from %s...", nh->resolveName(servname).c_str());
  octomap_msgs::GetOctomap::Request req;
  octomap_msgs::GetOctomap::Response resp;
  while(nh->ok() && !ros::service::call(servname, req, resp))
  {
    ROS_WARN("Request to %s failed; trying again...", nh->resolveName(servname).c_str());
    usleep(1000000);
  }


// Groovy:
#if ROS_VERSION_MINIMUM(1, 9, 0)
  m_map.reset(octomap_msgs::binaryMsgToMap(resp.map));
#else  // Fuerte:
  m_map.reset(octomap_msgs::binaryMsgDataToMap(resp.map.data));
#endif

  if (!m_map || m_map->size() <= 1){
    ROS_ERROR("Occupancy map is erroneous, exiting...");
    exit(-1);
  }
  double x,y,z;
  m_map->getMetricSize(x,y,z);
  ROS_INFO("Occupancy map initialized with %zd nodes (%.2f x %.2f x %.2f m), %f m res.", m_map->size(), x,y,z, m_map->getResolution());
  
  m_map->writeBinary("/tmp/octomap_loc");

}

OccupancyMap::~OccupancyMap(){

}

bool OccupancyMap::isOccupied(octomap::OcTreeNode* node) const{
  return m_map->isNodeOccupied(node);
}


double OccupancyMap::getFloorHeight(const tf::Transform& pose)const {
  octomap::point3d end;
  if (m_map->castRay(octomap::pointTfToOctomap(pose.getOrigin()), octomap::point3d(0.0, 0.0, -1.0), end, false)){
    // add resolution/2 so height is above voxel boundary:
    return end.z()+m_map->getResolution()/2.0;
  } else {
    ROS_WARN("getFloorHeight raycast did not succeed, using 0.0");
    return 0.0;
  }

}

}


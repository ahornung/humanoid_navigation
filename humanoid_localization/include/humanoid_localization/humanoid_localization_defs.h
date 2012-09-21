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

#ifndef HUMANOID_LOCALIZATION_HUMANOID_LOCALIZATION_DEFS_H_
#define HUMANOID_LOCALIZATION_HUMANOID_LOCALIZATION_DEFS_H_

#include <vector>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>

#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace humanoid_localization{

/// Particle consists of a pose and a weight
struct Particle{
  double weight;
  tf::Pose pose;
};

typedef std::vector<Particle> Particles;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


/// Boost RNG engine:
typedef boost::mt19937                 	EngineT;
/// Boost RNG distribution:
typedef boost::normal_distribution<>   	NormalDistributionT;
// Ugh! boost uniform_01 noise sucks: http://www.bnikolic.co.uk/blog/cpp-boost-uniform01.html
// => using uniform_real instead
typedef boost::uniform_real<>     		UniformDistributionT;
/// standard normal-distributed noise
typedef boost::variate_generator<EngineT&, NormalDistributionT>   NormalGeneratorT;
/// uniform noise in range [0,1)
typedef boost::variate_generator<EngineT&, UniformDistributionT>   UniformGeneratorT;


typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

}
#endif

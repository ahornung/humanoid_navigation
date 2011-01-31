// SVN $HeadURL$
// SVN $Id$

/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
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

#ifndef HELPER_H
#define HELPER_H

#include <gridmap_2d/GridMap2D.h>
#include <math.h>
#include <tf/transform_datatypes.h>

#define FLOAT_COMP_THR 0.00001
#define ANGLE_COMP_THR 0.087
#define MAPGRID_OCCUPENCY_THR 70.0


static const double TWO_PI = 2 * M_PI;
static const float  FLOAT_COMP_THR_SQR = FLOAT_COMP_THR * FLOAT_COMP_THR;


enum Leg
{

	NOLEG=0, RIGHT=1, LEFT=2

};


bool  close(float x, float y);
/**
 * Checking if a footstep (represented by its center and orientation (x, y, theta))
 * collides with an obstacle. The check is done by recursively testing if either
 * the outer circle around the foot, the inner circle of the foot or the are in
 * between has an appropriate distance to the nearest obstacle.
 *
 * @param x
 * @param y
 * @param theta
 * @param height of the foot
 * @param width of the foot
 * @param distanceMap containing distance information to the nearest obstacle
 * @return true if the footstep collides with an obstacle
 */
bool  collisionCheck(float x,
                     float y,
                     float theta,
                     float height,
                     float width,
                     int accuracy,
                     const GridMap2D& distanceMap);
float euclideanDistance(float x1, float x2, float y1, float y2, int roundingThreshold);
/**
 * Calculate the footstep necessary to reach 'to' from 'from'.
 *
 * @param supportLeg
 * @param footSeparation
 * @param from
 * @param to
 * @param footstep
 */
void  getFootstep(Leg supportLeg,
                  float footSeparation,
                  const tf::Transform& from,
                  const tf::Transform& to,
                  tf::Transform* footstep);
float round(float f, short decimal);

#endif

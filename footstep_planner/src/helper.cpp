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

#include <footstep_planner/helper.h>


/*
 * bool getFootstep(Leg supportLeg, float footSeparation, const tf::Transform& from,
 *                  const tf::Transform& to, tf::Transform* footstep)
 * --------------------------
 * Calculate the footstep necessary to reach 'to' from 'from'.
 */
void
getFootstep(Leg supportLeg, float footSeparation, const tf::Transform& from, const tf::Transform& to, tf::Transform* footstep)
{

	bool xInRange;
	bool yInRange;
	bool thetaInRange;

	tf::Transform current;
	tf::Vector3 translation;

	xInRange = false;
	yInRange = false;
	thetaInRange = false;

	tf::Quaternion rotation = to.getRotation().inverse() * from.getRotation();
	if (supportLeg == LEFT)
		translation = tf::Vector3(0, footSeparation/2, 0);
	else // supportLeg == RLEG
		translation = tf::Vector3(0, -footSeparation/2, 0);
	current = to * tf::Transform(rotation, translation);
	current *= tf::Transform(tf::createQuaternionFromYaw(0.0), translation);
	current = from.inverse() * current;

	footstep->setOrigin(current.getOrigin());
	footstep->setRotation(rotation.inverse());

}

// new version of above, using GridMap2D

bool
collisionCheck(float x, float y, float theta, float height, float width, int accuracy,
               const GridMap2D& distanceMap)
{

	float d = distanceMap.distanceMapAt(x, y);
	if (d < 0.0) // if out of bounds => collision
		return true;
	d -= distanceMap.getResolution();

	float r_o = sqrt(width*width + height*height) / 2.0;

	if (d >= r_o)
		return false;
	else if (accuracy == 0)
		return false;

	float h_half = height / 2.0f;
	float w_half = width / 2.0f;
	float r_i = std::min(w_half, h_half);

	if (d <= r_i)
		return true;
	else if (accuracy == 1)
		return true;

	float h_new;
	float w_new;
	float delta_x;
	float delta_y;
	if (width < height)
	{
		float h_clear = sqrt(d*d - w_half*w_half);
		h_new = h_half - h_clear;
		w_new = width;
		delta_x = h_clear + h_new/2.0;
		delta_y = 0.0;
	}
	else // footWidth >= footHeight
	{
		float w_clear = sqrt(d*d - h_half*h_half);
		h_new = height;
		w_new = w_half - w_clear;
		delta_x = 0.0;
		delta_y = w_clear + w_new/2.0;
	}
	float thetaCos = cos(theta);
	float thetaSin = sin(theta);
	float x_shift = thetaCos*delta_x - thetaSin*delta_y;
	float y_shift = thetaSin*delta_x + thetaCos*delta_y;

	return (collisionCheck(x+x_shift, y+y_shift, theta, h_new, w_new, accuracy, distanceMap) ||
	        collisionCheck(x-x_shift, y-y_shift, theta, h_new, w_new, accuracy, distanceMap));

}


/*
 * bool close(float x, float y)
 * --------------------------
 * Float equality check.
 */
bool
close(float x, float y)
{

	if (isinf(x) && isinf(y))
		return true;
	return (fabs(x-y) < FLOAT_COMP_THR);

}


/*
 * float round(float f, short decimal)
 * --------------------------
 * Float rounding.
 */
float
round(float f, short decimal)
{

	f = f*(pow(10,decimal));
	if (f >= 0)
		f = floor(f+0.5);
	else
		f = floor(f-0.5);
	f = f/(pow(10,decimal));

	return f;

}


/*
 * float euclideanDistance(float x1, float x2, float y1, float y2, int roundingThreshold)
 * --------------------------
 * Calculates the euclidean distance between (x1,x2) and (y1,y2).
 */
float
euclideanDistance(float x1, float x2, float y1, float y2, int roundingThreshold)
{

	float x = round(x1, roundingThreshold) - round(y1, roundingThreshold);
	float y = round(x2, roundingThreshold) - round(y2, roundingThreshold);

	return sqrt(x*x + y*y);

}


// SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_navigation/footstep_planner/src/Dstar.cpp $
// SVN $Id: Dstar.cpp 1168 2011-03-30 03:18:02Z hornunga@informatik.uni-freiburg.de $

/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 * D* Lite (Koenig et al. 2002) partly based on the implementation
 * by J. Neufeld (http://code.google.com/p/dstarlite/)
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

#include <footstep_planner/State.h>


namespace footstep_planner
{

	/*
	 * #############################################################################
	 * ### class State
	 * #############################################################################
	 */

	State::State()
	{

		ivGlobalX = 0;
		ivGlobalY = 0;
		ivGlobalTheta = 0;
		ivLeg = NOLEG;
		ivKey = key(INFINITY, INFINITY);

	}


	State::State(float x, float y, float theta, Leg leg)
	{

		ivGlobalX = x;
		ivGlobalY = y;
		ivGlobalTheta = theta;
		ivLeg = leg;
		ivKey = key(INFINITY, INFINITY);

	}


	State::~State()
	{}


	bool
	State::operator == (const State &s2)
	const
	{

		if (ivLeg != s2.getLeg())
			return false;

		float diffAngle = std::abs(ivGlobalTheta - s2.getTheta());
		if (diffAngle > M_PI)
			diffAngle = TWO_PI - diffAngle;

		float diffX = ivGlobalX - s2.getX();
		float diffY = ivGlobalY - s2.getY();

		return (((diffX*diffX + diffY*diffY) < FLOAT_COMP_THR_SQR) && (diffAngle < ANGLE_COMP_THR));

	}


	bool
	State::operator != (const State &s2)
	const
	{

		return !(*this == s2);

	}


	bool
	State::operator > (const State &s2)
	const
	{

		const key s2Key = s2.getKey();
		if (ivKey.first-FLOAT_COMP_THR > s2Key.first)
			return true;
		else if (ivKey.first < s2Key.first-FLOAT_COMP_THR)
			return false;
		return ivKey.second > s2Key.second;

	}


	bool
	State::operator <= (const State &s2)
	const
	{

		const key s2Key = s2.getKey();
		if (ivKey.first < s2Key.first)
			return true;
		else if (ivKey.first > s2Key.first)
			return false;
		return ivKey.second < s2Key.second + FLOAT_COMP_THR;

	}


	bool
	State::operator < (const State &s2)
	const
	{

		const key s2Key = s2.getKey();
		if (ivKey.first + FLOAT_COMP_THR < s2Key.first)
			return true;
		else if (ivKey.first - FLOAT_COMP_THR > s2Key.first)
			return false;
		return ivKey.second < s2Key.second;

	}


	size_t
	State::state_hash::operator ()(const State &s)
	const
	{

		return 124 * s.getX() + 34245*s.getY() + 1306*s.getTheta() + s.getLeg();

	//	float x = round(s.getX(), Dstar::cvRoundingThreshold);
	//	float y = round(s.getY(), Dstar::cvRoundingThreshold);
	//	float angle = round(s.getTheta(), Dstar::cvRoundingThreshold);
	//	return 124 * x + 34245 * y + 1306 * angle + s.getLeg();

	}

} // end of namespace

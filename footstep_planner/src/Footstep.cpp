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

#include <footstep_planner/Footstep.h>


namespace footstep_planner
{

	/*
	 * #########################################################################
	 * ### class Footstep
	 * #########################################################################
	 */

	Footstep::Footstep()
	{

		ivX = 0;
		ivY = 0;
		ivTheta = 0;
		ivLeg = NOLEG;

	}


	Footstep::Footstep(float x, float y, float theta, Leg leg)
		: ivX(x), ivY(y), ivTheta(theta), ivLeg(leg)
	{}


	Footstep::~Footstep()
	{}


	void
	Footstep::performMeOnThisState(const State& current, State* successor, float footSeparation)
	const
	{

		float globalX = current.getX();
		float globalY = current.getY();
		float globalTheta = current.getTheta();

		float footSeparationHalf = footSeparation/2.0;

		float thetaCos = cos(globalTheta);
		float thetaSin = sin(globalTheta);

		if (current.getLeg() == RIGHT)
		{
			float xShift = thetaCos*ivX - thetaSin*(ivY+footSeparationHalf);
			float yShift = thetaSin*ivX + thetaCos*(ivY+footSeparationHalf);
			globalX += xShift;
			globalY += yShift;
			globalTheta += ivTheta; // NOTE: new globalTheta

			xShift = -sin(globalTheta) * footSeparationHalf;
			yShift =  cos(globalTheta) * footSeparationHalf;
			globalX += xShift;
			globalY += yShift;

			successor->setLeg(LEFT);
		}
		else // leg == LEFT
		{
			float xShift = thetaCos*ivX + thetaSin*(ivY+footSeparationHalf);
			float yShift = thetaSin*ivX - thetaCos*(ivY+footSeparationHalf);
			globalX += xShift;
			globalY += yShift;
			globalTheta -= ivTheta; // NOTE: new globalTheta

			xShift =  sin(globalTheta) * footSeparationHalf;
			yShift = -cos(globalTheta) * footSeparationHalf;
			globalX += xShift;
			globalY += yShift;

			successor->setLeg(RIGHT);
		}

		successor->setX(globalX);
		successor->setY(globalY);
		successor->setTheta(globalTheta);

	}


	void
	Footstep::revertMeOnThisState(const State& current, State* predecessor, float footSeparation)
	const
	{

		float globalX = current.getX();
		float globalY = current.getY();
		float globalTheta = current.getTheta();

		float footSeparationHalf = footSeparation/2.0;

		if (current.getLeg() == LEFT)
		{
			float xShift =  sin(globalTheta)*(footSeparationHalf);
			float yShift = -cos(globalTheta)*(footSeparationHalf);
			globalX += xShift;
			globalY += yShift;
			globalTheta -= ivTheta;

			float thetaCos = cos(globalTheta);
			float thetaSin = sin(globalTheta);
			xShift = -thetaCos*ivX + thetaSin*(ivY+footSeparationHalf);
			yShift = -thetaSin*ivX - thetaCos*(ivY+footSeparationHalf);
			globalX += xShift;
			globalY += yShift;

			predecessor->setLeg(RIGHT);
		}
		else // leg == RIGHT
		{
			float xShift = -sin(globalTheta)*(footSeparationHalf);
			float yShift =  cos(globalTheta)*(footSeparationHalf);
			globalX += xShift;
			globalY += yShift;
			globalTheta += ivTheta;

			float thetaCos = cos(globalTheta);
			float thetaSin = sin(globalTheta);
			xShift = -thetaCos*ivX - thetaSin*(ivY+footSeparationHalf);
			yShift = -thetaSin*ivX + thetaCos*(ivY+footSeparationHalf);
			globalX += xShift;
			globalY += yShift;

			predecessor ->setLeg(LEFT);
		}

		predecessor->setX(globalX);
		predecessor->setY(globalY);
		predecessor->setTheta(globalTheta);

	}

} // end of namespace

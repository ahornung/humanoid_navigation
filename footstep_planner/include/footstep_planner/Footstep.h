// SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_navigation/footstep_planner/include/footstep_planner/Dstar.h $
// SVN $Id: Dstar.h 1168 2011-03-30 03:18:02Z hornunga@informatik.uni-freiburg.de $

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

#ifndef FOOTSTEP_H
#define FOOTSTEP_H

#include <footstep_planner/State.h>

namespace footstep_planner
{

	/**
	 * @brief A class representing a footstep (i.e. a translation and rotation
	 * of a specific foot) that can be performed by a humanoid robot.
	 */
	class Footstep
	{

	public:

		Footstep();
		Footstep(float x, float y, float theta, Leg leg = NOLEG);
		virtual ~Footstep();

		/**
		 * @brief Performs the footstep within the current state to receive the
		 * successor state.
		 *
		 * @param current state of the support leg
		 * @param successor state of the opposite leg (with respect to the support
		 * leg) after performing the footstep
		 * @param footSeparation standard separation of both feet in initial position
		 */
		void performMeOnThisState(const State& current, State* successor, float footSeparation) const;

		/**
		 * @brief Reverts the footstep within the current state to receive the
		 * predecessor state.
		 *
		 * @param current state of the support leg
		 * @param predecessor state of the opposite leg (with respect to the support
		 * leg) after reverting the footstep
		 * @param footSeparation standard separation of both feet in initial position
		 */
		void revertMeOnThisState(const State& current, State* predecessor, float footSeparation) const;


	private:

		/// translation on the x axis (w.r. to the support leg)
		float ivX;
		/// translation on the y axis (w.r. to the support leg)
		float ivY;
		/// rotation (w.r. to the support leg)
		float ivTheta;
		/// support leg
		Leg   ivLeg;

	};

} // end of namespace


#endif

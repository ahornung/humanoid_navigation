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

#ifndef HUMANOID_SBPL_FOOTSTEP_H_
#define HUMANOID_SBPL_FOOTSTEP_H_

#include <footstep_planner/PlanningState.h>


namespace footstep_planner
{
	/**
	 * @brief A class representing a footstep (i.e. a translation and rotation
	 * of a specific foot) that can be performed by a humanoid robot.
	 */
	class Footstep
	{
	public:
		Footstep(double x, double y, double theta,
	             double cell_size, int num_angle_bins, int max_hash_size,
	             double foot_separation);
		~Footstep();

		PlanningState performMeOnThisState(const PlanningState& current) const;

        PlanningState revertMeOnThisState(const PlanningState& current) const;

        void updateNumAngleBins(int num);

	private:
        typedef std::pair<double, double> shift_vector;

		void init();

        void calculateForwardStep(Leg leg, double global_theta,
                                  double* footstep_x, double* footstep_y) const;
        void calculateBackwardStep(Leg leg, double global_theta,
                                   double* footstep_x, double* footstep_y) const;

		int ivX;
		int ivY;
		int ivTheta;

		double ivCellSize;

		int ivNumAngleBins;
		int ivMaxHashSize;

		double ivFootSeparation;

        std::vector<shift_vector> ivSuccessorLeft;
        std::vector<shift_vector> ivSuccessorRight;
        std::vector<shift_vector> ivPredecessorLeft;
        std::vector<shift_vector> ivPredecessorRight;
	};
} // end of namespace

#endif  // HUMANOID_SBPL_FOOTSTEP_H_

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

#ifndef HUMANOID_SBPL_STATE_H_
#define HUMANOID_SBPL_STATE_H_

#include <footstep_planner/helper.h>


namespace footstep_planner
{
	/**
	 * @brief Representation of the robot's pose (i.e. location and orientation)
	 * in the search space. More precisely the state points to the robot's
	 * supporting leg.
	 */
	class PlanningState
	{
	public:
		/**
		 * @brief x, y and theta represent the global position and orientation
		 * of the robot's support leg defined by leg.
		 */
	    PlanningState(double x, double y, double theta, Leg leg,
                      double cell_size, int num_angle_bins, int max_hash_size);

	    /**
	     * @brief x, y and theta as discrete bin values (as used internally by
	     * the planner)
	     */
	    PlanningState(int x, int y, int theta, Leg leg,
                      double cell_size, int num_angle_bins, int max_hash_size);

	    /// Create a PlanningState from a State
        PlanningState(const State& s, double cell_size, int num_angle_bins,
                      int max_hash_size);

		PlanningState(const PlanningState& s);

		~PlanningState();

		bool operator ==(const PlanningState& s2) const;
		bool operator !=(const PlanningState& s2) const;

		void setId(unsigned int id) { ivId = id; };

		Leg getLeg() const { return ivLeg; };
		int getTheta() const { return ivTheta; };
		int getX() const { return ivX; };
		int getY() const { return ivY; };

		unsigned int getHashTag() const { return ivHashTag; };
		// TODO: move to helper?
		unsigned int calculateHashTag(int max_hash_size) const;

		int getId() const { return ivId; };

	private:
		int ivX;
		int ivY;
		int ivTheta;

		Leg	ivLeg;

	    int ivId;

		unsigned int ivHashTag;
	};
}
#endif  /* HUMANOID_SBPL_STATE_H_ */

// SVN $HeadURL$
// SVN $Id$

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

#include <footstep_planner/PlanningState.h>

namespace footstep_planner
{
	/*
	 * #########################################################################
	 * ### class State
	 * #########################################################################
	 */

    PlanningState::PlanningState(double x, double y, double theta, Leg leg,
                                 double cell_size, int num_angle_bins,
                                 int max_hash_size)
        : ivX(cont_2_disc(x, cell_size)),
          ivY(cont_2_disc(y, cell_size)),
          ivTheta(angle_cont_2_disc(theta, num_angle_bins)),
          ivLeg(leg),
          ivContX(x),
          ivContY(y),
          ivContTheta(theta),
          ivId(-1),
          ivHashTag(calculateHashTag(max_hash_size))
    {}


    PlanningState::PlanningState(const State& s, double cell_size,
                                 int num_angle_bins, int max_hash_size)
        : ivX(cont_2_disc(s.x, cell_size)),
          ivY(cont_2_disc(s.y, cell_size)),
          ivTheta(angle_cont_2_disc(s.theta, num_angle_bins)),
          ivLeg(s.leg),
          ivContX(s.x),
          ivContY(s.y),
          ivContTheta(s.theta),
          ivId(-1),
          ivHashTag(calculateHashTag(max_hash_size))
    {}


	PlanningState::PlanningState(const PlanningState& s)
	    : ivX(s.getX()),
	      ivY(s.getY()),
	      ivTheta(s.getTheta()),
	      ivLeg(s.getLeg()),
          ivContX(s.getContX()),
          ivContY(s.getContY()),
          ivContTheta(s.getContTheta()),
	      ivId(s.getId()),
	      ivHashTag(s.getHashTag())
	{}


	PlanningState::~PlanningState()
	{}


    unsigned int
    PlanningState::calculateHashTag(int max_hash_size)
    const
    {
        return int_hash((int_hash(ivX) << 3) + (int_hash(ivY) << 2) +
                        (int_hash(ivTheta) << 1) + (int_hash(ivLeg)))
                        % max_hash_size;
    }


	bool
	PlanningState::operator ==(const PlanningState &s2)
	const
	{
	    if (ivHashTag != s2.getHashTag())
            return false;

	    return (ivX == s2.getX() &&
                ivY == s2.getY() &&
                ivTheta == s2.getTheta() &&
                ivLeg == s2.getLeg());
	}


	bool
	PlanningState::operator !=(const PlanningState &s2)
	const
	{
		return !(*this == s2);
	}
} // end of namespace

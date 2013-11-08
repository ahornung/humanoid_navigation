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
PlanningState::PlanningState(double x, double y, double theta, Leg leg,
                             double cell_size, int num_angle_bins,
                             int max_hash_size)
: ivX(state_2_cell(x, cell_size)),
  ivY(state_2_cell(y, cell_size)),
  ivTheta(angle_state_2_cell(theta, num_angle_bins)),
  ivLeg(leg),
  ivId(-1),
  ivHashTag(calc_hash_tag(ivX, ivY, ivTheta, ivLeg, max_hash_size))
{}


PlanningState::PlanningState(int x, int y, int theta, Leg leg,
                             int max_hash_size)
:  ivX(x),
   ivY(y),
   ivTheta(theta),
   ivLeg(leg),
   ivId(-1),
   ivHashTag(calc_hash_tag(ivX, ivY, ivTheta, ivLeg, max_hash_size))
{}


PlanningState::PlanningState(const State& s, double cell_size,
                             int num_angle_bins, int max_hash_size)
: ivX(state_2_cell(s.getX(), cell_size)),
  ivY(state_2_cell(s.getY(), cell_size)),
  ivTheta(angle_state_2_cell(s.getTheta(), num_angle_bins)),
  ivLeg(s.getLeg()),
  ivId(-1),
  ivHashTag(calc_hash_tag(ivX, ivY, ivTheta, ivLeg, max_hash_size))
{}


PlanningState::PlanningState(const PlanningState& s)
: ivX(s.getX()),
  ivY(s.getY()),
  ivTheta(s.getTheta()),
  ivLeg(s.getLeg()),
  ivId(s.getId()),
  ivHashTag(s.getHashTag())
{}


PlanningState::~PlanningState()
{}


bool
PlanningState::operator ==(const PlanningState& s2)
const
{
  // First test the hash tag. If they differ, the states are definitely
  // different.
  if (ivHashTag != s2.getHashTag())
    return false;

  return (ivX == s2.getX() && ivY == s2.getY() &&
    ivTheta == s2.getTheta() && ivLeg == s2.getLeg());
}


bool
PlanningState::operator !=(const PlanningState& s2)
const
{
  return ivHashTag != s2.getHashTag();
}


State
PlanningState::getState(double cell_size, int num_angle_bins)
const
{
  return State(cell_2_state(ivX, cell_size),
               cell_2_state(ivY, cell_size),
               angles::normalize_angle(
                   angle_cell_2_state(ivTheta, num_angle_bins)),
                   ivLeg);
}
} // end of namespace

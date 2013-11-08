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

#include <footstep_planner/State.h>


namespace footstep_planner
{
State::State()
: ivX(0.0), ivY(0.0), ivTheta(0.0), ivLeg(NOLEG)
{}


State::State(double x, double y, double theta, Leg leg)
: ivX(x), ivY(y), ivTheta(theta), ivLeg(leg)
{}


State::~State()
{}


bool
State::operator ==(const State& s2)
const
{
  return (fabs(ivX - s2.getX()) < FLOAT_CMP_THR &&
      fabs(ivY - s2.getY()) < FLOAT_CMP_THR &&
      fabs(angles::shortest_angular_distance(ivTheta, s2.getTheta()))
  < FLOAT_CMP_THR &&
  ivLeg == s2.getLeg());
}


bool
State::operator !=(const State& s2)
const
{
  return not (*this == s2);
}
} // end of namespace

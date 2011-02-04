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

#ifndef STATE_H_
#define STATE_H_

#include <footstep_planner/helper.h>

namespace footstep_planner{

	/**
	 * @brief Representation of the robot's pose (i.e. location and orientation)
	 * in the search space. More precisely the state points to the robot's
	 * supporting leg.
	 *
	 */
	class State
	{

	public:

		typedef std::pair<float,float> key;

		State();
		/**
		 * @brief x, y and theta represent the global position and orientation
		 * of the robot's support leg defined by leg.
		 */
		State(float x, float y, float theta, Leg leg);
		virtual ~State();

		bool operator ==(const State& s2) const;
		bool operator !=(const State& s2) const;
		bool operator  >(const State& s2) const;
		bool operator <=(const State& s2) const;
		bool operator  <(const State& s2) const;

		void setLeg(Leg leg) { ivLeg = leg; };
		void setTheta(float theta) { ivGlobalTheta = theta; };
		void setX(float x) { ivGlobalX = x; };
		void setY(float y) { ivGlobalY = y; };

		void setKey(float f, float g) { ivKey = key(f, g); };
		void setKey(key key) { ivKey = key; };

		key   getKey() const { return ivKey; };
		Leg   getLeg() const { return ivLeg; };
		float getTheta() const { return ivGlobalTheta; };
		float getX() const { return ivGlobalX; };
		float getY() const { return ivGlobalY; };



	private:

		float ivGlobalX;
		float ivGlobalY;
		float ivGlobalTheta;
		/// support leg
		Leg	  ivLeg;

		/**
		 * @brief key.first = min{g(s), rhs(s)} + h(s_start, s) + km,
		 * key.second = min{g(s), rhs(s)}
		 */
		key ivKey;


	public:

		/**
		 * @brief The states currently known g and rhs value. Stored in the state
		 * hash.
		 */
		struct state_info
		{

			float g;
			float rhs;

		};

		/**
		 * @brief Hash function of the state hash.
		 */
		struct state_hash
		{

			size_t operator ()(const State &s) const;

		};

	};
}


#endif /* STATE_H_ */

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

#ifndef HUMANOID_SBPL_HELPER_H_
#define HUMANOID_SBPL_HELPER_H_

#define DEBUG_HASH 0
#define DEBUG_TIME 0


#include <gridmap_2d/GridMap2D.h>
#include <angles/angles.h>

#include <math.h>


namespace footstep_planner
{
	static const double TWO_PI = 2 * M_PI;

    enum Leg { RIGHT=0, LEFT=1, NOLEG=2 };


    struct State
    {
        double x;
        double y;
        double theta;
        Leg leg;

        bool operator ()(const State& a, const State& b);
    };


    /// euclidean distance between two coordinates
    inline double euclidean_distance(int x1, int y1, int x2, int y2)
    {
        return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    }


    /// euclidean distance between two coordinates
    inline double euclidean_distance(double x1, double y1, double x2, double y2)
    {
        return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    }


    /// returns the distance of two neighbored cell (with cell size 1)
    inline double grid_cost(int x1, int y1, int x2, int y2, float cell_size)
    {
        int x = abs(x1 - x2);
        int y = abs(y1 - y2);

        float cost = 1;
        if (x + y > 1)
            cost = M_SQRT2;

        return cost * cell_size;
    }

    inline double eight_way_cost(int x1, int y1, int x2, int y2, float cell_size)
    {
        double min = abs(x1 - x2);
        double max = abs(y1 - y2);
        if (min > max)
        {
            double temp = min;
            min = max;
            max = temp;
        }
        return ((M_SQRT2-1.0)*min + max) * cell_size;
    }


	inline int angle_cont_2_disc(double angle, int angle_bin_num)
	{
        double bin_size_half = TWO_PI / angle_bin_num / 2;
        return (int)(angles::normalize_angle_positive(angle + bin_size_half) /
                     TWO_PI * angle_bin_num);
	}


	inline double angle_disc_2_cont(int angle, int angle_bin_num)
	{
        double bin_size = TWO_PI / angle_bin_num;
        return angle * bin_size;
	}


	inline int cont_2_disc(double value, double cell_size)
	{
		return (int)floor(value / cell_size);
	}


	inline double disc_2_cont(int value, double cell_size)
	{
	    return (double(value) * cell_size + cell_size/2);
	}


	inline unsigned int int_hash(int key)
	{
        key += (key << 12);
        key ^= (key >> 22);
        key += (key << 4);
        key ^= (key >> 9);
        key += (key << 10);
        key ^= (key >> 2);
        key += (key << 7);
        key ^= (key >> 12);
        return key;
	}


    void get_footstep(Leg support_leg, double foot_separation,
                      double from_x, double from_y, double from_theta,
                      double to_x, double to_y, double to_theta,
                      double& footstep_x, double& footstep_y,
                      double& footstep_theta);

	/**
	 * Checking if a footstep (represented by its center and orientation (x, y, theta))
	 * collides with an obstacle. The check is done by recursively testing if either
	 * the outer circle around the foot, the inner circle of the foot or the are in
	 * between has an appropriate distance to the nearest obstacle.
	 *
	 * @param x
	 * @param y
	 * @param theta
	 * @param height of the foot
	 * @param width of the foot
	 * @param distanceMap containing distance information to the nearest obstacle
	 * @return true if the footstep collides with an obstacle
	 */
	bool collision_check(double x, double y, double theta,
                         double height, double width, int accuracy,
						 const GridMap2D& distanceMap);
}

#endif  /* HUMANOID_SBPL_HELPER_H_ */

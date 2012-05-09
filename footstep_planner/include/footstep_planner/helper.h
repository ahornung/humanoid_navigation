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
#include <tf/tf.h>

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


    /// euclidean distance between two integer coordinates (cells)
    inline double euclidean_distance(int x1, int y1, int x2, int y2)
    {
        return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    }

    /// squared euclidean distance between two integer coordinates (cells)
    inline double euclidean_distance_sq(int x1, int y1, int x2, int y2)
    {
        return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
    }


    /// euclidean distance between two coordinates
    inline double euclidean_distance(double x1, double y1, double x2, double y2)
    {
        return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    }

    /// squared euclidean distance between two coordinates
    inline double euclidean_distance_sq(double x1, double y1, double x2, double y2)
    {
        return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
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


	inline int angle_state_2_cell(double angle, int angle_bin_num)
	{
        double bin_size_half = TWO_PI / angle_bin_num / 2.0;
        return int(angles::normalize_angle_positive(angle + bin_size_half) /
                   TWO_PI * angle_bin_num);
	}


	inline double angle_cell_2_state(int angle, int angle_bin_num)
	{
        double bin_size = TWO_PI / angle_bin_num;
        return angle * bin_size;
	}


	// NOTE: this should only be used discretize a State to a PlanningState; use
	// cont_2_disc for lengths
	inline int state_2_cell(double value, double cell_size)
	{
		return int(floor(value / cell_size));
	}


    // NOTE: this should only be used to get a State from a discretized
	// PlanningState; use disc_2_cont for lengths
	inline double cell_2_state(int value, double cell_size)
	{
	    return (double(value) + 0.5) * cell_size;
	}


	// NOTE: use this only to discretize a length by a certain factor
	inline int discretize(double length, double factor)
	{
		return int(floor((length / factor) + 0.5));
	}


	// TODO: measure performance whether inline is useful here or not
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


	// TODO: measure performance whether inline is useful here or not
	inline unsigned int calc_hash_tag(int x, int y, int theta, int leg,
	                                  int max_hash_size)
	{
        return int_hash((int_hash(x) << 3) + (int_hash(y) << 2) +
                        (int_hash(theta) << 1) + (int_hash(leg)))
                        % max_hash_size;
	}


    void get_footstep(double from_x, double from_y, double from_theta,
    		          Leg from_leg, double to_x, double to_y, double to_theta,
                      double& footstep_x, double& footstep_y,
                      double& footstep_theta);

    /**
     * @param footstep_x
     * @param footstep_y
     * @param footstep_theta has to be in [-num_angle_bins/2..num_angle_bins/2)
     * @param max_footstep_x
     * @param max_footstep_y
     * @param max_footstep_theta has to be in [-num_angle_bins/2..num_angle_bins/2)
     * @param max_inv_footstep_x
     * @param max_inv_footstep_y
     * @param max_inv_footstep_theta has to be in [-num_angle_bins/2..num_angle_bins/2)
     * @param num_angle_bins
     * @return
     */
    bool performable(int footstep_x, int footstep_y, int footstep_theta,
                     int max_footstep_x, int max_footstep_y,
                     int max_footstep_theta,
                     int max_inv_footstep_x, int max_inv_footstep_y,
                     int max_inv_footstep_theta,
                     int num_angle_bins);

    /**
     * @param footstep_x
     * @param footstep_y
     * @param footstep_theta has to be in (-pi..+pi]
     * @param max_footstep_x
     * @param max_footstep_y
     * @param max_footstep_theta has to be in (-pi..+pi]
     * @param max_inv_footstep_x
     * @param max_inv_footstep_y
     * @param max_inv_footstep_theta has to be in (-pi..+pi]
     * @return
     */
    bool performable_cont(double footstep_x, double footstep_y,
                          double footstep_theta, double max_footstep_x,
                          double max_footstep_y, double max_footstep_theta,
                          double max_inv_footstep_x, double max_inv_footstep_y,
                          double max_inv_footstep_theta, double accuracy_x,
                          double accuracy_y, double accuracy_theta);

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

	/// Used to generate a State (continuous) from a PlanningState (discrete)
	void get_state(int x, int y, int theta, Leg leg, double cell_size,
	               int num_angle_bins, State* s);
}

#endif  /* HUMANOID_SBPL_HELPER_H_ */

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

#include <footstep_planner/helper.h>

namespace footstep_planner
{
    void get_footstep(Leg support_leg, double foot_separation,
                      double from_x, double from_y, double from_theta,
                      double to_x, double to_y, double to_theta,
                      double& footstep_x, double& footstep_y,
                      double& footstep_theta)
    {
        double diff_angle = angles::shortest_angular_distance(from_theta, to_theta);

        double foot_separation_half = foot_separation/2;

        double shift_to_x = -sin(to_theta) * foot_separation_half;
        double shift_to_y =  cos(to_theta) * foot_separation_half;
        double shift_from_x = -sin(from_theta) * foot_separation_half;
        double shift_from_y =  cos(from_theta) * foot_separation_half;
        if (support_leg == RIGHT)
        {
            to_x -= shift_to_x;
            to_y -= shift_to_y;

            to_x -= shift_from_x;
            to_y -= shift_from_y;

            diff_angle = -diff_angle;
        }
        else
        {
            to_x += shift_to_x;
            to_y += shift_to_y;

            to_x += shift_from_x;
            to_y += shift_from_y;
        }
        to_x -= from_x;
        to_y -= from_y;

        double from_theta_cos = cos(-from_theta);
        double from_theta_sin = sin(-from_theta);
        footstep_x = from_theta_cos*to_x - from_theta_sin*to_y;
        footstep_y = from_theta_sin*to_x + from_theta_cos*to_y;
        footstep_theta = diff_angle;
    }


    bool
    performable(int x, int y, int theta,
                int max_footstep_x, int max_footstep_y, int max_footstep_theta,
                int max_inv_footstep_x, int max_inv_footstep_y,
                int max_inv_footstep_theta,
                int num_angle_bins,
                Leg from_leg)
    {
        bool in_range_x = false;
        bool in_range_y = false;
        bool in_range_theta = false;

        // shift theta from range [0..num_angle_bins) to
        // [-num_angle_bins/2..num_angle_bins/2)
        theta -= num_angle_bins;

        if (x <= max_footstep_x && x >= -max_inv_footstep_x)
        {
            in_range_x = true;
        }
        if (from_leg == RIGHT)
        {
            if (y <= max_footstep_y && y >= -max_inv_footstep_y)
            {
                in_range_y = true;
            }
            if (theta <=  max_footstep_theta &&
                theta >= -max_inv_footstep_theta)
            {
                in_range_theta = true;
            }
        }
        else // from_leg == LEFT
        {
            if (y >= -max_footstep_y && y <= max_inv_footstep_y)
            {
                in_range_y = true;
            }
            if (theta >= -max_footstep_theta &&
                theta <=  max_inv_footstep_theta)
            {
                in_range_theta = true;
            }
        }

        return in_range_x && in_range_y && in_range_theta;
    }


	bool
	collision_check(double x, double y, double theta, double height,
                    double width, int accuracy, const GridMap2D& distance_map)
	{
		double d = distance_map.distanceMapAt(x, y);
		if (d < 0.0) // if out of bounds => collision
			return true;
		d -= distance_map.getResolution();

		double r_o = sqrt(width*width + height*height) / 2.0;

		if (d >= r_o)
			return false;
		else if (accuracy == 0)
			return false;

		double h_half = height / 2.0f;
		double w_half = width / 2.0f;
		double r_i = std::min(w_half, h_half);

		if (d <= r_i)
			return true;
		else if (accuracy == 1)
			return true;

		double h_new;
		double w_new;
		double delta_x;
		double delta_y;
		if (width < height)
		{
			double h_clear = sqrt(d*d - w_half*w_half);
			h_new = h_half - h_clear;
			w_new = width;
			delta_x = h_clear + h_new/2.0;
			delta_y = 0.0;
		}
		else // footWidth >= footHeight
		{
			double w_clear = sqrt(d*d - h_half*h_half);
			h_new = height;
			w_new = w_half - w_clear;
			delta_x = 0.0;
			delta_y = w_clear + w_new/2.0;
		}
		double theta_cos = cos(theta);
		double theta_sin = sin(theta);
		double x_shift = theta_cos*delta_x - theta_sin*delta_y;
		double y_shift = theta_sin*delta_x + theta_cos*delta_y;

		return (collision_check(x+x_shift, y+y_shift, theta, h_new, w_new,
		                        accuracy, distance_map) ||
				collision_check(x-x_shift, y-y_shift, theta, h_new, w_new,
                                accuracy, distance_map));
	}
}

// SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_navigation/footstep_planner/src/Dstar.cpp $
// SVN $Id: Dstar.cpp 1168 2011-03-30 03:18:02Z hornunga@informatik.uni-freiburg.de $

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

#include <footstep_planner/Footstep.h>


namespace footstep_planner
{
	/*
	 * #########################################################################
	 * ### class Footstep
	 * #########################################################################
	 */

	Footstep::Footstep(double x, double y, double theta,
                       double cell_size, int num_angle_bins,
                       int max_hash_size, double foot_separation)
		: ivContX(x),
		  ivContY(y),
		  ivContTheta(theta),
		  ivCellSize(cell_size),
		  ivNumAngleBins(num_angle_bins),
		  ivMaxHashSize(max_hash_size),
		  ivFootSeparation(foot_separation),
		  ivTheta(angle_cont_2_disc(theta, num_angle_bins))
	{
	    init();
	}


	Footstep::~Footstep()
	{}


	void
	Footstep::updateNumAngleBins(int num)
	{
	    ivNumAngleBins = num;
	    ivTheta = angle_cont_2_disc(ivContTheta, num);
	    init();
	}


	void
	Footstep::init()
	{
	    ivSuccessorLeft.clear();
	    ivSuccessorRight.clear();
	    ivPredecessorLeft.clear();
	    ivPredecessorRight.clear();

        double shift_x;
        double shift_y;
        double global_theta;

        for (int a = 0; a < ivNumAngleBins; ++a)
        {
            global_theta = angle_disc_2_cont(a, ivNumAngleBins);
            calculateForwardStep(RIGHT, global_theta, &shift_x, &shift_y);
            ivSuccessorRight.push_back(shift_vector(shift_x, shift_y));
            calculateBackwardStep(RIGHT, global_theta, &shift_x, &shift_y);
            ivPredecessorRight.push_back(shift_vector(shift_x, shift_y));
        }

        for (int a = 0; a < ivNumAngleBins; ++a)
        {
            global_theta = angle_disc_2_cont(a, ivNumAngleBins);
            calculateForwardStep(LEFT, global_theta, &shift_x, &shift_y);
            ivSuccessorLeft.push_back(shift_vector(shift_x, shift_y));
            calculateBackwardStep(LEFT, global_theta, &shift_x, &shift_y);
            ivPredecessorLeft.push_back(shift_vector(shift_x, shift_y));
        }
	}


	PlanningState
	Footstep::performMeOnThisState(const PlanningState& current)
	const
	{
	    int angle;
	    double x;
        double y;
        double theta;
        Leg leg;

	    angle = current.getTheta();
        if (current.getLeg() == RIGHT)
        {
            shift_vector xy = ivSuccessorRight[angle];
            x = xy.first;
            y = xy.second;
            theta = ivContTheta;
            leg = LEFT;
        }
        else // leg == LEFT
        {
            shift_vector xy = ivSuccessorLeft[angle];
            x = xy.first;
            y = xy.second;
            theta = -ivContTheta;
            leg = RIGHT;
        }

        return PlanningState(current.getContX() + x, current.getContY() + y,
                             current.getContTheta() + theta, leg,
                             ivCellSize, ivNumAngleBins, ivMaxHashSize);
	}


    PlanningState
    Footstep::revertMeOnThisState(const PlanningState& current)
    const
    {
        int angle;
        double x;
        double y;
        double theta;
        Leg leg;

        angle = current.getTheta();
        if (current.getLeg() == LEFT)
        {
            shift_vector xy = ivPredecessorLeft[angle];
            x = xy.first;
            y = xy.second;
            theta = -ivContTheta;
            leg = RIGHT;
        }
        else // leg == RIGHT
        {
            shift_vector xy = ivPredecessorRight[angle];
            x = xy.first;
            y = xy.second;
            theta = ivContTheta;
            leg = LEFT;
        }

        return PlanningState(current.getContX() + x, current.getContY() + y,
                			 current.getContTheta() + theta, leg,
                             ivCellSize, ivNumAngleBins, ivMaxHashSize);
    }


	void
	Footstep::calculateForwardStep(Leg leg, double global_theta,
	                               double* shift_x, double* shift_y)
	const
	{
        double foot_separation_half = ivFootSeparation/2;

        double theta_cos = cos(global_theta);
        double theta_sin = sin(global_theta);
        if (leg == RIGHT)
        {
            *shift_x = theta_cos*ivContX - theta_sin*(ivContY+foot_separation_half);
            *shift_y = theta_sin*ivContX + theta_cos*(ivContY+foot_separation_half);
            global_theta += ivContTheta;
            *shift_x += -sin(global_theta) * foot_separation_half;
            *shift_y +=  cos(global_theta) * foot_separation_half;
        }
        else // leg == LEFT
        {
            *shift_x = theta_cos*ivContX + theta_sin*(ivContY+foot_separation_half);
            *shift_y = theta_sin*ivContX - theta_cos*(ivContY+foot_separation_half);
            global_theta -= ivContTheta;
            *shift_x +=  sin(global_theta) * foot_separation_half;
            *shift_y += -cos(global_theta) * foot_separation_half;
        }
	}


	void
    Footstep::calculateBackwardStep(Leg leg, double global_theta,
                                    double* shift_x, double* shift_y)
    const
    {
        double foot_separation_half = ivFootSeparation/2;

        if (leg == LEFT)
        {
            *shift_x =  sin(global_theta) * foot_separation_half;
            *shift_y = -cos(global_theta) * foot_separation_half;
            global_theta -= ivContTheta;
            double theta_cos = cos(global_theta);
            double theta_sin = sin(global_theta);
            *shift_x += -theta_cos*ivContX + theta_sin*(ivContY+foot_separation_half);
            *shift_y += -theta_sin*ivContX - theta_cos*(ivContY+foot_separation_half);
        }
        else // leg == RIGHT
        {
            *shift_x = -sin(global_theta) * foot_separation_half;
            *shift_y =  cos(global_theta) * foot_separation_half;
            global_theta += ivContTheta;
            double theta_cos = cos(global_theta);
            double theta_sin = sin(global_theta);
            *shift_x += -theta_cos*ivContX - theta_sin*(ivContY+foot_separation_half);
            *shift_y += -theta_sin*ivContX + theta_cos*(ivContY+foot_separation_half);
        }
    }
} // end of namespace

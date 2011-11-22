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

#ifndef HUMANOID_SBPL_FOOTSTEPPLANNERENVIRONMENT_H
#define HUMANOID_SBPL_FOOTSTEPPLANNERENVIRONMENT_H


#include <vector>

#include <boost/tr1/unordered_map.hpp>

#include <footstep_planner/helper.h>
#include <footstep_planner/PathCostHeuristic.h>
#include <footstep_planner/Heuristic.h>
#include <footstep_planner/Footstep.h>
#include <footstep_planner/PlanningState.h>
#include <sbpl/headers.h>


namespace footstep_planner
{
    class FootstepPlannerEnvironment : public DiscreteSpaceInformation
    {
    public:
        typedef std::vector<int> exp_states_t;
        typedef exp_states_t::const_iterator exp_states_iter_t;

        FootstepPlannerEnvironment(
                const  std::vector<Footstep>& footstep_set,
                const  boost::shared_ptr<Heuristic> heuristic,
                double foot_separation,
                double origin_foot_shift_x,
                double origin_foot_shift_y,
                double footsize_x,
                double footsize_y,
                int    max_footstep_x,
                int    max_footstep_y,
                int    max_footstep_theta,
                int    max_inverse_footstep_x,
                int    max_inverse_footstep_y,
                int    max_inverse_footstep_theta,
                double step_cost,
                int    collision_check_accuracy,
                int    hash_table_size,
                double cell_size,
                int    num_angle_bins,
                bool   forward_search);
        virtual ~FootstepPlannerEnvironment();

        void updateDistanceMap(GridMap2DPtr map);

        void setUp(const State& start_left, const State& start_right,
                   const State& goal_left, const State& goal_right);

        bool getState(unsigned int id, State* s);

        void reset();

        void printHashStatistics();

        int getNumExpandedStates() { return ivExpandedStates.size(); };

        exp_states_iter_t getExpandedStatesStart()
        {
            return ivExpandedStates.begin();
        };

        exp_states_iter_t getExpandedStatesEnd()
        {
            return ivExpandedStates.end();
        };

        /// overloaded from SBPL, returns costs in mm (truncated as int)
        int GetFromToHeuristic(int FromStateID, int ToStateID);

        int GetGoalHeuristic(int stateID);

        void GetPreds(int TargetStateID, std::vector<int> *PredIDV, \
                      std::vector<int> *CostV);

        int GetStartHeuristic(int stateID);

        void GetSuccs(int SourceStateID, std::vector<int> *SuccIDV, \
                      std::vector<int> *CostV);

        bool InitializeEnv(const char *sEnvFile);

        bool InitializeMDPCfg(MDPConfig *MDPCfg);

        void PrintEnv_Config(FILE *fOut);

        void PrintState(int stateID, bool bVerbose, FILE *fOut);

        void SetAllActionsandAllOutcomes(CMDPSTATE *state);

        void SetAllPreds(CMDPSTATE *state);

        int SizeofCreatedEnv();

        bool reachable(const PlanningState& from, const PlanningState& to);

        void getPredsOfGridCells(
        		const std::vector<State>& changed_states,
        		std::vector<int>* pred_ids);
        void getSuccsOfGridCells(
        		const std::vector<State>& changed_states,
        		std::vector<int>* succ_ids);

        /// to scale continuous values in meter to discrete mm
        static const int cvMmScale = 1000;

    private:
        int  stepCost(const PlanningState& a, const PlanningState& b);
        bool occupied(const PlanningState& s);
        void calculateHashTag(const PlanningState& s);

        const PlanningState* createNewHashEntry(const State& s);
        const PlanningState* createNewHashEntry(const PlanningState& s);
        const PlanningState* getHashEntry(const State& s);
        const PlanningState* getHashEntry(const PlanningState& s);

        bool closeToGoal(const PlanningState& from);
        bool closeToStart(const PlanningState& from);

        void updateGoal(const State& foot_left, const State& foot_right);
        void updateStart(const State& foot_left, const State& right_right);

        void updateHeuristicValues();

        void getFootstep(Leg support_leg, const PlanningState& from,
                         const PlanningState& to, double* footstep_x,
                         double* footstep_y, double* footstep_theta) const;

        struct less
        {
            bool operator ()(const PlanningState* a,
                             const PlanningState* b) const;
        };

		int ivGoalFootIdLeft;
        int ivGoalFootIdRight;
        int ivStartFootIdLeft;
        int ivStartFootIdRight;
        int ivHashFaultCounter;

        std::vector<const PlanningState*>  ivStateId2State;
        std::vector<const PlanningState*>* ivpStateHash2State;

        const std::vector<Footstep>& ivFootstepSet;
        const boost::shared_ptr<Heuristic> ivHeuristicConstPtr;

        const double ivFootSeparation;
        const double ivOriginFootShiftX, ivOriginFootShiftY;
        const double ivFootsizeX, ivFootsizeY;
        /// discretized int in cell size
        const int    ivMaxFootstepX, ivMaxFootstepY, ivMaxFootstepTheta;
        /// discretized int in cell size
        const int    ivMaxInvFootstepX, ivMaxInvFootstepY, ivMaxInvFootstepTheta;
        const int    ivStepCost; /// discretized int in mm
        const int    ivCollisionCheckAccuracy;
        const int    ivHashTableSize;
        const double ivCellSize;
        const int    ivNumAngleBins;
        const bool   ivForwardSearch;

        boost::shared_ptr<GridMap2D> ivMapPtr;

        exp_states_t ivExpandedStates;
    };
}

#endif  // HUMANOID_SBPL_FOOTSTEPPLANNERENVIRONMENT_H

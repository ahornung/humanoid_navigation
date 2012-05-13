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

#ifndef FOOTSTEP_PLANNER_ENVIRONMENT_H_
#define FOOTSTEP_PLANNER_ENVIRONMENT_H_


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
	/**
	 * @brief A class defining a footstep planner environment for humanoid
	 * robots used by the SBPL to perform planning tasks.
	 */
    class FootstepPlannerEnvironment : public DiscreteSpaceInformation
    {
    public:
        typedef std::vector<int> exp_states_t;
        typedef exp_states_t::const_iterator exp_states_iter_t;

        // TODO: discretize here
        /**
		 * @param footstep_set The set of footsteps used for the path planning.
		 * @param heuristic The heuristic used by the planner.
		 * @param origin_foot_shift_x Shift in x direction from the foot's
		 * center.
		 * @param origin_foot_shift_y Shift in y direction from the foot's
		 * center.
		 * @param footsize_x Size of the foot in x direction.
		 * @param footsize_y Size of the foot in y direction.
		 * @param max_footstep_x The maximal translation in x direction.
		 * @param max_footstep_y The maximal translation in y direction.
		 * @param max_footstep_theta The maximal rotation.
		 * @param max_inverse_footstep_x The minimal translation in x direction.
		 * @param max_inverse_footstep_y The minimal translation in y direction.
		 * @param max_inverse_footstep_theta The minimal rotation.
		 * @param step_cost The costs for each step.
		 * @param collision_check_accuracy Whether to check just the foot's
		 * inner circle (0), the hole outer circle (1) or exactly the foot
		 * bounding box (2) for collision.
		 * @param hash_table_size Size of the hash table storing the planning
		 * states expanded during the search.
		 * @param cell_size The size of each grid cell discretizing the
		 * position.
		 * @param num_angle_bins The number of bins discretizing the
		 * orientation.
		 * @param forward_search Whether to use forward_search (1) or backward
		 * search (0).
		 */
		FootstepPlannerEnvironment(
                const  std::vector<Footstep>& footstep_set,
                const  boost::shared_ptr<Heuristic> heuristic,
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

        void setMap(GridMap2DPtr map);

        /**
         * @brief Used to set up the start (for both feet) and goal (for both
         * feet) position for a planning task.
         */
        void setUp(const State& start_left, const State& start_right,
                   const State& goal_left, const State& goal_right);

        /**
         * @return Returns true iff the foot in State s is colliding with an
         * obstacle.
         */
        bool occupied(const State& s);

        /**
         * @brief Try to resolve a state with a certain ID.
         *
         * @return Returns true if there is a state with such an ID, false
         * otherwise.
         */
        bool getState(unsigned int id, State* s);

        /**
         * @brief Resets the current planning task (i.e. the start and goal
         * poses).
         */
        void reset();

        /// @return Returns the number of exanded states during the search.
        int getNumExpandedStates() { return ivExpandedStates.size(); };

        exp_states_iter_t getExpandedStatesStart()
        {
            return ivExpandedStates.begin();
        };

        exp_states_iter_t getExpandedStatesEnd()
        {
            return ivExpandedStates.end();
        };

        exp_states_iter_t getRandomStatesStart()
        {
            return ivRandomStates.begin();
        };

        exp_states_iter_t getRandomStatesEnd()
        {
            return ivRandomStates.end();
        };

        /**
         * @return Returns the costs (in mm, truncated as int) to reach
         * ToStateID from FromStateID.
         */
        int GetFromToHeuristic(int FromStateID, int ToStateID);

        /**
         * @return Returns the heuristic value to reach the goal state from
         * the state stateID (used for forward planning).
         */
        int GetGoalHeuristic(int stateID);

        /**
         * @return Returns the heuristic value to reach the start state from
         * the state stateID (used for backward planning).
         */
        int GetStartHeuristic(int stateID);

        /**
         * @brief Calculates the predecessor states and the corresponding costs
         * when reverting the footsteps within state TargetStateID.
         */
        void GetPreds(int TargetStateID, std::vector<int> *PredIDV,
                      std::vector<int> *CostV);

        void GetSuccs(int SourceStateID, std::vector<int> *SuccIDV,
                      std::vector<int> *CostV);

        /**
         * @brief Mainly used for RStar: generate succs/preds at some
         * domain-dependent distance. The number of generated succs/preds is up
         * to the environment.
         */
        virtual void GetRandomSuccsatDistance(int SourceStateID,
		                                      std::vector<int>* SuccIDV,
		                                      std::vector<int>* CLowV);

        /**
         * @brief Mainly used for RStar: generate succs/preds at some
         * domain-dependent distance. The number of generated succs/preds is up
         * to the environment.
         */
        virtual void GetRandomPredsatDistance(int TargetStateID,
		                                      std::vector<int>* PredIDV,
		                                      std::vector<int>* CLowV);

    	/// @return True if two states meet the same condition. Used for R*.
        bool AreEquivalent(int StateID1, int StateID2);
        bool InitializeEnv(const char *sEnvFile);

        bool InitializeMDPCfg(MDPConfig *MDPCfg);

        void PrintEnv_Config(FILE *fOut);

        void PrintState(int stateID, bool bVerbose, FILE *fOut);

        void SetAllActionsandAllOutcomes(CMDPSTATE *state);

        void SetAllPreds(CMDPSTATE *state);

        int SizeofCreatedEnv();

        bool reachable(const PlanningState& from, const PlanningState& to);

        void getPredsOfGridCells(const std::vector<State>& changed_states,
		                         std::vector<int>* pred_ids);

        void getSuccsOfGridCells(const std::vector<State>& changed_states,
		                         std::vector<int>* succ_ids);

        /// Used to scale continuous values in meter to discrete values in mm.
        static const int cvMmScale = 1000;

    private:
        /// @return Returns the step cost for reaching b from a.
        int  stepCost(const PlanningState& a, const PlanningState& b);

        /**
         * @return Returns true iff the foot in PlanningState s is colliding
         * with an obstacle.
         */
        bool occupied(const PlanningState& s);

        void GetRandomNeighs(const PlanningState* currentState,
		                     std::vector<int>* NeighIDV,
		                     std::vector<int>* CLowV,
		                     int nNumofNeighs, int nDist_c, bool bSuccs);

        const PlanningState* createNewHashEntry(const State& s);
        const PlanningState* createNewHashEntry(const PlanningState& s);
        const PlanningState* getHashEntry(const State& s);
        const PlanningState* getHashEntry(const PlanningState& s);

        bool closeToGoal(const PlanningState& from);
        bool closeToStart(const PlanningState& from);

        void updateGoal(const State& foot_left, const State& foot_right);
        void updateStart(const State& foot_left, const State& right_right);

        void updateHeuristicValues();

        void getFootstep(const PlanningState& from, const PlanningState& to,
		                 double& footstep_x, double& footstep_y,
		                 double& footstep_theta) const;

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

        const double ivOriginFootShiftX, ivOriginFootShiftY;
        const double ivFootsizeX, ivFootsizeY;
        /// discretized int in cell size
        const int    ivMaxFootstepX, ivMaxFootstepY, ivMaxFootstepTheta;
        /// discretized int in cell size
        const int    ivMaxInvFootstepX, ivMaxInvFootstepY,
                     ivMaxInvFootstepTheta;
        const int    ivStepCost;
        const int    ivCollisionCheckAccuracy;
        const int    ivHashTableSize;
        const double ivCellSize;
        const int    ivNumAngleBins;
        const bool   ivForwardSearch;
        /// < number of random neighbors for R*
        const int 	 ivNumRandomNeighbors;
        /// < distance of random neighbors for R* (discretized in cells)
        const int    ivRandomNeighborsDist;


        boost::shared_ptr<GridMap2D> ivMapPtr;

        exp_states_t ivExpandedStates;
        exp_states_t ivRandomStates; ///< random intermediate states for R*

    };
}

#endif  // FOOTSTEP_PLANNER_ENVIRONMENT_H_

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

#include <footstep_planner/FootstepPlannerEnvironment.h>


namespace footstep_planner
{
    FootstepPlannerEnvironment::FootstepPlannerEnvironment(
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
            bool   forward_search)
        : DiscreteSpaceInformation(),
          ivGoalFootIdLeft(-1),
          ivGoalFootIdRight(-1),
          ivStartFootIdLeft(-1),
          ivStartFootIdRight(-1),
          ivHashFaultCounter(0),
          ivpStateHash2State(NULL),
          ivFootstepSet(footstep_set),
          ivHeuristicConstPtr(heuristic),
          ivFootSeparation(foot_separation),
          ivOriginFootShiftX(origin_foot_shift_x),
          ivOriginFootShiftY(origin_foot_shift_y),
          ivFootsizeX(footsize_x),
          ivFootsizeY(footsize_y),
          ivMaxFootstepX(max_footstep_x),
          ivMaxFootstepY(max_footstep_y),
          ivMaxFootstepTheta(max_footstep_theta),
          ivMaxInvFootstepX(max_inverse_footstep_x),
          ivMaxInvFootstepY(max_inverse_footstep_y),
          ivMaxInvFootstepTheta(max_inverse_footstep_theta),
          ivStepCost(cvMmScale * step_cost),
          ivCollisionCheckAccuracy(collision_check_accuracy),
          ivHashTableSize(hash_table_size),
          ivCellSize(cell_size),
          ivNumAngleBins(num_angle_bins),
          ivForwardSearch(forward_search)
    {}


    FootstepPlannerEnvironment::~FootstepPlannerEnvironment()
    {
        reset();
        if (ivpStateHash2State)
        {
			delete[] ivpStateHash2State;
			ivpStateHash2State = NULL;
        }
    }


    void
    FootstepPlannerEnvironment::setUp(const State& start_foot_left,
                                      const State& start_foot_right,
                                      const State& goal_foot_left,
                                      const State& goal_foot_right)
    {
        if (ivpStateHash2State == NULL)
        {
            ivpStateHash2State =
            		new std::vector<const PlanningState*>[ivHashTableSize];
        }

        int start_foot_id_left = ivStartFootIdLeft;
        int start_foot_id_right = ivStartFootIdRight;
        int goal_foot_id_left = ivGoalFootIdLeft;
        int goal_foot_id_right = ivGoalFootIdRight;

        updateStart(start_foot_left, start_foot_right);
        updateGoal(goal_foot_left, goal_foot_right);

        if (ivForwardSearch)
        {
			if (goal_foot_id_left != ivGoalFootIdLeft &&
				goal_foot_id_right != ivGoalFootIdRight)
			{
				updateHeuristicValues();
			}
        }
        else
        {
			if (start_foot_id_left != ivStartFootIdLeft &&
				start_foot_id_right != ivStartFootIdRight)
			{
				updateHeuristicValues();
			}
        }
    }


    void
    FootstepPlannerEnvironment::updateGoal(const State& foot_left,
                                           const State& foot_right)
    {
        const PlanningState* p_foot_left = getHashEntry(foot_left);
        if (p_foot_left == NULL)
        {
            p_foot_left = createNewHashEntry(foot_left);
            ivGoalFootIdLeft = p_foot_left->getId();
        }

        const PlanningState* p_foot_right = getHashEntry(foot_right);
        if (p_foot_right == NULL)
        {
            p_foot_right = createNewHashEntry(foot_right);
            ivGoalFootIdRight = p_foot_right->getId();
        }

        assert(ivGoalFootIdLeft != -1);
        assert(ivGoalFootIdRight != -1);
    }


    void
    FootstepPlannerEnvironment::updateStart(const State& foot_left,
                                            const State& foot_right)
    {
        const PlanningState* p_foot_left = getHashEntry(foot_left);
        if (p_foot_left == NULL)
        {
            p_foot_left = createNewHashEntry(foot_left);
            ivStartFootIdLeft = p_foot_left->getId();
        }

        const PlanningState* p_foot_right = getHashEntry(foot_right);
        if (p_foot_right == NULL)
        {
            p_foot_right = createNewHashEntry(foot_right);
            ivStartFootIdRight = p_foot_right->getId();
        }

        assert(ivStartFootIdLeft != -1);
        assert(ivStartFootIdRight != -1);
    }


    const PlanningState*
    FootstepPlannerEnvironment::createNewHashEntry(const State& s)
    {
        PlanningState tmp(s, ivCellSize, ivNumAngleBins, ivHashTableSize);
        return createNewHashEntry(tmp);
    }


    const PlanningState*
    FootstepPlannerEnvironment::createNewHashEntry(const PlanningState& s)
    {
        unsigned int state_hash = s.getHashTag();
        PlanningState* new_state = new PlanningState(s);

        int state_id = ivStateId2State.size();
        assert(state_id < numeric_limits<int>::max());

        new_state->setId(state_id);
        ivStateId2State.push_back(new_state);

        ivpStateHash2State[state_hash].push_back(new_state);
#if DEBUG_HASH
        if (ivpStateHash2State[state_hash].size() > 1)
        {
            const PlanningState* first = ivpStateHash2State[state_hash].front();
            if (*first != s)
            {
                ivHashFaultCounter++;
            }
        }
#endif

        int* entry = new int[NUMOFINDICES_STATEID2IND];
        StateID2IndexMapping.push_back(entry);
        for(int i = 0; i < NUMOFINDICES_STATEID2IND; i++)
        {
            StateID2IndexMapping[state_id][i] = -1;
        }

        return new_state;
    }


    const PlanningState*
    FootstepPlannerEnvironment::getHashEntry(const State& s)
    {
        PlanningState tmp(s, ivCellSize, ivNumAngleBins, ivHashTableSize);
        return getHashEntry(tmp);
    }


    const PlanningState*
    FootstepPlannerEnvironment::getHashEntry(const PlanningState& s)
    {
        unsigned int state_hash = s.getHashTag();
        std::vector<const PlanningState*>::const_iterator state_iter;
        state_iter = ivpStateHash2State[state_hash].begin();
        for (; state_iter != ivpStateHash2State[state_hash].end(); state_iter++)
        {
            if (*(*state_iter) == s)
            {
                return *state_iter;
            }
        }

        return NULL;
    }


    int
    FootstepPlannerEnvironment::stepCost(const PlanningState& a,
	                                     const PlanningState& b)
    {
        if (a == b)
            return 0;

		double dist = euclidean_distance(  disc_2_cont(a.getX(), ivCellSize),
									disc_2_cont(a.getY(), ivCellSize),
									disc_2_cont(b.getX(), ivCellSize),
									disc_2_cont(b.getY(), ivCellSize));
		return int(cvMmScale * dist) + ivStepCost;
    }


    bool
    FootstepPlannerEnvironment::occupied(const PlanningState& s)
    {
        double x = disc_2_cont(s.getX(), ivCellSize);
        double y = disc_2_cont(s.getY(), ivCellSize);
        if (ivMapPtr->isOccupiedAt(x,y))
        	return true;
        double theta = angle_disc_2_cont(s.getTheta(), ivNumAngleBins);
        double theta_cos = cos(theta);
        double theta_sin = sin(theta);

        x += theta_cos*ivOriginFootShiftX - theta_sin*ivOriginFootShiftY;
        if (s.getLeg() == LEFT)
            y += theta_sin*ivOriginFootShiftX + theta_cos*ivOriginFootShiftY;
        else // leg == RLEG
            y += theta_sin*ivOriginFootShiftX - theta_cos*ivOriginFootShiftY;

        return collision_check(x, y, theta, ivFootsizeX, ivFootsizeY,
                               ivCollisionCheckAccuracy, *ivMapPtr);
    }


    bool
    FootstepPlannerEnvironment::getState(unsigned int id, State* s)
    {
        if (ivStateId2State.size() <= id)
            return false;

        const PlanningState* planning_state = ivStateId2State[id];
        s->x = disc_2_cont(planning_state->getX(), ivCellSize);
        s->y = disc_2_cont(planning_state->getY(), ivCellSize);
        s->theta = angle_disc_2_cont(planning_state->getTheta(),
                                     ivNumAngleBins);
        s->leg = planning_state->getLeg();

        return true;
    }


    void
    FootstepPlannerEnvironment::updateDistanceMap(GridMap2DPtr map)
    {
        ivMapPtr.reset();
        ivMapPtr = map;

        if (ivHeuristicConstPtr->getHeuristicType() == Heuristic::PATH_COST)
        {
            boost::shared_ptr<PathCostHeuristic> h =
            		boost::dynamic_pointer_cast<PathCostHeuristic>(
            				ivHeuristicConstPtr);
            h->setMap(map);
        }

    }


    void
    FootstepPlannerEnvironment::updateHeuristicValues()
    {
        if (ivHeuristicConstPtr->getHeuristicType() == Heuristic::PATH_COST)
        {
            boost::shared_ptr<PathCostHeuristic> h =
            		boost::dynamic_pointer_cast<PathCostHeuristic>(ivHeuristicConstPtr);
            MDPConfig MDPCfg;
            InitializeMDPCfg(&MDPCfg);
            const PlanningState* start = ivStateId2State[MDPCfg.startstateid];
            const PlanningState* goal = ivStateId2State[MDPCfg.goalstateid];

            // NOTE: start/goal state are set to left leg
            bool success;
            if (ivForwardSearch)
                success = h->calculateDistances(*start, *goal);
            else
                success = h->calculateDistances(*goal, *start);
            if (!success)
            {
                ROS_ERROR("Failed to calculate path cost heuristic.");
                exit(1);
            }
            ROS_INFO("Finished calculating path cost heuristic.");
        }
    }


    void
    FootstepPlannerEnvironment::reset()
    {
        for(unsigned int i = 0; i < ivStateId2State.size(); ++i)
        {
        	if (ivStateId2State[i])
        	{
        		delete ivStateId2State[i];
        		ivStateId2State[i] = NULL;
        	}
        }
        ivStateId2State.clear();

        if (ivpStateHash2State)
        {
			for(int i = 0; i < ivHashTableSize; ++i)
				ivpStateHash2State[i].clear();
        }

        for(unsigned int i = 0; i < StateID2IndexMapping.size(); ++i)
        {
        	if (StateID2IndexMapping[i])
        	{
        		delete[] StateID2IndexMapping[i];
        		StateID2IndexMapping[i] = NULL;
        	}
        }
        StateID2IndexMapping.clear();

        ivExpandedStates.clear();

        ivGoalFootIdLeft = -1;
        ivGoalFootIdRight = -1;
        ivStartFootIdLeft = -1;
        ivStartFootIdRight = -1;
    }


    bool
    FootstepPlannerEnvironment::closeToStart(const PlanningState& from)
    {
        // NOTE: "goal check" for backward planning
        const PlanningState* start;
        if (from.getLeg() == RIGHT)
            start = ivStateId2State[ivStartFootIdLeft];
        else
            start = ivStateId2State[ivStartFootIdRight];

        return reachable(*start, from);
    }


    bool
    FootstepPlannerEnvironment::closeToGoal(const PlanningState& from)
    {
        // NOTE: goal check for forward planning
        const PlanningState* goal;
        if (from.getLeg() == RIGHT)
            goal = ivStateId2State[ivGoalFootIdLeft];
        else
            goal = ivStateId2State[ivGoalFootIdRight];

        return reachable(from, *goal);
    }


    void
    FootstepPlannerEnvironment::getFootstep(Leg support_leg,
                                            const PlanningState& from,
                                            const PlanningState& to,
                                            double* footstep_x,
                                            double* footstep_y,
                                            double* footstep_theta) const
    {
        double from_x = disc_2_cont(from.getX(), ivCellSize);
        double from_y = disc_2_cont(from.getY(), ivCellSize);
        double from_theta = angle_disc_2_cont(from.getTheta(), ivNumAngleBins);
        double to_x = disc_2_cont(to.getX(), ivCellSize);
        double to_y = disc_2_cont(to.getY(), ivCellSize);
        double to_theta = angle_disc_2_cont(to.getTheta(), ivNumAngleBins);

        // TODO: speed comparison with other get_footstep function
        get_footstep(support_leg, ivFootSeparation, from_x, from_y, from_theta,
                     to_x, to_y, to_theta, footstep_x, footstep_y,
                     footstep_theta);
    }


    bool
    FootstepPlannerEnvironment::reachable(const PlanningState& from,
	                                      const PlanningState& to)
    {
        bool in_range_x = false;
        bool in_range_y = false;
        bool in_range_theta = false;

        double x;
        double y;
        double theta;

        getFootstep(from.getLeg(), from, to, &x, &y, &theta);

        int diff_x = cont_2_disc(x, ivCellSize);
        int diff_y = cont_2_disc(y, ivCellSize);
        int diff_theta = angle_cont_2_disc(theta, ivNumAngleBins);

        if (diff_x <=  ivMaxFootstepX && diff_x >= -ivMaxInvFootstepX)
        {
            in_range_x = true;
        }
        if (from.getLeg() == RIGHT)
        {
            if (diff_y <=  ivMaxFootstepY && diff_y >= -ivMaxInvFootstepY)
            {
                in_range_y = true;
            }
            if (diff_theta <=  ivMaxFootstepTheta &&
                diff_theta >= -ivMaxInvFootstepTheta)
            {
                in_range_theta = true;
            }
        }
        else // leg == LEFT
        {
            if (diff_y >= -ivMaxFootstepY && diff_y <= ivMaxInvFootstepY)
            {
                in_range_y = true;
            }
            if (diff_theta >= -ivMaxFootstepTheta &&
                diff_theta <=  ivMaxInvFootstepTheta)
            {
                in_range_theta = true;
            }
        }

        return in_range_x && in_range_y && in_range_theta;
    }


    void
    FootstepPlannerEnvironment::getPredsOfGridCells(
	        const std::vector<State>& changed_states,
	        std::vector<int>* pred_ids)
    {
    	pred_ids->clear();

		std::vector<State>::const_iterator state_iter = changed_states.begin();
		for (; state_iter != changed_states.end(); state_iter++)
		{
			PlanningState s(*state_iter, ivCellSize, ivNumAngleBins,
			                ivHashTableSize);

			std::vector<Footstep>::const_iterator footstep_set_iter;
			for(footstep_set_iter = ivFootstepSet.begin();
                footstep_set_iter != ivFootstepSet.end();
                footstep_set_iter++)
			{
				PlanningState successor =
						footstep_set_iter->revertMeOnThisState(s);
				if (occupied(successor))
					continue;

				const PlanningState* successor_hash_entry =
						getHashEntry(successor);
				if (successor_hash_entry == NULL)
					continue;

				pred_ids->push_back(successor_hash_entry->getId());
			}
		}
    }


    void
    FootstepPlannerEnvironment::getSuccsOfGridCells(
	        const std::vector<State>& changed_states,
	        std::vector<int>* succ_ids)
    {
    	succ_ids->clear();

		std::vector<State>::const_iterator state_iter = changed_states.begin();
		for (; state_iter != changed_states.end(); state_iter++)
		{
			PlanningState s(*state_iter, ivCellSize, ivNumAngleBins,
			                ivHashTableSize);

			std::vector<Footstep>::const_iterator footstep_set_iter;
			for(footstep_set_iter = ivFootstepSet.begin();
			    footstep_set_iter != ivFootstepSet.end();
			    footstep_set_iter++)
			{
				PlanningState successor =
						footstep_set_iter->performMeOnThisState(s);
				if (occupied(successor))
					continue;

				const PlanningState* successor_hash_entry =
						getHashEntry(successor);
				if (successor_hash_entry == NULL)
					continue;

				succ_ids->push_back(successor_hash_entry->getId());
			}
		}
    }


    void
    FootstepPlannerEnvironment::printHashStatistics()
    {
#if DEBUG_HASH
        ROS_INFO("Number of unequal states with equal hash tag: %i",
                 ivHashFaultCounter);
#endif
    }


    int
    FootstepPlannerEnvironment::GetFromToHeuristic(int FromStateID,
                                                   int ToStateID)
    {
    	assert(FromStateID < ivStateId2State.size());

    	const PlanningState* from = ivStateId2State[FromStateID];
    	return cvMmScale * ivHeuristicConstPtr->getHValue(
    	        *from, *ivStateId2State[ToStateID]);
    }


    int
    FootstepPlannerEnvironment::GetGoalHeuristic(int stateID)
    {
        return GetFromToHeuristic(stateID, ivGoalFootIdLeft);
    }


    void
    FootstepPlannerEnvironment::GetPreds(int TargetStateID,
                                         std::vector<int> *PredIDV,
                                         std::vector<int> *CostV)
    {
        PredIDV->clear();
        CostV->clear();
        PredIDV->reserve(ivFootstepSet.size());
        CostV->reserve(ivFootstepSet.size());

        ivExpandedStates.push_back(TargetStateID);

        const PlanningState* current = ivStateId2State[TargetStateID];
        if (closeToStart(*current))
        {
            int start_id;
            if (current->getLeg() == RIGHT)
                start_id = ivStartFootIdLeft;
            else
                start_id = ivStartFootIdRight;
            const PlanningState* start = ivStateId2State[start_id];
            int cost = stepCost(*current, *start);
            PredIDV->push_back(start_id);
            CostV->push_back(cost);

            return;
        }

        std::vector<Footstep>::const_iterator footstep_set_iter;
        footstep_set_iter = ivFootstepSet.begin();
        for(; footstep_set_iter != ivFootstepSet.end(); footstep_set_iter++)
        {
            const PlanningState predecessor =
            		footstep_set_iter->revertMeOnThisState(*current);
            if (occupied(predecessor))
                continue;

            const PlanningState* predecessor_hash_entry =
            		getHashEntry(predecessor);
            if (predecessor_hash_entry == NULL)
                predecessor_hash_entry = createNewHashEntry(predecessor);

            int cost = stepCost(*current, *predecessor_hash_entry);
            PredIDV->push_back(predecessor_hash_entry->getId());
            CostV->push_back(cost);
        }
    }


    int
    FootstepPlannerEnvironment::GetStartHeuristic(int stateID)
    {
        return GetFromToHeuristic(stateID, ivStartFootIdLeft);
    }


    void
    FootstepPlannerEnvironment::GetSuccs(int SourceStateID,
                                         std::vector<int> *SuccIDV,
                                         std::vector<int> *CostV)
    {
        SuccIDV->clear();
        CostV->clear();
        SuccIDV->reserve(ivFootstepSet.size());
        CostV->reserve(ivFootstepSet.size());

        ivExpandedStates.push_back(SourceStateID);

        const PlanningState* current = ivStateId2State[SourceStateID];
        if (closeToGoal(*current))
        {
            int goal_id;
            if (current->getLeg() == RIGHT)
                goal_id = ivGoalFootIdLeft;
            else
                goal_id = ivGoalFootIdRight;
            const PlanningState* goal = ivStateId2State[goal_id];
            int cost = stepCost(*current, *goal);
            SuccIDV->push_back(goal_id);
            CostV->push_back(cost);

            return;
        }

        std::vector<Footstep>::const_iterator footstep_set_iter;
        footstep_set_iter = ivFootstepSet.begin();
        for(; footstep_set_iter != ivFootstepSet.end(); footstep_set_iter++)
        {
            PlanningState successor =
            		footstep_set_iter->performMeOnThisState(*current);
            if (occupied(successor))
                continue;

            const PlanningState* successor_hash_entry = getHashEntry(successor);
            if (successor_hash_entry == NULL)
                successor_hash_entry = createNewHashEntry(successor);

            int cost = stepCost(*current, *successor_hash_entry);
            SuccIDV->push_back(successor_hash_entry->getId());
            CostV->push_back(cost);
        }
    }


    bool
    FootstepPlannerEnvironment::InitializeEnv(const char *sEnvFile)
    {
        // TODO: planner instance can be initialized here (so far not necessary)

        return true;
    }


    bool
    FootstepPlannerEnvironment::InitializeMDPCfg(MDPConfig *MDPCfg)
    {
        // NOTE: The internal start and goal ids are set here to the left foot
        // (this affects the calculation of the heuristic values)
        MDPCfg->goalstateid = ivGoalFootIdLeft;
        MDPCfg->startstateid = ivStartFootIdLeft;

        return true;
    }


    void
    FootstepPlannerEnvironment::PrintEnv_Config(FILE *fOut)
    {
        // implement this if the planner needs to print out configurations

        ROS_ERROR("SimplePlanner::PrintEnv_Config: Hit  unimplemented function."
                  " Check this!");
    }


    void
    FootstepPlannerEnvironment::PrintState(int stateID, bool bVerbose,
                                           FILE *fOut)
    {
        if(fOut == NULL)
        {
            fOut = stdout;
        }

        if(stateID == ivGoalFootIdLeft && bVerbose)
        {
            SBPL_FPRINTF(fOut, "the state is a goal state\n");
        }

        const PlanningState* s = ivStateId2State[stateID];

        if(bVerbose)
        {
            SBPL_FPRINTF(fOut, "X=%i Y=%i THETA=%i FOOT=%i\n",
                         s->getX(), s->getY(), s->getTheta(), s->getLeg());
        }
        else
        {
            SBPL_FPRINTF(fOut, "%i %i %i %i\n",
                         s->getX(), s->getY(), s->getTheta(), s->getLeg());
        }
    }


    void
    FootstepPlannerEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE *state)
    {
        // NOTE: not implemented so far
        // Description: Some searches may also use SetAllActionsandAllOutcomes
        // or SetAllPreds functions if they keep the pointers to successors
        // (predecessors) but most searches do not require this, so it is not
        // necessary to support this

        ROS_ERROR("SimplePlanner::SetAllActionsandAllOutcomes: Hit "
                  "unimplemented function. Check this!");
    }


    void
    FootstepPlannerEnvironment::SetAllPreds(CMDPSTATE *state)
    {
        // NOTE: not implemented so far
        // Description: Some searches may also use SetAllActionsandAllOutcomes
        // or SetAllPreds functions if they keep the pointers to successors
        // (predecessors) but most searches do not require this, so it is not
        // necessary to support this

        ROS_ERROR("SimplePlanner::SetAllPreds: Hit unimplemented "
                  "function. Check this!");
    }


    int
    FootstepPlannerEnvironment::SizeofCreatedEnv()
    {
        return ivStateId2State.size();
    }


    bool
    FootstepPlannerEnvironment::less::operator ()(const PlanningState* a,
                                                  const PlanningState* b)
    const
    {
        if (a->getX() < b->getX())
            return true;
        else if (a->getY() < b->getY())
            return true;
        else
            return false;
    }
}

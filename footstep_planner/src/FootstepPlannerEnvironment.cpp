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

		double dist = euclidean_distance(cell_2_state(a.getX(), ivCellSize),
                                         cell_2_state(a.getY(), ivCellSize),
                                         cell_2_state(b.getX(), ivCellSize),
                                         cell_2_state(b.getY(), ivCellSize));
		return int(cvMmScale * dist) + ivStepCost;
    }


    bool
    FootstepPlannerEnvironment::occupied(const State& s)
    {
        return occupied(PlanningState(s, ivCellSize, ivNumAngleBins,
                                      ivHashTableSize));
    }


    bool
    FootstepPlannerEnvironment::occupied(const PlanningState& s)
    {
        double x = cell_2_state(s.getX(), ivCellSize);
        double y = cell_2_state(s.getY(), ivCellSize);
        // collision check for the planning state
        if (ivMapPtr->isOccupiedAt(x,y))
        	return true;
        double theta = angle_cell_2_state(s.getTheta(), ivNumAngleBins);
        double theta_cos = cos(theta);
        double theta_sin = sin(theta);

        // transform the planning state to the foot center
        x += theta_cos*ivOriginFootShiftX - theta_sin*ivOriginFootShiftY;
        if (s.getLeg() == LEFT)
            y += theta_sin*ivOriginFootShiftX + theta_cos*ivOriginFootShiftY;
        else // leg == RLEG
            y += theta_sin*ivOriginFootShiftX - theta_cos*ivOriginFootShiftY;

        // collision check for the foot center
        return collision_check(x, y, theta, ivFootsizeX, ivFootsizeY,
                               ivCollisionCheckAccuracy, *ivMapPtr);
    }


    bool
    FootstepPlannerEnvironment::getState(unsigned int id, State* s)
    {
        if (ivStateId2State.size() <= id)
            return false;

        const PlanningState* planning_state = ivStateId2State[id];
        s->x = cell_2_state(planning_state->getX(), ivCellSize);
        s->y = cell_2_state(planning_state->getY(), ivCellSize);
        s->theta = angle_cell_2_state(planning_state->getTheta(),
                                     ivNumAngleBins);
        s->leg = planning_state->getLeg();

        return true;
    }


    void
    FootstepPlannerEnvironment::setMap(GridMap2DPtr map)
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

        return reachableState(*start, from);
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

        return reachableState(from, *goal);
    }


    void
    FootstepPlannerEnvironment::getFootstep(Leg support_leg,
                                            const PlanningState& from,
                                            const PlanningState& to,
                                            double& footstep_x,
                                            double& footstep_y,
                                            double& footstep_theta) const
    {
        double from_x = cell_2_state(from.getX(), ivCellSize);
        double from_y = cell_2_state(from.getY(), ivCellSize);
        double from_theta = angle_cell_2_state(from.getTheta(), ivNumAngleBins);
        double to_x = cell_2_state(to.getX(), ivCellSize);
        double to_y = cell_2_state(to.getY(), ivCellSize);
        double to_theta = angle_cell_2_state(to.getTheta(), ivNumAngleBins);

        get_footstep(support_leg, ivFootSeparation,
                     from_x, from_y, from_theta,
                     to_x, to_y, to_theta,
                     footstep_x, footstep_y, footstep_theta);
    }


    bool
    FootstepPlannerEnvironment::reachableState(const PlanningState& from,
	                                           const PlanningState& to)
    {
        double cont_footstep_x;
        double cont_footstep_y;
        double cont_footstep_theta;
        getFootstep(from.getLeg(), from, to,
                    cont_footstep_x, cont_footstep_y, cont_footstep_theta);

        int footstep_x = discretize(cont_footstep_x, ivCellSize);
        int footstep_y = discretize(cont_footstep_y, ivCellSize);
        int footstep_theta = angle_state_2_cell(cont_footstep_theta,
                                                ivNumAngleBins);
        return performable(footstep_x, footstep_y,
                           footstep_theta,
                           ivMaxFootstepX, ivMaxFootstepY, ivMaxFootstepTheta,
                           ivMaxInvFootstepX, ivMaxInvFootstepY,
                           ivMaxInvFootstepTheta,
                           ivNumAngleBins);
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
            // generate predecessor planning states
			std::vector<Footstep>::const_iterator footstep_set_iter;
			for(footstep_set_iter = ivFootstepSet.begin();
                footstep_set_iter != ivFootstepSet.end();
                footstep_set_iter++)
			{
				PlanningState successor =
						footstep_set_iter->revertMeOnThisState(s);
				// check if predecessor exists
				const PlanningState* successor_hash_entry =
						getHashEntry(successor);
				if (successor_hash_entry == NULL)
					continue;
			    // check if predecessor is occupied
//				if (occupied(successor))
//					continue;
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
			// generate successors
			std::vector<Footstep>::const_iterator footstep_set_iter;
			for(footstep_set_iter = ivFootstepSet.begin();
			    footstep_set_iter != ivFootstepSet.end();
			    footstep_set_iter++)
			{
				PlanningState successor =
						footstep_set_iter->performMeOnThisState(s);
				// check if successor exists
				const PlanningState* successor_hash_entry =
						getHashEntry(successor);
				if (successor_hash_entry == NULL)
					continue;
				// check if successor is occupied
//				if (occupied(successor))
//					continue;
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
    	assert((unsigned int) FromStateID < ivStateId2State.size());

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

        assert((unsigned int) TargetStateID < ivStateId2State.size());
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

        PredIDV->reserve(ivFootstepSet.size());
        CostV->reserve(ivFootstepSet.size());
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

        assert((unsigned int) SourceStateID < ivStateId2State.size());
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

        SuccIDV->reserve(ivFootstepSet.size());
        CostV->reserve(ivFootstepSet.size());
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


    void
    FootstepPlannerEnvironment::GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV)
    {
    	//number of random neighbors
    	int nNumofNeighs = 20;
    	//distance at which the neighbors are generated
    	double randomNeighborDist = 0.5;
    	int nDist_c = randomNeighborDist / ivCellSize;

    	assert(SourceStateID < ivStateId2State.size());
    	const PlanningState* currentState = ivStateId2State[SourceStateID];

    	//goal state should be absorbing
    	if (closeToGoal(*currentState))
    		return;

    	//get the successors
    	bool bSuccs = true;
    	GetRandomNeighs(currentState, SuccIDV, CLowV, nNumofNeighs, nDist_c, bSuccs);
    }

    void
    FootstepPlannerEnvironment::GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV)
    {
    	//number of random neighbors
    	int nNumofNeighs = 20;
    	//distance at which the neighbors are generated
    	double randomNeighborDist = 0.5;
    	int nDist_c = randomNeighborDist / ivCellSize;

    	assert(TargetStateID < ivStateId2State.size());
    	const PlanningState* currentState = ivStateId2State[TargetStateID];

    	//start state should be absorbing
    	if(closeToStart(*currentState))
    		return;

    	//get the predecessors
    	bool bSuccs = false;
    	GetRandomNeighs(currentState, PredIDV, CLowV, nNumofNeighs, nDist_c, bSuccs);

    }

    //generates nNumofNeighs random neighbors of cell <X,Y> at distance nDist_c (measured in cells)
    //it will also generate goal if within this distance as an additional neighbor
    //bSuccs is set to true if we are computing successor states, otherwise it is Preds
    // (see fct. implemented in environment_nav2D)
    void FootstepPlannerEnvironment::GetRandomNeighs(const PlanningState* currentState, std::vector<int>* NeighIDV, std::vector<int>* CLowV, int nNumofNeighs, int nDist_c, bool bSuccs)
    {

    	//clear the successor array
    	NeighIDV->clear();
    	CLowV->clear();

    	//get X, Y for the states
    	int X = currentState->getX();
    	int Y = currentState->getY();

    	//iterate through random actions
    	for (int i = 0, nAttempts = 0; i < nNumofNeighs && nAttempts < 5*nNumofNeighs; i++, nAttempts++)
    	{
    		int dX = 0;
    		int dY = 0;

    		//pick a direction
    		float fDir = (float)(2*PI_CONST*(((double)rand())/RAND_MAX));

    		//compute the successor that result from following this direction until one of the coordinates reaches the desired distance
    		//decide whether |dX| = dist or |dY| = dist
    		float fRadius = 0;
    		if(fabs(cos(fDir)) > fabs(sin(fDir)))
    		{
    			fRadius = (float)((nDist_c+0.5)/fabs(cos(fDir)));
    		}
    		else
    		{
    			fRadius = (float)((nDist_c+0.5)/fabs(sin(fDir)));
    		}

    		dX = (int)(fRadius*cos(fDir));
    		dY = (int)(fRadius*sin(fDir));

    		if((fabs((float)dX) < nDist_c && fabs((float)dY) < nDist_c) || fabs((float)dX) > nDist_c ||
    				fabs((float)dY) > nDist_c)
    		{
    			SBPL_ERROR("ERROR in EnvNav2D genneighs function: dX=%d dY=%d\n", dX, dY);
    			throw new SBPL_Exception();
    		}

    		//get the coords of the state
    		int newX = X + dX;
    		int newY = Y + dY;

    		// random theta: // TODO: pick better choice
    		int newTheta = rand() % ivNumAngleBins;
    		Leg newLeg = Leg(rand() % 2);
    		PlanningState random_state(newX, newY, newTheta, newLeg, ivCellSize, ivNumAngleBins, ivHashTableSize);

    		//skip the invalid cells
    		if(occupied(random_state))
    		{
    			i--;
    			continue;
    		}

    		//get the state
            const PlanningState* random_hash_entry = getHashEntry(random_state);
            if (random_hash_entry == NULL)
            	random_hash_entry = createNewHashEntry(random_state);

    		//compute clow
    		int clow;
    		if(bSuccs)
    			clow = GetFromToHeuristic(currentState->getId(), random_hash_entry->getId());
    		else
    			clow = GetFromToHeuristic(random_hash_entry->getId(), currentState->getId());

    		//insert it into the list
    		NeighIDV->push_back(random_hash_entry->getId());
    		CLowV->push_back(clow);

    	}

    	//see if the goal/start belongs to the inside area and if yes then add it to Neighs as well
    	// NOTE: "goal check" for backward planning
    	const PlanningState* goal_left = NULL;
    	const PlanningState* goal_right = NULL;
    	if (bSuccs){
    		goal_left = ivStateId2State[ivGoalFootIdLeft];
    		goal_right = ivStateId2State[ivGoalFootIdLeft];
    	} else {
    		goal_left = ivStateId2State[ivStartFootIdLeft];
    		goal_right = ivStateId2State[ivStartFootIdLeft];
    	}


    	//add left if within the distance
    	if(std::abs(goal_left->getX() - X) <= nDist_c && std::abs(goal_left->getY() - Y) <= nDist_c)
    	{
    		//compute clow
    		int clow;
    		int desstateID = goal_left->getId();
    		if(bSuccs)
    			clow = GetFromToHeuristic(currentState->getId(), desstateID);
    		else
    			clow = GetFromToHeuristic(desstateID, currentState->getId());

    		NeighIDV->push_back(desstateID);
    		CLowV->push_back(clow);
    	}

    	//add right if within the distance
    	if(std::abs(goal_right->getX() - X) <= nDist_c && std::abs(goal_right->getY() - Y) <= nDist_c)
    	{
    		//compute clow
    		int clow;
    		int desstateID = goal_right->getId();
    		if(bSuccs)
    			clow = GetFromToHeuristic(currentState->getId(), desstateID);
    		else
    			clow = GetFromToHeuristic(desstateID, currentState->getId());

    		NeighIDV->push_back(desstateID);
    		CLowV->push_back(clow);
    	}
    }

	bool
	FootstepPlannerEnvironment::AreEquivalent(int StateID1, int StateID2)
	{
		// TODO: check if needed --> NOTE: hash tags are not unique
		return (StateID1 == StateID2);
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

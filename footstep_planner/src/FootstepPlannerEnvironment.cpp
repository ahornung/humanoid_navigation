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


namespace footstep_planner{

    FootstepPlannerEnvironment::FootstepPlannerEnvironment(
            const  std::vector<Footstep>& footstep_set,
            const  boost::shared_ptr<Heuristic> heuristic,
            double footsize_x,
            double footsize_y,
            double origin_foot_shift_x,
            double origin_foot_shift_y,
            double max_footstep_x,
            double max_footstep_y,
            double max_footstep_theta,
            double max_inverse_footstep_x,
            double max_inverse_footstep_y,
            double max_inverse_footstep_theta,
            double step_cost,
            int    collision_check_accuracy,
            int    hash_table_size,
            double cell_size,
            int    num_angle_bins,
            bool   forward_search,
            int num_random_nodes,
            double random_node_distance,
            double heuristic_scale)
        : DiscreteSpaceInformation(),
          ivIdStartFootLeft(-1),
          ivIdStartFootRight(-1),
          ivIdGoalFootLeft(-1),
          ivIdGoalFootRight(-1),
          ivpStateHash2State(
                new std::vector<const PlanningState*>[hash_table_size]),
          ivFootstepSet(footstep_set),
          ivHeuristicConstPtr(heuristic),
          ivFootsizeX(footsize_x),
          ivFootsizeY(footsize_y),
          ivOriginFootShiftX(origin_foot_shift_x),
          ivOriginFootShiftY(origin_foot_shift_y),
          ivMaxFootstepX(disc_val(max_footstep_x, cell_size)),
          ivMaxFootstepY(disc_val(max_footstep_y, cell_size)),
          ivMaxFootstepTheta(
                angle_state_2_cell(max_footstep_theta, num_angle_bins)),
          ivMaxInvFootstepX(disc_val(max_inverse_footstep_x, cell_size)),
          ivMaxInvFootstepY(disc_val(max_inverse_footstep_y, cell_size)),
          ivMaxInvFootstepTheta(
                angle_state_2_cell(max_inverse_footstep_theta, num_angle_bins)),
          ivStepCost(cvMmScale * step_cost),
          ivCollisionCheckAccuracy(collision_check_accuracy),
          ivHashTableSize(hash_table_size),
          ivCellSize(cell_size),
          ivNumAngleBins(num_angle_bins),
          ivForwardSearch(forward_search),
          ivNumRandomNodes(num_random_nodes),
          ivRandomNodeDist(random_node_distance / ivCellSize),
          ivHeuristicScale(heuristic_scale),
          ivHeuristicExpired(true),
          ivNumExpandedStates(0)
    {
        int num_angle_bins_half = ivNumAngleBins / 2;
        if (ivMaxFootstepTheta >= num_angle_bins_half)
            ivMaxFootstepTheta -= ivNumAngleBins;
        if (ivMaxInvFootstepTheta >= num_angle_bins_half)
            ivMaxInvFootstepTheta -= ivNumAngleBins;
    }


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
    FootstepPlannerEnvironment::updateGoal(const State& foot_left,
                                           const State& foot_right)
    {
    	// keep the old IDs
        int goal_foot_id_left = ivIdGoalFootLeft;
        int goal_foot_id_right = ivIdGoalFootRight;

        // update the states for both feet (if necessary)
		const PlanningState* p_foot_left = getHashEntry(foot_left);
        if (p_foot_left == NULL)
        {
            p_foot_left = createNewHashEntry(foot_left);
            ivIdGoalFootLeft = p_foot_left->getId();
        }
        else
        {
            ivIdGoalFootLeft = p_foot_left->getId();
        }
        const PlanningState* p_foot_right = getHashEntry(foot_right);
        if (p_foot_right == NULL)
        {
            p_foot_right = createNewHashEntry(foot_right);
            ivIdGoalFootRight = p_foot_right->getId();
        }
        else
        {
            ivIdGoalFootRight = p_foot_right->getId();
        }

        // check if everything has been set correctly
        assert(ivIdGoalFootLeft != -1);
        assert(ivIdGoalFootRight != -1);

        // if using the forward search a change of the goal states involves an
        // update of the heuristic
        if (ivForwardSearch)
		{
        	// check if the goal states have been changed
			if (goal_foot_id_left != ivIdGoalFootLeft &&
				goal_foot_id_right != ivIdGoalFootRight)
			{
				ivHeuristicExpired = true;
			}
		}
    }


    void
    FootstepPlannerEnvironment::updateStart(const State& foot_left,
                                            const State& foot_right)
    {
    	// keep the old IDs
        int start_foot_id_left = ivIdStartFootLeft;
        int start_foot_id_right = ivIdStartFootRight;

        // update the states for both feet (if necessary)
        const PlanningState* p_foot_left = getHashEntry(foot_left);
        if (p_foot_left == NULL)
        {
            p_foot_left = createNewHashEntry(foot_left);
            ivIdStartFootLeft = p_foot_left->getId();
        }
        else
        {
            ivIdStartFootLeft = p_foot_left->getId();
        }
        const PlanningState* p_foot_right = getHashEntry(foot_right);
        if (p_foot_right == NULL)
        {
            p_foot_right = createNewHashEntry(foot_right);
            ivIdStartFootRight = p_foot_right->getId();
        }
        else
        {
            ivIdStartFootRight = p_foot_right->getId();
        }

        // check if everything has been set correctly
        assert(ivIdStartFootLeft != -1);
        assert(ivIdStartFootRight != -1);

        // if using the backward search a change of the start states involves an
        // update of the heuristic
        if (!ivForwardSearch)
        {
        	// check if the start states have been changed
			if (start_foot_id_left != ivIdStartFootLeft &&
				start_foot_id_right != ivIdStartFootRight)
			{
				ivHeuristicExpired = true;
			}
        }
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

        // insert the ID of the new state into the corresponding map
        new_state->setId(state_id);
        ivStateId2State.push_back(new_state);

        // insert the new state into the hash map at the corresponding position
        ivpStateHash2State[state_hash].push_back(new_state);

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
        for (state_iter = ivpStateHash2State[state_hash].begin();
             state_iter != ivpStateHash2State[state_hash].end();
             state_iter++)
        {
            if (*(*state_iter) == s)
                return *state_iter;
        }

        return NULL;
    }

    const PlanningState*
    FootstepPlannerEnvironment::createHashEntryIfNotExists(
            const PlanningState& s)
    {
    	const PlanningState* hash_entry = getHashEntry(s);
    	if (hash_entry == NULL)
    		hash_entry = createNewHashEntry(s);

    	return hash_entry;

    }


    int
    FootstepPlannerEnvironment::stepCost(const PlanningState& a,
	                                     const PlanningState& b)
    {
        if (a == b)
            return 0;

		// NOTE: instead of using cont_val() the calculation is done directly
		// here because cont_val() truncates the input length to int
		double dist = euclidean_distance(
		        a.getX(), a.getY(), b.getX(), b.getY()) * ivCellSize;

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
        if (id >= ivStateId2State.size())
            return false;

        const PlanningState* planning_state = ivStateId2State[id];
        s->setX(cell_2_state(planning_state->getX(), ivCellSize));
        s->setY(cell_2_state(planning_state->getY(), ivCellSize));
        s->setTheta(angles::normalize_angle(angle_cell_2_state(
                planning_state->getTheta(), ivNumAngleBins)));
        s->setLeg(planning_state->getLeg());

        return true;
    }


    void
    FootstepPlannerEnvironment::updateMap(gridmap_2d::GridMap2DPtr map)
    {
        ivMapPtr.reset();
        ivMapPtr = map;

        if (ivHeuristicConstPtr->getHeuristicType() == Heuristic::PATH_COST)
        {
            boost::shared_ptr<PathCostHeuristic> h =
            		boost::dynamic_pointer_cast<PathCostHeuristic>(
            				ivHeuristicConstPtr);
            h->updateMap(map);
        }
    }


    void
    FootstepPlannerEnvironment::updateHeuristicValues()
    {
        // check if start and goal have been set
        assert(ivIdGoalFootLeft != -1 && ivIdGoalFootRight != -1);
        assert(ivIdStartFootLeft != -1 && ivIdStartFootRight != -1);

        if (!ivHeuristicExpired)
            return;

        ROS_DEBUG("Updating the heuristic values.");

        if (ivHeuristicConstPtr->getHeuristicType() == Heuristic::PATH_COST)
        {
            boost::shared_ptr<PathCostHeuristic> h =
            		boost::dynamic_pointer_cast<PathCostHeuristic>(
            				ivHeuristicConstPtr);
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
        }

        ROS_DEBUG("Finished updating the heuristic values.");
        ivHeuristicExpired = false;
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
        ivNumExpandedStates = 0;
        ivRandomStates.clear();

        ivIdGoalFootLeft = -1;
        ivIdGoalFootRight = -1;
        ivIdStartFootLeft = -1;
        ivIdStartFootRight = -1;
    }


    bool
    FootstepPlannerEnvironment::closeToStart(const PlanningState& from)
    {
        // NOTE: "goal check" for backward planning
        const PlanningState* start;
        if (from.getLeg() == RIGHT)
            start = ivStateId2State[ivIdStartFootLeft];
        else
            start = ivStateId2State[ivIdStartFootRight];

        return reachable(*start, from);
    }


    bool
    FootstepPlannerEnvironment::closeToGoal(const PlanningState& from)
    {
        // NOTE: "goal check" for forward planning
        const PlanningState* goal;
        if (from.getLeg() == RIGHT)
            goal = ivStateId2State[ivIdGoalFootLeft];
        else
            goal = ivStateId2State[ivIdGoalFootRight];

        return reachable(from, *goal);
    }


    bool
    FootstepPlannerEnvironment::reachable(const PlanningState& from,
                                          const PlanningState& to)
    {
        // get the (continuous) orientation of state 'from'
        double orient = -(angle_cell_2_state(from.getTheta(), ivNumAngleBins));
        double orient_cos = cos(orient);
        double orient_sin = sin(orient);

        // calculate the footstep shift and rotate it into the 'from'-view
        int footstep_x = to.getX() - from.getX();
        int footstep_y = to.getY() - from.getY();
        double shift_x = footstep_x * orient_cos - footstep_y * orient_sin;
        double shift_y = footstep_x * orient_sin + footstep_y * orient_cos;
        footstep_x = round(shift_x);
        footstep_y = round(shift_y);

        // calculate the footstep rotation
        int footstep_theta = to.getTheta() - from.getTheta();

        // transform the value into [-ivNumAngleBins/2..ivNumAngleBins/2)
        int num_angle_bins_half = ivNumAngleBins / 2;
        if (footstep_theta >= num_angle_bins_half)
            footstep_theta -= ivNumAngleBins;
        else if (footstep_theta < -num_angle_bins_half)
            footstep_theta += ivNumAngleBins;

        // adjust for the left foot
        if (from.getLeg() == LEFT)
        {
            footstep_y = -footstep_y;
            footstep_theta = -footstep_theta;
        }
        return (footstep_x <= ivMaxFootstepX &&
                footstep_x >= ivMaxInvFootstepX &&
                footstep_y <= ivMaxFootstepY &&
                footstep_y >= ivMaxInvFootstepY &&
                footstep_theta <= ivMaxFootstepTheta &&
                footstep_theta >= ivMaxInvFootstepTheta);
    }


    void
    FootstepPlannerEnvironment::getPredsOfGridCells(
	        const std::vector<State>& changed_states,
	        std::vector<int>* pred_ids)
    {
    	pred_ids->clear();

		std::vector<State>::const_iterator state_iter;
		for (state_iter = changed_states.begin();
		     state_iter != changed_states.end();
		     state_iter++)
		{
			PlanningState s(*state_iter, ivCellSize, ivNumAngleBins,
			                ivHashTableSize);
            // generate predecessor planning states
			std::vector<Footstep>::const_iterator footstep_set_iter;
			for(footstep_set_iter = ivFootstepSet.begin();
                footstep_set_iter != ivFootstepSet.end();
                footstep_set_iter++)
			{
				PlanningState pred = footstep_set_iter->reverseMeOnThisState(s);
				// check if predecessor exists
				const PlanningState* pred_hash_entry = getHashEntry(pred);
				if (pred_hash_entry == NULL)
					continue;
				pred_ids->push_back(pred_hash_entry->getId());
			}
		}
    }


    void
    FootstepPlannerEnvironment::getSuccsOfGridCells(
	        const std::vector<State>& changed_states,
	        std::vector<int>* succ_ids)
    {
    	succ_ids->clear();

		std::vector<State>::const_iterator state_iter;
		for (state_iter = changed_states.begin();
		     state_iter != changed_states.end();
		     state_iter++)
		{
			PlanningState s(*state_iter, ivCellSize, ivNumAngleBins,
			                ivHashTableSize);
			// generate successors
			std::vector<Footstep>::const_iterator footstep_set_iter;
			for(footstep_set_iter = ivFootstepSet.begin();
			    footstep_set_iter != ivFootstepSet.end();
			    footstep_set_iter++)
			{
				PlanningState succ = footstep_set_iter->performMeOnThisState(s);
				// check if successor exists
				const PlanningState* succ_hash_entry = getHashEntry(succ);
				if (succ_hash_entry == NULL)
                    continue;
                succ_ids->push_back(succ_hash_entry->getId());
			}
		}
    }


    int
    FootstepPlannerEnvironment::GetFromToHeuristic(int FromStateID,
                                                   int ToStateID)
    {
    	assert(FromStateID >= 0 && (unsigned int) FromStateID < ivStateId2State.size());
    	assert(ToStateID >= 0 && (unsigned int) ToStateID < ivStateId2State.size());

    	if ((FromStateID == ivIdGoalFootLeft && ToStateID == ivIdGoalFootRight)
    					|| (FromStateID == ivIdGoalFootRight && ToStateID == ivIdGoalFootLeft)){
    		return 0;
    	}

    	const PlanningState* from = ivStateId2State[FromStateID];
    	const PlanningState* to = ivStateId2State[ToStateID];
//    	if (ivHeuristicConstPtr->getHeuristicType() == Heuristic::PATH_COST){
//    		boost::shared_ptr<PathCostHeuristic> pathCostHeuristic = boost::dynamic_pointer_cast<PathCostHeuristic>(ivHeuristicConstPtr);
//    		pathCostHeuristic->calculateDistances(*from, *to);
//    	}
    	return GetFromToHeuristic(*from, *to);
    }

    int
    FootstepPlannerEnvironment::GetFromToHeuristic(const PlanningState& from,
                                                   const PlanningState& to)
    {
    	return cvMmScale * ivHeuristicScale *
    	       ivHeuristicConstPtr->getHValue(from, to);
    }


    int
    FootstepPlannerEnvironment::GetGoalHeuristic(int stateID)
    {
        return GetFromToHeuristic(stateID, ivIdGoalFootLeft);
    }


    void
    FootstepPlannerEnvironment::GetPreds(int TargetStateID,
                                         std::vector<int> *PredIDV,
                                         std::vector<int> *CostV)
    {
        PredIDV->clear();
        CostV->clear();

        assert(TargetStateID >= 0 &&
               (unsigned int) TargetStateID < ivStateId2State.size());

        // make goal state absorbing (only left!)
        if (TargetStateID == ivIdStartFootLeft)
        	return;

        const PlanningState* current = ivStateId2State[TargetStateID];

        // add cheap transition from right to left, so right becomes an equivalent goal
        if (TargetStateID == ivIdStartFootRight && current->getLeg() == RIGHT)
        {
            PredIDV->push_back(ivIdStartFootLeft);
            CostV->push_back(ivStepCost);
            return;
        }

        ivExpandedStates.insert(std::pair<int,int>(current->getX(),
                                                   current->getY()));
        ivNumExpandedStates++;

        if (closeToStart(*current))
        {
            int start_id;
            if (current->getLeg() == RIGHT)
                start_id = ivIdStartFootLeft;
            else
                start_id = ivIdStartFootRight;

            const PlanningState* start = ivStateId2State[start_id];
            PredIDV->push_back(start_id);
            CostV->push_back(stepCost(*current, *start));

            return;
        }

        PredIDV->reserve(ivFootstepSet.size());
        CostV->reserve(ivFootstepSet.size());
        std::vector<Footstep>::const_iterator footstep_set_iter;
        for(footstep_set_iter = ivFootstepSet.begin();
            footstep_set_iter != ivFootstepSet.end();
            footstep_set_iter++)
        {
            const PlanningState predecessor =
            		footstep_set_iter->reverseMeOnThisState(*current);
            if (occupied(predecessor))
                continue;

            const PlanningState* predecessor_hash = createHashEntryIfNotExists(
                    predecessor);

            int cost = stepCost(*current, *predecessor_hash);
            PredIDV->push_back(predecessor_hash->getId());
            CostV->push_back(cost);
        }
    }


    int
    FootstepPlannerEnvironment::GetStartHeuristic(int stateID)
    {
        return GetFromToHeuristic(stateID, ivIdStartFootLeft);
    }


    void
    FootstepPlannerEnvironment::GetSuccs(int SourceStateID,
                                         std::vector<int> *SuccIDV,
                                         std::vector<int> *CostV)
    {
        SuccIDV->clear();
        CostV->clear();

        assert(SourceStateID >= 0 &&
               unsigned(SourceStateID) < ivStateId2State.size());

        // make goal state absorbing (only left!)
        if (SourceStateID == ivIdGoalFootLeft)
        	return;

        const PlanningState* current = ivStateId2State[SourceStateID];

        // add cheap transition from right to left, so right becomes an
        // equivalent goal
        if (SourceStateID == ivIdGoalFootRight && current->getLeg() == RIGHT)
        {
            SuccIDV->push_back(ivIdGoalFootLeft);
            CostV->push_back(ivStepCost);
            return;
        }

        ivExpandedStates.insert(std::pair<int,int>(current->getX(),
                                                   current->getY()));
        ivNumExpandedStates++;

        if (closeToGoal(*current))
        {
            int goal_id;
            assert(current->getLeg() != NOLEG);
            if (current->getLeg() == RIGHT)
                goal_id = ivIdGoalFootLeft;
            else
                goal_id = ivIdGoalFootRight;

            const PlanningState* goal = ivStateId2State[goal_id];
            SuccIDV->push_back(goal_id);
            CostV->push_back(stepCost(*current, *goal));

            return;
        }

        SuccIDV->reserve(ivFootstepSet.size());
        CostV->reserve(ivFootstepSet.size());
        std::vector<Footstep>::const_iterator footstep_set_iter;
        for(footstep_set_iter = ivFootstepSet.begin();
            footstep_set_iter != ivFootstepSet.end();
            footstep_set_iter++)
        {
            PlanningState successor =
            		footstep_set_iter->performMeOnThisState(*current);
            if (occupied(successor))
                continue;

            const PlanningState* successor_hash_entry =
                    createHashEntryIfNotExists(successor);

            int cost = stepCost(*current, *successor_hash_entry);
            SuccIDV->push_back(successor_hash_entry->getId());
            CostV->push_back(cost);
        }
    }

    void
    FootstepPlannerEnvironment::GetSuccsTo(int SourceStateID, int goalStateId,
                                         std::vector<int> *SuccIDV,
                                         std::vector<int> *CostV)
    {
        SuccIDV->clear();
        CostV->clear();

        assert(SourceStateID >= 0 &&
               unsigned(SourceStateID) < ivStateId2State.size());

        // make goal state absorbing
        if (SourceStateID == ivIdGoalFootLeft ){
        	return;
        }

        const PlanningState* current = ivStateId2State[SourceStateID];
        ivExpandedStates.insert(std::pair<int,int>(current->getX(), current->getY()));
        ivNumExpandedStates++;

        //ROS_INFO("GetSuccsTo %d -> %d: %f", SourceStateID, goalStateId, euclidean_distance(current->getX(), current->getY(), ivStateId2State[goalStateId]->getX(), ivStateId2State[goalStateId]->getY()));

        // add cheap transition from right to left, so right becomes an equivalent goal
        if (goalStateId== ivIdGoalFootLeft && SourceStateID == ivIdGoalFootRight && current->getLeg() == RIGHT){
          SuccIDV->push_back(ivIdGoalFootLeft);
          CostV->push_back(ivStepCost);
          return;
        }

        if (closeToGoal(*current))
        {
            int goal_id;
            assert(current->getLeg() != NOLEG);
            if (current->getLeg() == RIGHT){
                goal_id = ivIdGoalFootLeft;
            } else {
                goal_id = ivIdGoalFootRight;
            }

            const PlanningState* goal = ivStateId2State[goal_id];
            int cost = stepCost(*current, *goal);
            SuccIDV->push_back(goal_id);
            CostV->push_back(cost);

            return;
        }

        // intermediate goal reachable (R*)?
        assert(goalStateId >= 0 && unsigned(goalStateId) < ivStateId2State.size());
       	const PlanningState* randomGoal = ivStateId2State[goalStateId];
       	if (randomGoal->getLeg() != current->getLeg() && reachable(*current, *randomGoal)){
       		int cost = stepCost(*current, *randomGoal);
       		SuccIDV->push_back(goalStateId);
       		CostV->push_back(cost);
//       		ROS_INFO("%d %d", goalStateId, cost);

//       		return;
       	}


        SuccIDV->reserve(ivFootstepSet.size());
        CostV->reserve(ivFootstepSet.size());
        std::vector<Footstep>::const_iterator footstep_set_iter;
        for(footstep_set_iter = ivFootstepSet.begin();
            footstep_set_iter != ivFootstepSet.end();
            footstep_set_iter++)
        {
            PlanningState successor =
            		footstep_set_iter->performMeOnThisState(*current);
            if (occupied(successor))
                continue;

            const PlanningState* successor_hash = createHashEntryIfNotExists(successor);

            int cost = stepCost(*current, *successor_hash);
            SuccIDV->push_back(successor_hash->getId());
            CostV->push_back(cost);
        }
    }


    void
    FootstepPlannerEnvironment::GetRandomSuccsatDistance(int SourceStateID,
    		std::vector<int>* SuccIDV, std::vector<int>* CLowV)
    {

    	assert(SourceStateID >= 0 && unsigned(SourceStateID) < ivStateId2State.size());
    	//goal state should be absorbing
    	if (SourceStateID == ivIdGoalFootLeft || SourceStateID == ivIdGoalFootRight )
    		return;


    	const PlanningState* currentState = ivStateId2State[SourceStateID];
    	// TODO: closeToGoal?
//
//    	if (closeToGoal(*currentState))
//    		return;

    	//get the successors
    	GetRandomNeighs(currentState, SuccIDV, CLowV, ivNumRandomNodes,
		                ivRandomNodeDist, true);
    }

    void
    FootstepPlannerEnvironment::GetRandomPredsatDistance(int TargetStateID,
    		std::vector<int>* PredIDV, std::vector<int>* CLowV)
    {

    	assert(TargetStateID >= 0 && unsigned(TargetStateID) < ivStateId2State.size());

    	//start state should be absorbing
    	if (TargetStateID == ivIdStartFootLeft || TargetStateID == ivIdStartFootRight )
    		return;

    	const PlanningState* currentState = ivStateId2State[TargetStateID];

    	// TODO: ???
//    	if(closeToStart(*currentState))
//    		return;

    	//get the predecessors
    	GetRandomNeighs(currentState, PredIDV, CLowV, ivNumRandomNodes,
		                ivRandomNodeDist, false);

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
    	//int theta = currentState->getTheta();

    	//see if the goal/start belongs to the inside area and if yes then add it to Neighs as well
    	// NOTE: "goal check" for backward planning
    	const PlanningState* goal_left = NULL;
    	const PlanningState* goal_right = NULL;
    	if (bSuccs){
    		goal_left = ivStateId2State[ivIdGoalFootLeft];
    		goal_right = ivStateId2State[ivIdGoalFootRight];
    	} else {
    		goal_left = ivStateId2State[ivIdStartFootLeft];
    		goal_right = ivStateId2State[ivIdStartFootRight];
    	}

    	int nDist_sq = nDist_c*nDist_c;

    	//add left if within the distance
    	if (euclidean_distance_sq(X, Y, goal_left->getX(), goal_left->getY()) <= nDist_sq)
    	{
    		//compute clow
    		int clow;
    		if(bSuccs)
    			clow = GetFromToHeuristic(*currentState, *goal_left);
    		else
    			clow = GetFromToHeuristic(*goal_left, *currentState);

    		NeighIDV->push_back(goal_left->getId());
    		CLowV->push_back(clow);
    		ivRandomStates.push_back(goal_left->getId());
    	}

    	//add right if within the distance
    	if(euclidean_distance_sq(X, Y, goal_right->getX(), goal_right->getY()) <= nDist_sq)
    	{
    		//compute clow
    		int clow;
    		if(bSuccs)
    			clow = GetFromToHeuristic(*currentState, *goal_right);
    		else
    			clow = GetFromToHeuristic(*goal_right, *currentState);

    		NeighIDV->push_back(goal_right->getId());
    		CLowV->push_back(clow);
    		ivRandomStates.push_back(goal_right->getId());
    	}

    	//iterate through random actions
    	int nAttempts = 0;
    	for (int i = 0; i < nNumofNeighs && nAttempts < 5*nNumofNeighs; i++, nAttempts++)
    	{

    		// pick goal in random direction
    		float fDir = (float)(TWO_PI*(((double)rand())/RAND_MAX));

    		int dX = (int)(nDist_c*cos(fDir));
    		int dY = (int)(nDist_c*sin(fDir));

    		//get the coords of the state
    		int newX = X + dX;
    		int newY = Y + dY;

    		// TODO / FIXME x,y, can be negative! need offset
    		// check if outside of map:
//    		if (newX < 0 || newY < 0 || unsigned(newX) >= ivMapPtr->getInfo().width || unsigned(newY) >= ivMapPtr->getInfo().height){
//    			i--;
//    			ROS_INFO("Outside of map: %d %d", newX, newY);
//    			continue;
//    		}

    		// direction of random exploration (facing forward):
    		int newTheta = angle_state_2_cell(fDir, ivNumAngleBins);

    		// random left/right
    		Leg newLeg = Leg(rand() % 2);

    		PlanningState randomState(newX, newY, newTheta, newLeg, ivHashTableSize);

    		// add both left and right if available:
//    		int sep = disc_val(0.07, ivCellSize);
//    		int ddX = int(-sin(fDir) * sep);
//    		int ddY = int(cos(fDir) * sep);
//    		PlanningState randomState(newX+ddX, newY+ddY, newTheta, LEFT, ivHashTableSize);
//
//    		PlanningState randomStateR(newX-ddX, newY-ddY, newTheta, RIGHT, ivHashTableSize);

    		if(!occupied(randomState))
    		{
    			const PlanningState* random_hash_entry = getHashEntry(randomState);
    			if (random_hash_entry == NULL){
    				random_hash_entry = createNewHashEntry(randomState);
    				ivRandomStates.push_back(random_hash_entry->getId());
    			}

    			//compute clow
    			int clow;
    			if(bSuccs)
    				clow = GetFromToHeuristic(currentState->getId(), random_hash_entry->getId());

    			else
    				clow = GetFromToHeuristic(random_hash_entry->getId(), currentState->getId());

    			NeighIDV->push_back(random_hash_entry->getId());
    			CLowV->push_back(clow);

    		}else{
    			i--;
    		}

//    		if(!occupied(randomStateR))
//    		{
//    			const PlanningState* random_hash_entry = getHashEntry(randomStateR);
//    			if (random_hash_entry == NULL){
//    				random_hash_entry = createNewHashEntry(randomStateR);
//    				ivRandomStates.push_back(random_hash_entry->getId());
//    			}
//
//    			//compute clow
//    			int clow;
//    			if(bSuccs)
//    				clow = GetFromToHeuristic(currentState->getId(), random_hash_entry->getId());
//    			else
//    				clow = GetFromToHeuristic(random_hash_entry->getId(), currentState->getId());
//
//    			NeighIDV->push_back(random_hash_entry->getId());
//    			CLowV->push_back(clow);
//
//    		}else{
//    			i--;
//    		}


    	}

    	if (NeighIDV->size() == 0){
    	  ROS_WARN("Could not create any random neighbor nodes (%d attempts) from id %d (%d %d)",
    	       nAttempts, currentState->getId(), X, Y);
    	} else

          ROS_DEBUG("Created %zu random neighbors (%d attempts) from id %d "
                    "(%d %d)", NeighIDV->size(), nAttempts, currentState->getId(),
                    X, Y);
    }

	bool
	FootstepPlannerEnvironment::AreEquivalent(int StateID1, int StateID2)
	{
		assert(StateID1 >= 0 && StateID2 >= 0
				&& unsigned(StateID1) < ivStateId2State.size() && unsigned(StateID2) < ivStateId2State.size());


		if (StateID1 == StateID2)
			return true;

		const PlanningState* s1 = ivStateId2State[StateID1];
		const PlanningState* s2 = ivStateId2State[StateID2];

//		// approximately compare, ignore theta:
		return (std::abs(s1->getX() - s2->getX()) < 1
			                && std::abs(s1->getY() - s2->getY()) < 1
//			                && std::abs(s1->getTheta() - s2->getTheta()) < 3
			                && s1->getLeg() == s2->getLeg()
			                );


		// compare the actual values (exact comparison)
//		return (*s1 == *s2);
	}



    bool
    FootstepPlannerEnvironment::InitializeEnv(const char *sEnvFile)
    {
//        ROS_ERROR("FootstepPlanerEnvironment::InitializeEnv: Hit "
//		          "unimplemented function. Check this!");
        return true;
    }


    bool
    FootstepPlannerEnvironment::InitializeMDPCfg(MDPConfig *MDPCfg)
    {
        // NOTE: The internal start and goal ids are set here to the left foot
        // (this affects the calculation of the heuristic values)
        MDPCfg->goalstateid = ivIdGoalFootLeft;
        MDPCfg->startstateid = ivIdStartFootLeft;

        assert(ivIdGoalFootLeft != -1);
        assert(ivIdStartFootLeft != -1);

        return true;
    }


    void
    FootstepPlannerEnvironment::PrintEnv_Config(FILE *fOut)
    {
        // NOTE: implement this if the planner needs to print out configurations
        ROS_ERROR("FootstepPlanerEnvironment::PrintEnv_Config: Hit "
		          "unimplemented function. Check this!");
    }


    void
    FootstepPlannerEnvironment::PrintState(int stateID, bool bVerbose,
                                           FILE *fOut)
    {
        if(fOut == NULL)
        {
            fOut = stdout;
        }

        if(stateID == ivIdGoalFootLeft && bVerbose)
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

        ROS_ERROR("FootstepPlannerEnvironment::SetAllActionsandAllOutcomes: Hit"
                  " unimplemented function. Check this!");
    }


    void
    FootstepPlannerEnvironment::SetAllPreds(CMDPSTATE *state)
    {
        // NOTE: not implemented so far
        // Description: Some searches may also use SetAllActionsandAllOutcomes
        // or SetAllPreds functions if they keep the pointers to successors
        // (predecessors) but most searches do not require this, so it is not
        // necessary to support this

        ROS_ERROR("FootstepPlannerEnvironment::SetAllPreds: Hit unimplemented "
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


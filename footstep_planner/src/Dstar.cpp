// SVN $HeadURL$
// SVN $Id$

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

#include <footstep_planner/Dstar.h>


namespace footstep_planner
{

	/*
	 * #############################################################################
	 * #### class Dstar
	 * #############################################################################
	 */

	int
	Dstar::cvRoundingThreshold = 5;


	Dstar::Dstar(const std::vector<Footstep>& footstepSet,
		         const float footSeparation,
		         const float footOriginShiftX,
		         const float footOriginShiftY,
		         const float footsizeX,
		         const float footsizeY,
		         const float footMaxStepX,
		         const float footMaxStepY,
		         const float footMaxStepTheta,
		         const float footMaxInverseStepY,
		         const float footMaxInverseStepTheta,
		         const float maxStepWidth,
		         const float stepCosts,
		         const int   collisionCheckAccuracy,
		         const int   roundingThreshold,
		         const int   plannerMaxSteps,
		         const boost::shared_ptr<Heuristic> heuristicConstPtr)
		: ivFootstepSet(footstepSet),
		  ivFootSeparation(footSeparation),
		  ivFootOriginShiftX(footOriginShiftX),
		  ivFootOriginShiftY(footOriginShiftY),
		  ivFootsizeX(footsizeX),
		  ivFootsizeY(footsizeY),
		  ivFootMaxStepX(footMaxStepX),
		  ivFootMaxStepY(footMaxStepY),
		  ivFootMaxStepTheta(footMaxStepTheta),
		  ivFootMaxInverseStepY(footMaxInverseStepY),
		  ivFootMaxInverseStepTheta(footMaxInverseStepTheta),
		  ivMaxStepWidth(maxStepWidth),
		  ivStepCosts(stepCosts),
		  ivCollisionCheckAccuracy(collisionCheckAccuracy),
		  ivPlannerMaxSteps(plannerMaxSteps),
		  ivHeuristicConstPtr(heuristicConstPtr)
	{

		cvRoundingThreshold = roundingThreshold;

		ivKM = 0;
		ivPathCosts = 0;

	}


	Dstar::~Dstar()
	{}


	float
	Dstar::keyHashCode(const State& u) const
	{

		std::pair<float,float> key = u.getKey();
		return (float)(key.first + 1193*key.second);

	}


	bool
	Dstar::isValid(const State& u) const
	{

		openhash_type::const_iterator cur = ivOpenHash.find(u);

		if (cur == ivOpenHash.end())
			return false;
		if (!close(keyHashCode(u), cur->second))
			return false;

		return true;

	}


	bool
	Dstar::occupied(const State& u) const
	{

		float cosTheta = cos(u.getTheta());
		float sinTheta = sin(u.getTheta());
		float xShift = cosTheta*ivFootOriginShiftX - sinTheta*ivFootOriginShiftY;
		float yShift;

		if (u.getLeg() == LEFT)
			yShift = sinTheta*ivFootOriginShiftX + cosTheta*ivFootOriginShiftY;
		else // leg == RLEG
			yShift = sinTheta*ivFootOriginShiftX - cosTheta*ivFootOriginShiftY;

		return collisionCheck(u.getX() + xShift,
							  u.getY() + yShift,
							  u.getTheta(),
							  ivFootsizeX,
							  ivFootsizeY,
							  ivCollisionCheckAccuracy,
							  *ivMapPtr);

	}


	void
	Dstar::updateDistanceMap(boost::shared_ptr<GridMap2D> map)
	{

		bool exists = ivMapPtr;

		ivMapPtr.reset();
		ivMapPtr = map;

		if (ivHeuristicConstPtr->getHeuristicType() == Heuristic::ASTAR_PATH)
		{
			boost::shared_ptr<AstarHeuristic> h;
			h = boost::dynamic_pointer_cast<AstarHeuristic>(ivHeuristicConstPtr);
			h->setMap(map);
		}

		if (exists)
		{
			// TODO: implement updating of the edges efficiently
		}

	}


	bool
	Dstar::updateHeuristicValues()
	{

		if (ivHeuristicConstPtr->getHeuristicType() == Heuristic::ASTAR_PATH)
		{
			boost::shared_ptr<AstarHeuristic> h;
			h = boost::dynamic_pointer_cast<AstarHeuristic>(ivHeuristicConstPtr);
			// NOTE: start state is set to left leg; IMPORTANT: planning from goal to start!
			bool success = h->astarPlanning(ivGoal, ivStart);

			if (!success)
			{
				ROS_ERROR("Failed to generate A* subgoal path");
				return false;
			}

			ROS_INFO("Finished generating A* subgoal path");
		}

		return true;

	}


	bool
	Dstar::setUp(const State& startFootLeft, const State& startFootRight, const State& goal)
	{

		if (!ivMapPtr)
		{
			ROS_ERROR("Distance map hasn't been initialized yet.");
			return false;
		}

		ROS_INFO("D* lite initializing");

		// reset the planner
		reset();

		// set the start feet
		ivStartStateLeft = startFootLeft;
		ivStartStateRight = startFootRight;

		// set the state the search wants to reach
		float leftFootDist = ivMapPtr->distanceMapAt(ivStartStateLeft.getX(), ivStartStateLeft.getY());
		float rightFootDist = ivMapPtr->distanceMapAt(ivStartStateRight.getX(), ivStartStateRight.getY());
		if (leftFootDist > rightFootDist)
			ivStart = ivStartStateLeft;
		else
			ivStart = ivStartStateRight;
//		// NOTE: the state's leg is set to left
//		ivStart = ivStartStateLeft;

		// set the goal state; NOTE: the state's leg is set to right
		ivGoal = goal;

		// update the heuristic values; NOTE: be sure 'ivStart' has been set before
		updateHeuristicValues();

		State::state_info tmp;
		tmp.g = INFINITY;
		tmp.rhs =  0;
		ivStateHash[ivGoal] = tmp;
		insert(ivGoal);

		tmp.g = tmp.rhs = INFINITY;
		ivStateHash[ivStart] = tmp;
		insert(ivStart);

		ivLast = ivStart;

		return true;

	}


	void Dstar::reset()
	{

		ivStateHash.clear();
		ivPath.clear();
		ivOpenHash.clear();

		while (!ivOpenList.empty())
			ivOpenList.pop();

		ivPathCosts = 0;
		ivKM = 0;

	}


	void
	Dstar::addState(const State& u)
	{

	//	if (ivStateHash.find(u) != ivStateHash.end()) return;
	//
		State::state_info tmp;
		tmp.g = tmp.rhs = INFINITY;

		//	ivStateHash[u] = tmp;
		// new insertion and search in one (if not exists):

		ivStateHash.insert(std::pair<State, State::state_info>(u,tmp));

	}


	float
	Dstar::getG(const State& u) const
	{

		if (ivStateHash.find(u) == ivStateHash.end())
			return INFINITY;

		return ivStateHash.at(u).g;

	}


	float
	Dstar::getRhs(const State& u) const
	{

		if (u == ivGoal)
			return 0;

		if (ivStateHash.find(u) == ivStateHash.end())
			return INFINITY;

		return ivStateHash.at(u).rhs;

	}


	void
	Dstar::setG(const State& u, float g)
	{

		if (ivStateHash.find(u) == ivStateHash.end())
			addState(u);

		ivStateHash[u].g = g;

	}


	void
	Dstar::setRhs(const State& u, float rhs)
	{

		if (ivStateHash.find(u) == ivStateHash.end())
			addState(u);

		ivStateHash[u].rhs = rhs;

	}


	int
	Dstar::computeShortestPath()
	{

		std::vector<State> s;
		std::vector<State>::iterator i;

		int k=0;
		while ((!ivOpenList.empty()) &&
				(ivOpenList.top() < ivStart || getRhs(ivStart) > getG(ivStart)))
		{
			if (k++ > ivPlannerMaxSteps)
				return -1;

			State u;

			bool test = (getRhs(ivStart) < getG(ivStart));

			// lazy remove
			while(1)
			{
				u = ivOpenList.top();
				ivOpenList.pop();

				if (!isValid(u))
					continue;
				if (!(u < ivStart) && (!test))
					return 2;
				break;
			}

			ivExpandedStates.push_back(u);

			bool closeToStart = isCloseToStart(u);

			if (closeToStart)
			{
				ivKM += ivHeuristicConstPtr->getHValue(u, ivStart);
				ivStart = u;
			}

			State::key k_old = u.getKey();
			State::key k_new = calculateKey(u);

			if (k_old < k_new) // u is out of date
			{
				insert(u);
			}
			else if (getG(u) > getRhs(u)) // needs update (got better)
			{
				setG(u, getRhs(u));
				remove(u);
				getPredecessors(u, &s);
				float uGValue = getG(u);
				for (i=s.begin(); i != s.end(); i++)
				{
					float tmp;
					float tmp2;
					if (*i != ivGoal)
					{
						tmp = getRhs(*i);
						tmp2 = uGValue;
						tmp2 += cost(*i, u);
						if (tmp > tmp2)
							tmp = tmp2;
						if (!close(getRhs(*i),tmp))
							setRhs(*i, tmp);
					}
					update(*i);
				}
			}
			else // g <= rhs, state has got worse
			{
				float g_old = getG(u);
				setG(u, INFINITY);
				getPredecessors(u, &s);
				for (i=s.begin(); i != s.end(); i++)
				{
					updateRhs(*i, u, g_old);
					update(*i);
				}
				updateRhs(u, u, g_old);
				update(u);
			}

			ivStart.setKey(calculateKey(ivStart));
		}

		return 0;

	}


	void
	Dstar::updateRhs(const State& predecessor, const State& successor, float g_old)
	{

		std::vector<State> s;
		std::vector<State>::iterator i;

		float rhsVal = getRhs(predecessor);
		float gVal = g_old;
		gVal += cost(predecessor, successor);
		if (close(rhsVal, gVal))
		{
			if (predecessor != ivGoal)
			{
				getSuccessors(predecessor, &s);
				float tmp = INFINITY;
				float tmp2;

				for (i=s.begin(); i != s.end(); i++)
				{
					tmp2 = getG(*i);
					tmp2 += cost(predecessor, *i);
					if (tmp2 < tmp)
						tmp = tmp2;
				}

				if (!close(getRhs(predecessor),tmp))
					setRhs(predecessor, tmp);
			}
		}

	}


	void
	Dstar::update(State& u)
	{

		openhash_type::iterator cur = ivOpenHash.find(u);

		bool closeState = close(getG(u), getRhs(u));
		if (!closeState)
			// insert provides the same functionality as update
			insert(u);
		else
			// if u \in openHash will be checked in the remove method
			remove(u);

	}


	void
	Dstar::insert(State& u)
	{

	//	openhash_type::iterator cur;
		float csum;

		u.setKey(calculateKey(u));
		csum = keyHashCode(u);
	//	cur  = ivOpenHash.find(u);
		// return if cell is already in list. TODO: this should be
		// uncommented except it introduces a bug, I suspect that there is a
		// bug somewhere else and having duplicates in the openList queue
		// hides the problem...
		//if ((cur != ivOpenHash.end()) && (close(csum,cur->second))) return;

		ivOpenHash[u] = csum;
		ivOpenList.push(u);

	}


	void
	Dstar::remove(const State& u)
	{

		openhash_type::iterator cur = ivOpenHash.find(u);
		if (cur == ivOpenHash.end())
			return;
		ivOpenHash.erase(cur);

	}


	State::key
	Dstar::calculateKey(const State& u) const
	{

		float val = fmin(getRhs(u), getG(u));

		State::key key;
		key.first  = val + ivHeuristicConstPtr->getHValue(u, ivStart) + ivKM;
		key.second = val;

		return key;

	}


	float
	Dstar::cost(const State& a, const State& b) const
	{

		if (a == b)
			return 0;

		float dist = euclideanDistance(a.getX(), a.getY(), b.getX(), b.getY(), cvRoundingThreshold);
		return dist + ivStepCosts;

	}


	void
	Dstar::getSuccessors(const State& u, std::vector<State>* s)
	{

		State succ;

		s->clear();
		State::key key;
		key.first = -1;
		key.second = -1;
		succ.setKey(key);

		if (occupied(u))
			return;

		std::vector<Footstep>::const_iterator iter = ivFootstepSet.begin();
		for(; iter != ivFootstepSet.end(); iter++)
		{
			iter->performMeOnThisState(u, &succ, ivFootSeparation);
			s->push_back(succ);
		}

	}


	void
	Dstar::getPredecessors(const State& u, std::vector<State>* s)
	{

		s->clear();

		State pred;
		State::key key;
		key.first = -1;
		key.second = -1;
		pred.setKey(key);

		std::vector<Footstep>::const_iterator iter = ivFootstepSet.begin();
		for (; iter != ivFootstepSet.end(); iter++)
		{
			iter->revertMeOnThisState(u, &pred, ivFootSeparation);
			if (!occupied(pred))
			{
				s->push_back(pred);
			}
		}

	}


	bool
	Dstar::isCloseToStart(const State& s) const
	{

		if (s.getLeg() == RIGHT)
			return reachable(ivStartStateLeft, s);
		else // leg == LEFT
			return reachable(ivStartStateRight, s);

	}


	bool
	Dstar::reachable(const State& from, const State& to) const
	{

		bool xInRange = false;
		bool yInRange = false;
		bool thetaInRange = false;

		tf::Transform fromTransform(tf::createQuaternionFromYaw(from.getTheta()),
									tf::Point(from.getX(), from.getY(), 0));
		tf::Transform toTransform(tf::createQuaternionFromYaw(to.getTheta()),
								  tf::Point(to.getX(), to.getY(), 0));
		tf::Transform footstep;
		getFootstep(from.getLeg(), ivFootSeparation, fromTransform, toTransform, &footstep);

		float diffX = footstep.getOrigin().x();
		float diffY = footstep.getOrigin().y();
		float diffTheta = tf::getYaw(footstep.getRotation());

		if (diffX <= ivFootMaxStepX+FLOAT_COMP_THR &&  diffX >= -ivFootMaxStepX-FLOAT_COMP_THR)
			xInRange = true;
		if (from.getLeg() == RIGHT)
		{
			if (diffY <= ivFootMaxStepY+FLOAT_COMP_THR && diffY >= -ivFootMaxInverseStepY-FLOAT_COMP_THR)
				yInRange = true;
			if (diffTheta <= ivFootMaxStepTheta+ANGLE_COMP_THR && diffTheta >= -ivFootMaxInverseStepTheta-ANGLE_COMP_THR)
				thetaInRange = true;
		}
		else // leg == LEFT
		{
			if (diffY >= -ivFootMaxStepY-FLOAT_COMP_THR && diffY <= ivFootMaxInverseStepY+FLOAT_COMP_THR)
				yInRange = true;
			if (diffTheta >= -ivFootMaxStepTheta-ANGLE_COMP_THR && diffTheta <= ivFootMaxInverseStepTheta+ANGLE_COMP_THR)
				thetaInRange = true;
		}

		return xInRange && yInRange && thetaInRange;

	}


	bool
	Dstar::closeSteps(Leg stepLeg, tf::Transform& supportFoot, tf::Transform& footPlacement)
	{

		bool xInRange;
		bool yInRange;
		bool thetaInRange;

		tf::Transform current;
		tf::Vector3 translation;

		xInRange = false;
		yInRange = false;
		thetaInRange = false;

		tf::Quaternion rotation = footPlacement.getRotation().inverse() * supportFoot.getRotation();
		if (stepLeg == RIGHT)
			translation = tf::Vector3(0, ivFootSeparation/2, 0);
		else // stepLeg == LEFT
			translation = tf::Vector3(0, -ivFootSeparation/2, 0);
		current = footPlacement * tf::Transform(rotation, translation);
		current *= tf::Transform(tf::createQuaternionFromYaw(0.0), translation);
		current = supportFoot.inverse() * current;

		float diffX = current.getOrigin().x();
		float diffY = current.getOrigin().y();
		float diffAngle = -tf::getYaw(rotation);

		if (diffX <= ivFootMaxStepX+FLOAT_COMP_THR && diffX >= -ivFootMaxStepX-FLOAT_COMP_THR)
			xInRange = true;
		if (stepLeg == RIGHT)
		{
			if (diffY >= -ivFootMaxStepY-FLOAT_COMP_THR && diffY <= ivFootMaxInverseStepY+FLOAT_COMP_THR)
				yInRange = true;
			if (diffAngle >= -ivFootMaxStepTheta-FLOAT_COMP_THR && diffAngle <= ivFootMaxInverseStepTheta+FLOAT_COMP_THR)
				thetaInRange = true;
		}
		else // leg == LEFT
		{
			if (diffY <= ivFootMaxStepY+FLOAT_COMP_THR && diffY >= -ivFootMaxInverseStepY-FLOAT_COMP_THR)
				yInRange = true;
			if (diffAngle <= ivFootMaxStepTheta+FLOAT_COMP_THR && diffAngle >= -ivFootMaxInverseStepTheta-FLOAT_COMP_THR)
				thetaInRange = true;
		}

		return xInRange && yInRange && thetaInRange;

	}


	bool
	Dstar::isCloseToGoal(const State& s) const
	{

		return s == ivGoal;

	}


	void
	Dstar::updateStart(const State& startFootLeft, const State& startFootRight)
	{

		ROS_INFO("D* lite start updated to (%f %f %f), (%f %f %f)",
				 startFootLeft.getX(), startFootLeft.getY(), startFootLeft.getTheta(),
				 startFootRight.getX(), startFootRight.getY(), startFootRight.getTheta());

		ivStartStateLeft = startFootLeft;
		ivStartStateRight = startFootRight;

		float leftFootDist = ivMapPtr->distanceMapAt(startFootLeft.getX(), startFootLeft.getY());
		float rightFootDist = ivMapPtr->distanceMapAt(startFootRight.getX(), startFootRight.getY());
		if (leftFootDist <= rightFootDist)
			ivStart = ivStartStateLeft;
		else
			ivStart = ivStartStateRight;
//		// NOTE: the state's leg is set to left
//		ivStart = ivStartStateLeft;

		ivKM += ivHeuristicConstPtr->getHValue(ivLast, ivStart);
		ivStart.setKey(calculateKey(ivStart));

		ivLast = ivStart;

	}


	void
	Dstar::setGoal(const State& goal)
	{

		ROS_INFO("D* lite goal updated to (%f %f %f)", goal.getX(), goal.getY(), goal.getTheta());

		// reset the planner so a new planning without any previously collected
		// information can be started
		reset();

		// set the goal state; NOTE: the state's leg is set to right
		ivGoal = goal;
		updateHeuristicValues();

		State::state_info tmp;
		tmp.g = INFINITY;
		tmp.rhs =  0;
		ivStateHash[ivGoal] = tmp;
		insert(ivGoal);

		tmp.g = tmp.rhs = INFINITY;
		ivStateHash[ivStart] = tmp;
		insert(ivStart);

		ivLast = ivStart;

	}


	bool
	Dstar::replan()
	{

		if (!ivMapPtr)
		{
			ROS_ERROR("Distance map hasn't been initialized yet.");
			return false;
		}

		ivPath.clear();
		ivExpandedStates.clear();

		ivPathCosts = 0;

		ROS_INFO("Start path planning.");

		ros::Time startTime = ros::Time::now();
		int res = computeShortestPath();

		if (res < 0)
		{
			ROS_ERROR("Path planning failed, reached max expansions(%d)", ivPlannerMaxSteps);
			return false;
		}
		else if (isinf(getG(ivStart)))
		{
			ROS_ERROR("Path planning failed. Start state still locally inconsistent.");
			return false;
		}
		else
		{
			ROS_INFO("Path planning successful. Start extracting path now..");
		}

		State cur = ivStart;
		State succ = cur;

		// reset start state to its initialized original values because the start
		// values might have changed during the planning process
		ivStart = ivLast;

		int k = 0;
		while(!isCloseToGoal(cur))
		{
			// NOTE: the robot is not supposed to perform more than 100 footsteps
			if (k++ > 10000)
			{
				ROS_ERROR("Extracting path failed.");
				ivPath.clear();
				return false;
			}

			ivPath.push_back(cur);

			if (!getMinSucc(cur, &succ))
			{
				ROS_ERROR("No successor state. Extracting path failed.");
				ivPath.clear();
				return false;
			}
			ivPathCosts += cost(cur, succ);

			cur = succ;
		}
		ivPath.push_back(cur);

		// move other foot next to last moved foot
		Footstep neutralStep;
		neutralStep.performMeOnThisState(cur, &succ, ivFootSeparation);
		if (!occupied(succ))
			ivPath.push_back(succ);

		ROS_INFO("Path successfully extracted.");

		return true;

	}


	bool
	Dstar::getMinSucc(const State u, State* succ)
	{

		std::vector<State> s;
		std::vector<State>::iterator i;

		getSuccessors(u, &s);
		if (s.size() <= 0)
			return false;

		float g;
		float min = INFINITY;
		for (i = s.begin(); i != s.end(); i++)
		{
			g = cost(u, *i);
			g += getG(*i);

			if (g < min)
			{
				min = g;
				*succ = *i;
			}
		}

		return true;

	}


	/*
	 * #########################################################################
	 * ### class Footstep
	 * #########################################################################
	 */

	Footstep::Footstep()
	{

		ivX = 0;
		ivY = 0;
		ivTheta = 0;
		ivLeg = NOLEG;

	}


	Footstep::Footstep(float x, float y, float theta, Leg leg)
		: ivX(x), ivY(y), ivTheta(theta), ivLeg(leg)
	{}


	Footstep::~Footstep()
	{}


	void
	Footstep::performMeOnThisState(const State& current, State* successor, float footSeparation)
	const
	{

		float globalX = current.getX();
		float globalY = current.getY();
		float globalTheta = current.getTheta();

		float footSeparationHalf = footSeparation/2.0;

		float thetaCos = cos(globalTheta);
		float thetaSin = sin(globalTheta);

		if (current.getLeg() == RIGHT)
		{
			float xShift = thetaCos*ivX - thetaSin*(ivY+footSeparationHalf);
			float yShift = thetaSin*ivX + thetaCos*(ivY+footSeparationHalf);
			globalX += xShift;
			globalY += yShift;
			globalTheta += ivTheta; // NOTE: new globalTheta

			xShift = -sin(globalTheta) * footSeparationHalf;
			yShift =  cos(globalTheta) * footSeparationHalf;
			globalX += xShift;
			globalY += yShift;

			successor->setLeg(LEFT);
		}
		else // leg == LEFT
		{
			float xShift = thetaCos*ivX + thetaSin*(ivY+footSeparationHalf);
			float yShift = thetaSin*ivX - thetaCos*(ivY+footSeparationHalf);
			globalX += xShift;
			globalY += yShift;
			globalTheta -= ivTheta; // NOTE: new globalTheta

			xShift =  sin(globalTheta) * footSeparationHalf;
			yShift = -cos(globalTheta) * footSeparationHalf;
			globalX += xShift;
			globalY += yShift;

			successor->setLeg(RIGHT);
		}

		successor->setX(globalX);
		successor->setY(globalY);
		successor->setTheta(globalTheta);

	}


	void
	Footstep::revertMeOnThisState(const State& current, State* predecessor, float footSeparation)
	const
	{

		float globalX = current.getX();
		float globalY = current.getY();
		float globalTheta = current.getTheta();

		float footSeparationHalf = footSeparation/2.0;

		if (current.getLeg() == LEFT)
		{
			float xShift =  sin(globalTheta)*(footSeparationHalf);
			float yShift = -cos(globalTheta)*(footSeparationHalf);
			globalX += xShift;
			globalY += yShift;
			globalTheta -= ivTheta;

			float thetaCos = cos(globalTheta);
			float thetaSin = sin(globalTheta);
			xShift = -thetaCos*ivX + thetaSin*(ivY+footSeparationHalf);
			yShift = -thetaSin*ivX - thetaCos*(ivY+footSeparationHalf);
			globalX += xShift;
			globalY += yShift;

			predecessor->setLeg(RIGHT);
		}
		else // leg == RIGHT
		{
			float xShift = -sin(globalTheta)*(footSeparationHalf);
			float yShift =  cos(globalTheta)*(footSeparationHalf);
			globalX += xShift;
			globalY += yShift;
			globalTheta += ivTheta;

			float thetaCos = cos(globalTheta);
			float thetaSin = sin(globalTheta);
			xShift = -thetaCos*ivX - thetaSin*(ivY+footSeparationHalf);
			yShift = -thetaSin*ivX + thetaCos*(ivY+footSeparationHalf);
			globalX += xShift;
			globalY += yShift;

			predecessor ->setLeg(LEFT);
		}

		predecessor->setX(globalX);
		predecessor->setY(globalY);
		predecessor->setTheta(globalTheta);

	}


	/*
	 * #############################################################################
	 * ### class State
	 * #############################################################################
	 */

	State::State()
	{

		ivGlobalX = 0;
		ivGlobalY = 0;
		ivGlobalTheta = 0;
		ivLeg = NOLEG;
		ivKey = key(INFINITY, INFINITY);

	}


	State::State(float x, float y, float theta, Leg leg)
	{

		ivGlobalX = x;
		ivGlobalY = y;
		ivGlobalTheta = theta;
		ivLeg = leg;
		ivKey = key(INFINITY, INFINITY);

	}


	State::~State()
	{}


	bool
	State::operator == (const State &s2)
	const
	{

		if (ivLeg != s2.getLeg())
			return false;

		float diffAngle = fabs(round(ivGlobalTheta, Dstar::cvRoundingThreshold) - round(s2.getTheta(), Dstar::cvRoundingThreshold));
		if (diffAngle > M_PI)
			diffAngle = TWO_PI - diffAngle;

		float diffX = round(ivGlobalX, Dstar::cvRoundingThreshold) - round(s2.getX(), Dstar::cvRoundingThreshold);
		float diffY = round(ivGlobalY, Dstar::cvRoundingThreshold) - round(s2.getY(), Dstar::cvRoundingThreshold);

		return (((diffX*diffX + diffY*diffY) < FLOAT_COMP_THR_SQR) && (diffAngle < ANGLE_COMP_THR));

	}


	bool
	State::operator != (const State &s2)
	const
	{

		return not (*this == s2);

	}


	bool
	State::operator > (const State &s2)
	const
	{

		const key s2Key = s2.getKey();
		if (ivKey.first-FLOAT_COMP_THR > s2Key.first)
			return true;
		else if (ivKey.first < s2Key.first-FLOAT_COMP_THR)
			return false;
		return ivKey.second > s2Key.second;

	}


	bool
	State::operator <= (const State &s2)
	const
	{

		const key s2Key = s2.getKey();
		if (ivKey.first < s2Key.first)
			return true;
		else if (ivKey.first > s2Key.first)
			return false;
		return ivKey.second < s2Key.second + FLOAT_COMP_THR;

	}


	bool
	State::operator < (const State &s2)
	const
	{

		const key s2Key = s2.getKey();
		if (ivKey.first + FLOAT_COMP_THR < s2Key.first)
			return true;
		else if (ivKey.first - FLOAT_COMP_THR > s2Key.first)
			return false;
		return ivKey.second < s2Key.second;

	}


	size_t
	State::state_hash::operator ()(const State &s)
	const
	{

		return 124 * s.getX() + 34245*s.getY() + 1306*s.getTheta() + s.getLeg();

	//	float x = round(s.getX(), Dstar::cvRoundingThreshold);
	//	float y = round(s.getY(), Dstar::cvRoundingThreshold);
	//	float angle = round(s.getTheta(), Dstar::cvRoundingThreshold);
	//	return 124 * x + 34245 * y + 1306 * angle + s.getLeg();

	}

} // end of namespace

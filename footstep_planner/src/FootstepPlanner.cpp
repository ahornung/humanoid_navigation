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

#include <footstep_planner/FootstepPlanner.h>
#include <time.h>


namespace footstep_planner
{
    FootstepPlanner::FootstepPlanner()
        : ivStartPoseSetUp(false),
          ivGoalPoseSetUp(false),
          ivLastMarkerMsgSize(0),
          ivPathCost(0),
          ivRFootID("/RFoot_link"),
          ivLFootID("/LFoot_link"),
          ivMarkerNamespace("")
    {
        // private NodeHandle for parameters and private messages (debug / info)
        ros::NodeHandle nh_private("~");
        ros::NodeHandle nh_public;

        // ..publishers
        ivExpandedStatesVisPub = nh_private.advertise<sensor_msgs::PointCloud>("expanded_states", 1);
        ivFootstepPathVisPub = nh_private.advertise<visualization_msgs::MarkerArray>("footsteps_array", 1);
        ivHeuristicPathVisPub = nh_private.advertise<nav_msgs::Path>("heuristic_path", 1);
        ivPathVisPub = nh_private.advertise<nav_msgs::Path>("path", 1);
        ivStartPoseVisPub = nh_private.advertise<geometry_msgs::PoseStamped>("start", 1);

        int num_angle_bins;
        int max_hash_size;
        std::string heuristic_type;
        double step_cost;
        double diff_angle_cost;

        // read parameters from config file:
        // - planner environment settings
        nh_private.param("heuristic_type", heuristic_type, std::string("EuclideanHeuristic"));
        nh_private.param("max_hash_size", max_hash_size, 65536);
        nh_private.param("accuracy/collision_check", ivCollisionCheckAccuracy, 2);
        nh_private.param("accuracy/cell_size", ivCellSize, 0.01);
        nh_private.param("accuracy/num_angle_bins", num_angle_bins, 64);
        nh_private.param("step_cost", step_cost, 0.05);
        nh_private.param("diff_angle_cost", diff_angle_cost, 0.0);
        nh_private.param("rfoot_frame_id", ivRFootID, ivRFootID);
        nh_private.param("lfoot_frame_id", ivLFootID, ivLFootID);
        nh_private.param("accuracy/footstep/x", ivFootstepAccuracyX, 0.01);
        nh_private.param("accuracy/footstep/y", ivFootstepAccuracyY, 0.01);
        nh_private.param("accuracy/footstep/theta", ivFootstepAccuracyTheta, 0.15);
        nh_private.param("planner_type", ivPlannerType, std::string("ARAPlanner"));
        nh_private.param("search_until_first_solution",
        		ivSearchUntilFirstSolution, false);
        nh_private.param("allocated_time", ivMaxSearchTime, 7.0);
        nh_private.param("forward_search", ivForwardSearch, false);
        nh_private.param("initial_epsilon", ivInitialEpsilon, 3.0);

        // - footstep settings
        nh_private.param("foot/size/x", ivFootsizeX, 0.16);
        nh_private.param("foot/size/y", ivFootsizeY, 0.06);
        nh_private.param("foot/size/z", ivFootsizeZ, 0.015);
        nh_private.param("foot/separation", ivFootSeparation, 0.095);
        nh_private.param("foot/origin_shift/x", ivOriginFootShiftX, 0.02);
        nh_private.param("foot/origin_shift/y", ivOriginFootShiftY, 0.0);
        nh_private.param("foot/max/step/x", ivMaxFootstepX, 0.04);
        nh_private.param("foot/max/step/y", ivMaxFootstepY, 0.04);
        nh_private.param("foot/max/step/theta", ivMaxFootstepTheta, 0.349);
        nh_private.param("foot/max/inverse/step/x", ivMaxInvFootstepX, 0.04);
        nh_private.param("foot/max/inverse/step/y", ivMaxInvFootstepY, 0.01);
        nh_private.param("foot/max/inverse/step/theta", ivMaxInvFootstepTheta, 0.05);

        // - footstep discretisation
        XmlRpc::XmlRpcValue discretization_list_x;
        XmlRpc::XmlRpcValue discretization_list_y;
        XmlRpc::XmlRpcValue discretization_list_theta;
        nh_private.getParam("footsteps/x", discretization_list_x);
        nh_private.getParam("footsteps/y", discretization_list_y);
        nh_private.getParam("footsteps/theta", discretization_list_theta);
        if (discretization_list_x.getType() != XmlRpc::XmlRpcValue::TypeArray)
            ROS_ERROR("Error reading footsteps/x from config file.");
        if (discretization_list_y.getType() != XmlRpc::XmlRpcValue::TypeArray)
            ROS_ERROR("Error reading footsteps/y from config file.");
        if (discretization_list_theta.getType() != XmlRpc::XmlRpcValue::TypeArray)
            ROS_ERROR("Error reading footsteps/theta from config file.");
        // check if received footstep discretization is valid
        int size, size_y, size_t;
        try
        {
            size = discretization_list_x.size();
            size_y = discretization_list_y.size();
            size_t = discretization_list_theta.size();

            if (size != size_y || size != size_t)
            {
                ROS_ERROR("Footstep parameterization has different sizes for x/y/theta, exiting.");
                exit(0);
            }
        }
        catch (const XmlRpc::XmlRpcException& e)
        {
            ROS_ERROR("No footstep parameterization available, exiting.");
            exit(0);
        }

        // create footstep set
        ivFootstepSet.clear();
        double max_step_width = 0;
        for(int i=0; i < size; i++)
        {
            double x = (double)discretization_list_x[i];
            double y = (double)discretization_list_y[i];
            double theta = (double)discretization_list_theta[i];

            Footstep f(x, y, theta, ivCellSize, num_angle_bins, max_hash_size,
                       ivFootSeparation);
            ivFootstepSet.push_back(f);

            double cur_step_width = sqrt(x*x + y*y);

            if (cur_step_width > max_step_width)
                max_step_width = cur_step_width;
        }

        // discretise planner settings
        int max_footstep_x = cont_2_disc(ivMaxFootstepX, ivCellSize);
        int max_footstep_y = cont_2_disc(ivMaxFootstepY, ivCellSize);
        int max_footstep_theta = angle_cont_2_disc(ivMaxFootstepTheta,
                                                   num_angle_bins);
        int max_inv_footstep_x = cont_2_disc(ivMaxInvFootstepX,
                                                      ivCellSize);
        int max_inv_footstep_y = cont_2_disc(ivMaxInvFootstepY,
                                                      ivCellSize);
        int max_inv_footstep_theta = angle_cont_2_disc(ivMaxInvFootstepTheta,
                                                       num_angle_bins);

        // initialize the heuristic
        boost::shared_ptr<Heuristic> h;
        if (heuristic_type == "EuclideanHeuristic")
        {
        	h.reset(new EuclideanHeuristic(ivCellSize, num_angle_bins));
        	ROS_INFO("FootstepPlanner heuristic: euclidean distance");
        }
        else if(heuristic_type == "EuclStepCostHeuristic")
        {
            h.reset(new EuclStepCostHeuristic(ivCellSize, num_angle_bins,
                                              step_cost, diff_angle_cost,
                                              max_step_width));
            ROS_INFO("FootstepPlanner heuristic: euclidean distance with step "
                     "costs");
        }
        else if (heuristic_type == "PathCostHeuristic")
        {
            h.reset(new PathCostHeuristic(ivCellSize, num_angle_bins, step_cost,
                                          diff_angle_cost, max_step_width));
            ROS_INFO("FootstepPlanner heuristic: 2D path euclidean distance "
                     "with step costs");
            // keep a local ptr for visualization
            ivPathCostHeuristicPtr = boost::dynamic_pointer_cast<PathCostHeuristic>(h);
        }
        else
        {
            ROS_ERROR_STREAM("Heuristic " << heuristic_type << " not available,"
                             " exiting.");
            exit(1);
        }

        // initialize the planner environment
        ivPlannerEnvironmentPtr.reset(
                new FootstepPlannerEnvironment(ivFootstepSet,
                                               h,
                                               ivFootSeparation,
                                               ivOriginFootShiftX,
                                               ivOriginFootShiftY,
                                               ivFootsizeX,
                                               ivFootsizeY,
                                               max_footstep_x,
                                               max_footstep_y,
                                               max_footstep_theta,
                                               max_inv_footstep_x,
                                               max_inv_footstep_y,
                                               max_inv_footstep_theta,
                                               step_cost,
                                               ivCollisionCheckAccuracy,
                                               max_hash_size,
                                               ivCellSize,
                                               num_angle_bins,
                                               ivForwardSearch));

        // set up planner
        if (ivPlannerType == "ARAPlanner")
            ROS_INFO_STREAM("Planning with " << ivPlannerType);
        else if (ivPlannerType == "ADPlanner")
            ROS_INFO_STREAM("Planning with " << ivPlannerType);
        else if (ivPlannerType == "RSTARPlanner")
            ROS_INFO_STREAM("Planning with " << ivPlannerType);
        else
        {
            ROS_ERROR_STREAM("Planner "<< ivPlannerType <<" not available / "
                             "untested.");
            exit(1);
        }
        if (ivForwardSearch)
            ROS_INFO_STREAM("Search direction: forward planning");
        else
            ROS_INFO_STREAM("Search direction: backward planning");
        setupPlanner();
    }


    FootstepPlanner::~FootstepPlanner()
    {}


    void
    FootstepPlanner::setupPlanner()
    {
        if (ivPlannerType == "ARAPlanner")
        {
            ivPlannerPtr.reset(new ARAPlanner(ivPlannerEnvironmentPtr.get(),
                                              ivForwardSearch));
        }
        else if (ivPlannerType == "ADPlanner")
        {
            ivPlannerPtr.reset(new ADPlanner(ivPlannerEnvironmentPtr.get(),
                                             ivForwardSearch));
        }
        else if (ivPlannerType == "RSTARPlanner")
        {
            ivPlannerPtr.reset(new RSTARPlanner(ivPlannerEnvironmentPtr.get(),
                                                ivForwardSearch));
        }
    }


    bool
    FootstepPlanner::run()
    {
        ROS_DEBUG("Setting up environment");
		ivPlannerEnvironmentPtr->setUp(ivStartFootLeft, ivStartFootRight,
									   ivGoalFootLeft, ivGoalFootRight);
		ROS_DEBUG("Setting up environment done");

        int ret = 0;
        MDPConfig mdp_config;
        std::vector<int> solution_state_ids;

        // NOTE: just for the sake of completeness since this method is
        // currently doing nothing
        ivPlannerEnvironmentPtr->InitializeEnv(NULL);
        ivPlannerEnvironmentPtr->InitializeMDPCfg(&mdp_config);

        // set up planner
        if (ivPlannerPtr->set_start(mdp_config.startstateid) == 0)
        {
            ROS_ERROR("Failed to set start state.");
            return false;
        }
        if (ivPlannerPtr->set_goal(mdp_config.goalstateid) == 0)
        {
            ROS_ERROR("Failed to set goal state\n");
            return false;
        }

        ivPlannerPtr->set_initialsolution_eps(ivInitialEpsilon);
        ivPlannerPtr->set_search_mode(ivSearchUntilFirstSolution);

        ROS_INFO("Start planning (max time: %f, initial eps: %f (%f))\n",
                 ivMaxSearchTime, ivInitialEpsilon, ivPlannerPtr->get_initial_eps());
        int path_cost;
        ros::WallTime startTime = ros::WallTime::now();
        ret = ivPlannerPtr->replan(ivMaxSearchTime, &solution_state_ids,
                                   &path_cost);
        ivPathCost = double(path_cost) / FootstepPlannerEnvironment::cvMmScale;

        ivPlannerEnvironmentPtr->printHashStatistics();

        if (ret)
        {
            ROS_INFO("Solution of size %zu found after %f s",
                     solution_state_ids.size(),
            		 (ros::WallTime::now()-startTime).toSec());

            bool success = extractSolution(solution_state_ids);
            broadcastExpandedNodesVis();

            if (!success)
            {
                ROS_ERROR("extracting path failed\n\n");
                return false;
            }

            ROS_INFO("Expanded states: %i total / %i new",
                     ivPlannerEnvironmentPtr->getNumExpandedStates(),
                     ivPlannerPtr->get_n_expands());

            ROS_INFO("Final eps: %f", ivPlannerPtr->get_final_epsilon());
            ROS_INFO("Path cost: %f (%i)", ivPathCost, path_cost);

            broadcastFootstepPathVis();
            broadcastPathVis();

            return true;
        }
        else
        {
        	ROS_ERROR("No solution found");
            return false;
        }
    }


    bool
    FootstepPlanner::extractSolution(const std::vector<int>& state_ids)
    {
        ivPath.clear();

        State s;
        for(unsigned i = 0; i < state_ids.size(); ++i)
        {
            bool success = ivPlannerEnvironmentPtr->getState(state_ids[i], &s);
            if (!success)
            {
                ivPath.clear();
                return false;
            }
            ivPath.push_back(s);
        }

        return true;
    }


    bool
    FootstepPlanner::plan()
    {
    	if (!ivMapPtr)
    	{
    		ROS_ERROR("FootstepPlanner has no map yet for planning");
    		return false;
    	}
        if (!ivGoalPoseSetUp || !ivStartPoseSetUp)
        {
            ROS_ERROR("FootstepPlanner has no start or goal pose set");
            return false;
        }

        // reset the planner
        ivPlannerEnvironmentPtr->reset();
        setupPlanner();
        //ivPlannerPtr->force_planning_from_scratch();

        // start the planning and return success
        return run();
    }


    bool
    FootstepPlanner::plan(const geometry_msgs::PoseStampedConstPtr& start,
                          const geometry_msgs::PoseStampedConstPtr& goal)
    {
        return plan(start->pose.position.x, start->pose.position.y,
                    tf::getYaw(start->pose.orientation),
                    goal->pose.position.x, goal->pose.position.y,
                    tf::getYaw(goal->pose.orientation));
    }


    bool
    FootstepPlanner::plan(float start_x, float start_y, float start_theta,
                          float goal_x, float goal_y, float goal_theta)
    {
        if (!(setStart(start_x, start_y, start_theta) &&
              setGoal(goal_x, goal_y, goal_theta)))
        {
            return false;
        }

        return plan();
    }


    bool
    FootstepPlanner::planService(humanoid_nav_msgs::PlanFootsteps::Request &req,
                                 humanoid_nav_msgs::PlanFootsteps::Response &resp)
    {
    	bool result = plan(req.start.x, req.start.y, req.start.theta,
                           req.goal.x, req.goal.y, req.goal.theta);

    	resp.costs = getPathCosts();
    	resp.footsteps.reserve(ivPath.size());

    	humanoid_nav_msgs::StepTarget foot;
    	state_iter_t path_iter;
    	for (path_iter = ivPath.begin(); path_iter != ivPath.end(); path_iter++)
    	{
    		foot.pose.x = path_iter->x;
    		foot.pose.y = path_iter->y;
    		foot.pose.theta = path_iter->theta;
    		if (path_iter->leg == LEFT)
    		{
    		    foot.leg = humanoid_nav_msgs::StepTarget::left;
    		}
    		else if (path_iter->leg == RIGHT)
    		{
    		    foot.leg = humanoid_nav_msgs::StepTarget::right;
    		}
    		else
    		{
    			ROS_ERROR("Footstep pose at (%f, %f, %f) is set to NOLEG!",
                          path_iter->x, path_iter->y, path_iter->theta);
    			continue;
    		}

    		resp.footsteps.push_back(foot);
    	}
    	resp.result = result;

    	return result;
    }


    void
    FootstepPlanner::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose)
    {
        bool success = setGoal(goal_pose);
        if (success)
        {
            // NOTE: updates to the goal pose are handled in the run method
            if (ivStartPoseSetUp)
            {
            	assert(ivMapPtr);
            	run();
            }
        }
    }


    void
    FootstepPlanner::startPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose)
    {
        bool success = setStart(start_pose->pose.pose.position.x,
                                start_pose->pose.pose.position.y,
                                tf::getYaw(start_pose->pose.pose.orientation));
        if (success)
        {
            // NOTE: updates to the start pose are handled in the run method
            if (ivGoalPoseSetUp)
            {
            	assert(ivMapPtr);
                run();
            }
        }
    }


    void
    FootstepPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_map)
    {
        boost::shared_ptr<GridMap2D> gridMap(new GridMap2D(occupancy_map));
        setMap(gridMap);
    }


    bool
    FootstepPlanner::setGoal(const geometry_msgs::PoseStampedConstPtr& goal_pose)
    {
        return setGoal(goal_pose->pose.position.x,
                       goal_pose->pose.position.y,
                       tf::getYaw(goal_pose->pose.orientation));
    }


    bool
    FootstepPlanner::setGoal(float x, float y, float theta)
    {
        if (!ivMapPtr)
        {
            ROS_ERROR("Distance map hasn't been initialized yet.");
            return false;
        }

        State goal;
        goal.x = x;
        goal.y = y;
        goal.theta = theta;
        getFootPositionLeft(goal, &ivGoalFootLeft);
        getFootPositionRight(goal, &ivGoalFootRight);
        if (occupied(ivGoalFootLeft) || occupied(ivGoalFootRight))
        {
            ROS_ERROR("Goal pose at (%f %f %f) not accessible.", x, y, theta);
            return false;
        }

        ivGoalPoseSetUp = true;
        ROS_INFO("Goal pose set to (%f %f %f)", x, y, theta);

        return true;
    }


    bool
    FootstepPlanner::setStart(const geometry_msgs::PoseStampedConstPtr& start_pose)
    {
        return setStart(start_pose->pose.position.x,
                        start_pose->pose.position.y,
                        tf::getYaw(start_pose->pose.orientation));
    }


    bool
    FootstepPlanner::setStart(float x, float y, float theta)
    {
        if (!ivMapPtr)
        {
            ROS_ERROR("Distance map hasn't been initialized yet.");
            return false;
        }

        State start;
        start.x = x;
        start.y = y;
        start.theta = theta;
        getFootPositionLeft(start, &ivStartFootLeft);
        getFootPositionRight(start, &ivStartFootRight);
        if (occupied(ivStartFootLeft) || occupied(ivStartFootRight))
        {
            ROS_ERROR("Start pose (%f %f %f) not accessible.", x, y, theta);
            return false;
        }

        ivStartPoseSetUp = true;
        ROS_INFO("Start pose set to (%f %f %f)", x, y, theta);

        // publish visualization:
        geometry_msgs::PoseStamped start_pose;
        start_pose.pose.position.x = x;
        start_pose.pose.position.y = y;
        start_pose.pose.position.z = 0.025;
        start_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        start_pose.header.frame_id = ivMapPtr->getFrameID();
        start_pose.header.stamp = ros::Time::now();
        ivStartPoseVisPub.publish(start_pose);

        return true;
    }


    void
    FootstepPlanner::setMap(boost::shared_ptr<GridMap2D> gridMap)
    {
        ivMapPtr.reset();
        ivMapPtr = gridMap;
        ivPlannerEnvironmentPtr->updateDistanceMap(gridMap);
    }


    void
    FootstepPlanner::getFootPositionLeft(const State& robot, State* foot_left)
    {
        double theta = robot.theta;
        double shift_x = -sin(theta) * ivFootSeparation/2;
        double shift_y =  cos(theta) * ivFootSeparation/2;

        foot_left->x = robot.x + shift_x;
        foot_left->y = robot.y + shift_y;
        foot_left->theta = theta;
        foot_left->leg = LEFT;
    }


    void
    FootstepPlanner::getFootPositionRight(const State& robot, State* foot_right)
    {
        double theta = robot.theta;
        double shift_x = -sin(theta) * ivFootSeparation/2;
        double shift_y =  cos(theta) * ivFootSeparation/2;

        foot_right->x = robot.x - shift_x;
        foot_right->y = robot.y - shift_y;
        foot_right->theta = theta;
        foot_right->leg = RIGHT;
    }


    bool
    FootstepPlanner::occupied(const State& u)
    {
        float theta_cos = cos(u.theta);
        float theta_sin = sin(u.theta);
        float shift_x = theta_cos*ivOriginFootShiftX - theta_sin*ivOriginFootShiftY;
        float shift_y;
        if (u.leg == LEFT)
            shift_y = theta_sin*ivOriginFootShiftX + theta_cos*ivOriginFootShiftY;
        else // leg == RLEG
            shift_y = theta_sin*ivOriginFootShiftX - theta_cos*ivOriginFootShiftY;

        return collision_check(u.x + shift_x, u.y + shift_y, u.theta,
                               ivFootsizeX, ivFootsizeY,
                               ivCollisionCheckAccuracy,
                               *ivMapPtr);
    }


    void
    FootstepPlanner::clearFootstepPathVis(unsigned num_footsteps)
    {
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_msg;

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = ivMapPtr->getFrameID();


        if (num_footsteps < 1)
            num_footsteps = ivLastMarkerMsgSize;

        for (unsigned i = 0; i < num_footsteps; i++)
        {
            marker.ns = ivMarkerNamespace;
            marker.id = i;
            marker.action = visualization_msgs::Marker::DELETE;

            marker_msg.markers.push_back(marker);
        }

        ivFootstepPathVisPub.publish(marker_msg);
    }


    void
    FootstepPlanner::broadcastExpandedNodesVis()
    {
        if (ivExpandedStatesVisPub.getNumSubscribers() > 0)
        {
            sensor_msgs::PointCloud cloud_msg;
            geometry_msgs::Point32 point;
            std::vector<geometry_msgs::Point32> points;

            State s;
            FootstepPlannerEnvironment::exp_states_iter_t state_id_iter;
            for(state_id_iter = ivPlannerEnvironmentPtr->getExpandedStatesStart();
                state_id_iter != ivPlannerEnvironmentPtr->getExpandedStatesEnd();
                state_id_iter++)
            {
                ivPlannerEnvironmentPtr->getState(*state_id_iter, &s);
                point.x = s.x;
                point.y = s.y;
                point.z = 0.01;
                points.push_back(point);
            }
            cloud_msg.header.stamp = ros::Time::now();
            cloud_msg.header.frame_id = ivMapPtr->getFrameID();

            cloud_msg.points = points;

            ivExpandedStatesVisPub.publish(cloud_msg);
        }
    }


    void
    FootstepPlanner::broadcastFootstepPathVis()
    {

        if (ivPath.size() == 0)
        {
            ROS_INFO("no path has been extracted yet");
            return;
        }

        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray broadcast_msg;
        std::vector<visualization_msgs::Marker> markers;

        int markers_counter = 0;

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = ivMapPtr->getFrameID();

		// add the missing start foot to the publish vector
        if (ivPath.front().leg == LEFT)
        	footstepToMarker(ivStartFootRight, &marker);
        else
        	footstepToMarker(ivStartFootLeft, &marker);

		marker.id = markers_counter++;
		markers.push_back(marker);

		// add the right start foot to the publish vector
		footstepToMarker(ivStartFootRight, &marker);
		marker.id = markers_counter++;
		markers.push_back(marker);


        // add the footsteps of the path to the publish vector
        state_iter_t path_iter = ivPath.begin();
        for(; path_iter != ivPath.end(); path_iter++)
        {
            footstepToMarker(*path_iter, &marker);
            marker.id = markers_counter++;
            markers.push_back(marker);
        }
        if (markers_counter < ivLastMarkerMsgSize)
        {
            for(int j = markers_counter; j < ivLastMarkerMsgSize; j++)
            {
                marker.ns = ivMarkerNamespace;
                marker.id = j;
                marker.action = visualization_msgs::Marker::DELETE;

                markers.push_back(marker);
            }
        }

        // add goal feet for visualization:
        if (ivPath.back().leg == LEFT)
        	footstepToMarker(ivGoalFootRight, &marker);
        else
        	footstepToMarker(ivGoalFootLeft, &marker);
        marker.id = markers_counter++;
        markers.push_back(marker);

        broadcast_msg.markers = markers;
        ivLastMarkerMsgSize = markers.size();

        ivFootstepPathVisPub.publish(broadcast_msg);
    }


    void
    FootstepPlanner::broadcastPathVis()
    {
        if (ivPath.size() == 0)
        {
            ROS_INFO("no path has been extracted yet");
            return;
        }

        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped state;

        state.header.stamp = ros::Time::now();
        state.header.frame_id = ivMapPtr->getFrameID();

        state_iter_t path_iter;
        for(path_iter = ivPath.begin(); path_iter != ivPath.end(); path_iter++)
        {
            state.pose.position.x = path_iter->x;
            state.pose.position.y = path_iter->y;
            path_msg.poses.push_back(state);
        }

        path_msg.header = state.header;
        ivPathVisPub.publish(path_msg);
    }


    void
    FootstepPlanner::footstepToMarker(const State& footstep,
                                      visualization_msgs::Marker* marker)
    {
        marker->header.stamp = ros::Time::now();
        marker->header.frame_id = ivMapPtr->getFrameID();
        marker->ns = ivMarkerNamespace;
        marker->type = visualization_msgs::Marker::CUBE;
        marker->action = visualization_msgs::Marker::ADD;

        float cos_theta = cos(footstep.theta);
        float sin_theta = sin(footstep.theta);
        float x_shift = cos_theta*ivOriginFootShiftX - sin_theta*ivOriginFootShiftY;
        float y_shift;
        if (footstep.leg == LEFT)
            y_shift = sin_theta*ivOriginFootShiftX + cos_theta*ivOriginFootShiftY;
        else // leg == RLEG
            y_shift = sin_theta*ivOriginFootShiftX - cos_theta*ivOriginFootShiftY;
        marker->pose.position.x = footstep.x + x_shift;
        marker->pose.position.y = footstep.y + y_shift;
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(footstep.theta),
                              marker->pose.orientation);

        marker->scale.x = ivFootsizeX; // - 0.01;
        marker->scale.y = ivFootsizeY; // - 0.01;
        marker->scale.z = ivFootsizeZ;

        // TODO: make color configurable?
        if (footstep.leg == RIGHT)
        {
            marker->color.r = 0.0f;
            marker->color.g = 1.0f;
        }
        else // leg == LEFT
        {
            marker->color.r = 1.0f;
            marker->color.g = 0.0f;
        }
        marker->color.b = 0.0;
        marker->color.a = 0.4;

        marker->lifetime = ros::Duration();
    }
}

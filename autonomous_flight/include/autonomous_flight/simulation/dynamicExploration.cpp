/*
	FILE: dynamicExploration.cpp
	-----------------------------
	Implementation of dynamic exploration
*/

#include <autonomous_flight/simulation/dynamicExploration.h>
#include <autonomous_flight/simulation/climbColumn.h>
#include <iomanip>
#include <sstream>
#include <limits>

namespace AutoFlight{
	dynamicExploration::dynamicExploration(const ros::NodeHandle& nh) : flightBase(nh){
		this->initParam();
		this->initModules();
		this->registerPub();
		this->registerMissionInterface();
		if (this->useFakeDetector_){
			// free map callback
			this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::freeMapCB, this);
		}
	}

	void dynamicExploration::initParam(){
    	// use simulation detector	
		if (not this->nh_.getParam("autonomous_flight/use_fake_detector", this->useFakeDetector_)){
			this->useFakeDetector_ = false;
			cout << "[AutoFlight]: No use fake detector param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use fake detector is set to: " << this->useFakeDetector_ << "." << endl;
		}
		
		// desired velocity
		if (not this->nh_.getParam("autonomous_flight/desired_velocity", this->desiredVel_)){
			this->desiredVel_ = 0.5;
			cout << "[AutoFlight]: No desired velocity param. Use default 0.5m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired velocity is set to: " << this->desiredVel_ << "m/s." << endl;
		}	

		// desired acceleration
		if (not this->nh_.getParam("autonomous_flight/desired_acceleration", this->desiredAcc_)){
			this->desiredAcc_ = 2.0;
			cout << "[AutoFlight]: No desired acceleration param. Use default 2.0m/s^2." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired acceleration is set to: " << this->desiredAcc_ << "m/s^2." << endl;
		}	

		//  desired angular velocity
		if (not this->nh_.getParam("autonomous_flight/desired_angular_velocity", this->desiredAngularVel_)){
			this->desiredAngularVel_ = 0.5;
			cout << "[AutoFlight]: No angular velocity param. Use default 0.5rad/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Angular velocity is set to: " << this->desiredAngularVel_ << "rad/s." << endl;
		}	

		//  desired angular velocity
		if (not this->nh_.getParam("autonomous_flight/waypoint_stablize_time", this->wpStablizeTime_)){
			this->wpStablizeTime_ = 1.0;
			cout << "[AutoFlight]: No waypoint stablize time param. Use default 1.0s." << endl;
		}
		else{
			cout << "[AutoFlight]: Waypoint stablize time is set to: " << this->wpStablizeTime_ << "s." << endl;
		}	

		//  initial scan
		if (not this->nh_.getParam("autonomous_flight/initial_scan", this->initialScan_)){
			this->initialScan_ = false;
			cout << "[AutoFlight]: No initial scan param. Use default 1.0s." << endl;
		}
		else{
			cout << "[AutoFlight]: Initial scan is set to: " << this->initialScan_ << endl;
		}	

    	// replan time for dynamic obstacle
		if (not this->nh_.getParam("autonomous_flight/replan_time_for_dynamic_obstacles", this->replanTimeForDynamicObstacle_)){
			this->replanTimeForDynamicObstacle_ = 0.3;
			cout << "[AutoFlight]: No dynamic obstacle replan time param found. Use default: 0.3s." << endl;
		}
		else{
			cout << "[AutoFlight]: Dynamic obstacle replan time is set to: " << this->replanTimeForDynamicObstacle_ << "s." << endl;
		}	

    	// free range 
    	std::vector<double> freeRangeTemp;
		if (not this->nh_.getParam("autonomous_flight/free_range", freeRangeTemp)){
			this->freeRange_ = Eigen::Vector3d (2, 2, 1);
			cout << "[AutoFlight]: No free range param found. Use default: [2, 2, 1]m." << endl;
		}
		else{
			this->freeRange_(0) = freeRangeTemp[0];
			this->freeRange_(1) = freeRangeTemp[1];
			this->freeRange_(2) = freeRangeTemp[2];
			cout << "[AutoFlight]: Free range is set to: " << this->freeRange_.transpose() << "m." << endl;
		}	

    	// reach goal distance
		if (not this->nh_.getParam("autonomous_flight/reach_goal_distance", this->reachGoalDistance_)){
			this->reachGoalDistance_ = 0.1;
			cout << "[AutoFlight]: No reach goal distance param found. Use default: 0.1m." << endl;
		}
		else{
			cout << "[AutoFlight]: Reach goal distance is set to: " << this->reachGoalDistance_ << "m." << endl;
		}

		// initial waypoints: flat list of [x, y, z, yaw_deg] flown before exploration starts
		std::vector<double> initialWaypointsTemp;
		if (not this->nh_.getParam("autonomous_flight/initial_waypoints", initialWaypointsTemp) or initialWaypointsTemp.size() == 0){
			cout << "[AutoFlight]: No initial waypoints. Start exploration directly." << endl;
		}
		else if (initialWaypointsTemp.size() % 4 != 0){
			cout << "\033[1;31m[AutoFlight]: initial_waypoints must be a list of [x, y, z, yaw_deg] groups. Ignored.\033[0m" << endl;
		}
		else{
			for (size_t i=0; i<initialWaypointsTemp.size(); i+=4){
				this->initialWaypoints_.push_back(Eigen::Vector4d (initialWaypointsTemp[i], initialWaypointsTemp[i+1], initialWaypointsTemp[i+2], initialWaypointsTemp[i+3] * PI_const / 180.0));
			}
			cout << "[AutoFlight]: Initial waypoints (" << this->initialWaypoints_.size() << "):" << endl;
			for (Eigen::Vector4d& wp : this->initialWaypoints_){
				cout << "    [" << wp(0) << ", " << wp(1) << ", " << wp(2) << "], yaw " << wp(3) * 180.0 / PI_const << " deg" << endl;
			}
		}

		// mission interface parameters
		this->nh_.param("autonomous_flight/max_exploration_failures", this->maxExplorationFailures_, 5);
		this->nh_.param("autonomous_flight/inspect_hold_time", this->inspectHoldTime_, 3.0);
		this->nh_.param("autonomous_flight/inspect_distance", this->inspectDistance_, 2.5);
		this->nh_.param("autonomous_flight/land_clearance", this->landClearance_, 0.4);
		this->nh_.param<std::string>("autonomous_flight/land_topic", this->landTopic_, "/CERLAB/quadcopter/land");
		this->nh_.param<double>("autonomous_flight/floor_z", this->floorZ_, std::numeric_limits<double>::quiet_NaN());
		this->nh_.param<std::string>("autonomous_flight/preset", this->preset_, "");
		this->nh_.param<std::string>("autonomous_flight/run_save_dir", this->runSaveDir_, ""); // empty = ~/mission_runs
		this->nh_.param<double>("autonomous_flight/complete_min_gain", this->completeMinGain_, 1.0);
		this->nh_.param<int>("autonomous_flight/complete_stall_plans", this->completeStallPlans_, 0); // off unless a config sets it
		this->nh_.param<double>("autonomous_flight/stuck_land_time", this->stuckLandTime_, 15.0); // s hovering with no way home before landing where it is
		this->nh_.param<std::string>("autonomous_flight/takeoff_topic", this->takeoffTopic_, "/CERLAB/quadcopter/takeoff");

		// % explored is measured over this region (default: the DEP exploration region)
		std::vector<double> regionMin, regionMax;
		if (not this->nh_.getParam("autonomous_flight/progress_region_min", regionMin) or not this->nh_.getParam("autonomous_flight/progress_region_max", regionMax)){
			this->nh_.getParam("DEP/global_region_min", regionMin);
			this->nh_.getParam("DEP/global_region_max", regionMax);
		}
		if (regionMin.size() == 3 and regionMax.size() == 3){
			this->progressRegionMin_ = Eigen::Vector3d (regionMin[0], regionMin[1], regionMin[2]);
			this->progressRegionMax_ = Eigen::Vector3d (regionMax[0], regionMax[1], regionMax[2]);
		}
		else{
			this->progressRegionMin_.setZero();
			this->progressRegionMax_.setZero();
		}
		cout << "[AutoFlight]: Progress region: [" << this->progressRegionMin_.transpose() << "] to [" << this->progressRegionMax_.transpose() << "]" << endl;
	}

	void dynamicExploration::initModules(){
		// initialize map
		if (this->useFakeDetector_){
			// initialize fake detector
			this->detector_.reset(new onboardDetector::fakeDetector (this->nh_));	
			this->map_.reset(new mapManager::dynamicMap (this->nh_, false));
		}
		else{
			this->map_.reset(new mapManager::dynamicMap (this->nh_));
		}

		// initialize exploration planner
		this->expPlanner_.reset(new globalPlanner::DEP (this->nh_));
		this->expPlanner_->setMap(this->map_);
		this->expPlanner_->loadVelocity(this->desiredVel_, this->desiredAngularVel_);

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->map_);
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->map_);
		this->bsplineTraj_->updateMaxVel(this->desiredVel_);
		this->bsplineTraj_->updateMaxAcc(this->desiredAcc_);		
	}

	void dynamicExploration::registerCallback(){
		// fly the initial waypoints first (current pose is the path start, like a DEP path)
		if (this->initialWaypoints_.size() != 0){
			this->flyPath(this->makeEntryPath(), NUDGE);
			this->logEvent("Flying " + std::to_string(this->initialWaypoints_.size()) + " entry waypoints");
		}
		else{
			this->missionState_ = EXPLORE;
			this->explorationReplan_ = true;
			this->logEvent("Exploring");
		}

		// initialize exploration planner replan in another thread
		this->exploreReplanWorker_ = std::thread(&dynamicExploration::exploreReplan, this);
		this->exploreReplanWorker_.detach();

		// exploration callback
		// this->explorationTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicExploration::explorationCB, this);

		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.02), &dynamicExploration::plannerCB, this);

		// replan check timer
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::replanCheckCB, this);
		
		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::trajExeCB, this);
	
		// visualization execution callabck
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicExploration::visCB, this);

		// mark the space the robot flies through as free (the tilted lidar never observes the cone under the robot)
		this->freeRobotRegionTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicExploration::freeRobotRegionCB, this);
	}

	nav_msgs::Path dynamicExploration::makeEntryPath(){
		nav_msgs::Path initialPath;
		geometry_msgs::PoseStamped start;
		start.pose = this->odom_.pose.pose;
		initialPath.poses.push_back(start);
		for (Eigen::Vector4d& wp : this->initialWaypoints_){
			geometry_msgs::PoseStamped ps;
			ps.header.frame_id = "map";
			ps.pose.position.x = wp(0);
			ps.pose.position.y = wp(1);
			ps.pose.position.z = wp(2);
			ps.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, wp(3));
			initialPath.poses.push_back(ps);
		}
		return initialPath;
	}

	void dynamicExploration::freeRobotRegionCB(const ros::TimerEvent&){
		Eigen::Vector3d robotSize;
		this->map_->getRobotSize(robotSize);
		Eigen::Vector3d pos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		this->map_->freeRegion(pos - robotSize/2, pos + robotSize/2);
	}

	void dynamicExploration::registerPub(){
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/poly_traj", 1000);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/pwl_trajectory", 1000);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/bspline_trajectory", 1000);
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/input_trajectory", 1000);
	}

	void dynamicExploration::explorationCB(const ros::TimerEvent&){
		if (this->explorationReplan_){
			this->expPlanner_->setMap(this->map_);
			ros::Time startTime = ros::Time::now();
			bool replanSuccess = this->expPlanner_->makePlan();
			if (replanSuccess){
				this->waypoints_ = this->expPlanner_->getBestPath();
				this->newWaypoints_ = true;
				this->waypointIdx_ = 1;
				this->explorationReplan_ = false;
			}
			ros::Time endTime = ros::Time::now();
			cout << "[AutoFlight]: DEP planning time: " << (endTime - startTime).toSec() << "s." << endl;			
		}
	}

	void dynamicExploration::plannerCB(const ros::TimerEvent&){
		// cout << "in planner callback" << endl;

		if (this->replan_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useFakeDetector_){
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}			
			nav_msgs::Path inputTraj;
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions); 
			// double initTs = this->bsplineTraj_->getInitTs();

			// generate new trajectory
			nav_msgs::Path simplePath;
			geometry_msgs::PoseStamped pStart, pGoal;
			pStart.pose = this->odom_.pose.pose;
			pGoal = this->goal_;
			simplePath.poses = {pStart, pGoal};
			this->pwlTraj_->updatePath(simplePath, false);
			this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
			// if (not this->trajectoryReady_){
			// 	// generate new trajectory
			// 	nav_msgs::Path simplePath;
			// 	geometry_msgs::PoseStamped pStart, pGoal;
			// 	pStart.pose = this->odom_.pose.pose;
			// 	pGoal = this->goal_;
			// 	simplePath.poses = {pStart, pGoal};
			// 	this->pwlTraj_->updatePath(simplePath, false);
			// 	this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
			// }
			// else{
			// 	Eigen::Vector3d bsplineLastPos = this->trajectory_.at(this->trajectory_.getDuration());
			// 	geometry_msgs::PoseStamped lastPs; lastPs.pose.position.x = bsplineLastPos(0); lastPs.pose.position.y = bsplineLastPos(1); lastPs.pose.position.z = bsplineLastPos(2);
			// 	Eigen::Vector3d goalPos (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
			// 	// if ((bsplineLastPos - goalPos).norm() >= 0.1){
			// 	nav_msgs::Path inputPWLTraj;
			// 	nav_msgs::Path simplePath;
			// 	geometry_msgs::PoseStamped pStart, pGoal;
			// 	pStart = lastPs;
			// 	pGoal = this->goal_;
			// 	simplePath.poses = {pStart, pGoal};
			// 	this->pwlTraj_->updatePath(simplePath, false);
			// 	this->pwlTraj_->makePlan(inputPWLTraj, this->bsplineTraj_->getControlPointDist());


			// 	nav_msgs::Path adjustedInputCombinedTraj;
			// 	bool satisfyDistanceCheck = false;
			// 	double dtTemp = initTs;
			// 	double finalTimeTemp;
			// 	ros::Time startTime = ros::Time::now();
			// 	ros::Time currTime;
			// 	while (ros::ok()){
			// 		currTime = ros::Time::now();
			// 		if ((currTime - startTime).toSec() >= 0.05){
			// 			cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 			break;
			// 		}							
			// 		nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 		nav_msgs::Path inputCombinedTraj;
			// 		inputCombinedTraj.poses = inputRestTraj.poses;
			// 		for (size_t i=1; i<inputPWLTraj.poses.size(); ++i){
			// 			inputCombinedTraj.poses.push_back(inputPWLTraj.poses[i]);
			// 		}
					
			// 		satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputCombinedTraj, adjustedInputCombinedTraj, dtTemp, finalTimeTemp);
			// 		if (satisfyDistanceCheck) break;
					
			// 		dtTemp *= 0.8; // magic number 0.8
			// 	}
			// 	inputTraj = adjustedInputCombinedTraj;
			// 	// }
			// 	// else{
			// 	// 	nav_msgs::Path adjustedInputRestTraj;
			// 	// 	bool satisfyDistanceCheck = false;
			// 	// 	double dtTemp = initTs;
			// 	// 	double finalTimeTemp;
			// 	// 	ros::Time startTime = ros::Time::now();
			// 	// 	ros::Time currTime;
			// 	// 	while (ros::ok()){
			// 	// 		currTime = ros::Time::now();
			// 	// 		if ((currTime - startTime).toSec() >= 0.05){
			// 	// 			cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 	// 			break;
			// 	// 		}
			// 	// 		nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 	// 		satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputRestTraj, adjustedInputRestTraj, dtTemp, finalTimeTemp);
			// 	// 		if (satisfyDistanceCheck) break;
						
			// 	// 		dtTemp *= 0.8;
			// 	// 	}
			// 	// 	inputTraj = adjustedInputRestTraj;					
			// 	// }

			// }

			

			this->inputTrajMsg_ = inputTraj;
			bool updateSuccess = this->bsplineTraj_->updatePath(inputTraj, startEndConditions);
			if (obstaclesPos.size() != 0 and updateSuccess){
				this->bsplineTraj_->updateDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			if (updateSuccess){
				nav_msgs::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					this->trajStartTime_ = ros::Time::now();
					this->trajTime_ = 0.0; // reset trajectory time
					this->trajectory_ = this->bsplineTraj_->getTrajectory();

					// optimize time
					// ros::Time timeOptStartTime = ros::Time::now();
					// this->timeOptimizer_->optimize(this->trajectory_, this->desiredVel_, this->desiredAcc_, 0.1);
					// ros::Time timeOptEndTime = ros::Time::now();
					// cout << "[AutoFlight]: Time optimizatoin spends: " << (timeOptEndTime - timeOptStartTime).toSec() << "s." << endl;

					// never hand a non-finite trajectory to the controller
					bool trajFinite = true;
					for (double t=0.0; t<=this->trajectory_.getDuration() and trajFinite; t+=0.1){
						trajFinite = this->trajectory_.at(t).allFinite();
					}
					if (not trajFinite){
						cout << "\033[1;31m[AutoFlight]: Rejected a non-finite trajectory. Hovering.\033[0m" << endl;
						this->trajectoryReady_ = false;
						this->stop();
						this->replan_ = false;
						this->onPathFinished(false, "non-finite trajectory");
						return;
					}

					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "\033[1;32m[AutoFlight]: Trajectory generated successfully.\033[0m " << endl;
				}
				else{
					// if the current trajectory is still valid, then just ignore this iteration
					// if the current trajectory/or new goal point is assigned is not valid, then just stop
					if (this->hasCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails." << endl;
						this->replan_ = false;
						this->onPathFinished(false, "trajectory generation failed");
					}
					else if (this->hasDynamicCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails. Replan for dynamic obstacles." << endl;
						this->replan_ = true;
					}
					else{
						if (this->trajectoryReady_){
							cout << "[AutoFlight]: Trajectory fail. Use trajectory from previous iteration." << endl;
							this->replan_ = false;
						}
						else{
							cout << "[AutoFlight]: Unable to generate a feasible trajectory." << endl;
							this->replan_ = false;
							this->onPathFinished(false, "no feasible trajectory");
						}
					}
				}
			}
			else{
				this->trajectoryReady_ = false;
				this->stop();
				this->replan_ = false;
				cout << "[AutoFlight]: Goal is not valid. Stop." << endl;
				this->onPathFinished(false, "goal not valid");
			}


		}
	}

	void dynamicExploration::replanCheckCB(const ros::TimerEvent&){
		/*
			Replan if
			1. collision detected
			2. new goal point assigned
			3. fixed distance
		*/

		// take over a path planned by the planning thread (exploration, return home, inspection)
		this->applyPendingPath();

		// landing: cut the motors once the robot has settled (target reached or resting on the ground)
		if (this->missionState_ == LAND){
			double vz = this->odom_.twist.twist.linear.z;
			if (vz < -0.1) this->descentStarted_ = true;
			// the fallback descent has no trajectory to finish: the landing target can sit in the inflated ground
			if (not this->trajectoryReady_ and not this->replan_ and not this->newWaypoints_ and this->currPos()(2) - this->landTarget_(2) < 0.15 and std::abs(vz) < 0.1){
				this->touchdown();
				return;
			}
			if (not this->descentStarted_ or std::abs(vz) > 0.05 or this->currPos()(2) - this->landTarget_(2) < 0.15){
				this->touchdownCheckTime_ = ros::Time::now();
			}
			else if ((ros::Time::now() - this->touchdownCheckTime_).toSec() > 1.5){
				this->touchdown(); // stuck above the target: resting on unmapped ground
				return;
			}
		}

		if (this->newWaypoints_){
			this->replan_ = false;
			this->trajectoryReady_ = false;
			double dx = this->waypoints_.poses[1].pose.position.x - this->odom_.pose.pose.position.x;
			double dy = this->waypoints_.poses[1].pose.position.y - this->odom_.pose.pose.position.y;
			double yaw = atan2(dy, dx);
			if (std::hypot(dx, dy) < 0.3){ // a vertical leg has no travel direction: face where the path goes next
				yaw = AutoFlight::rpy_from_quaternion(this->waypoints_.poses[1].pose.orientation);
			}
			// cout << "[AutoFlight]: Go to next waypoint. Press ENTER to continue rotation." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();
			int motionGen = this->motionGen_;
			this->moveToOrientation(yaw, this->desiredAngularVel_);
			if (motionGen != this->motionGen_) return; // a command replaced this leg while rotating
			// cout << "[AutoFlight]: Press ENTER to move forward." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();		
			this->replan_ = true;
			this->newWaypoints_ = false;
			if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
				this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			}
			++this->waypointIdx_;
			cout << "[AutoFlight]: Replan for new waypoints." << endl; 

			return;
		}

		// if (this->isReach(this->goal_, 0.1, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
		// cout << "outside the if" << endl;
		// cout << "waypoints size: " << this->waypoints_.poses.size() << endl;
		// cout << "current waypoint idx: " << this->waypointIdx_ << endl;
		if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
			// cout << "1" << endl;
			// when reach current goal point, reset replan and trajectory ready
			this->replan_ = false;
			this->trajectoryReady_ = false;
			// cout << "[AutoFlight]: Go to next waypoint. Press ENTER to continue rotation." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();
			cout << "[AutoFlight]: Rotate and replan..." << endl;
			geometry_msgs::Quaternion quat = this->goal_.pose.orientation;
			double yaw = AutoFlight::rpy_from_quaternion(quat);
			int motionGen = this->motionGen_;
			this->waitTime(this->wpStablizeTime_);
			this->moveToOrientation(yaw, this->desiredAngularVel_);
			cout << "[AutoFlight]: Finish rotation." << endl;
			if (motionGen != this->motionGen_) return; // a command replaced this leg while rotating: it did not finish
			// cout << "[AutoFlight]: Press ENTER to move forward." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();	

			// change current goal
			if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
				this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			}
			bool pathFinished = this->waypointIdx_ + 1 > int(this->waypoints_.poses.size());
			if (pathFinished){
				this->replan_ = false;
			}
			else{
				cout << "[AutoFlight]: Start planning for next waypoint." << endl;
				this->replan_ = true;
			}
			++this->waypointIdx_;
			this->trajectoryReady_ = false;
			if (pathFinished){
				this->onPathFinished(true);
			}
			return;
		}
		else if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, true) and (this->replan_ or this->trajectoryReady_)){
			this->replan_ = false;
			this->trajectoryReady_ = false;
			this->onPathFinished(true);
			return;
		}

		if (this->waypoints_.poses.size() != 0){
			if (not this->isGoalValid() and (this->replan_ or this->trajectoryReady_)){
				this->replan_ = false;
				this->trajectoryReady_ = false;
				cout << "[AutoFlight]: Current goal is invalid." << endl;
				this->onPathFinished(false, "current goal invalid");
				return;
			}
		}

		// if (this->reachExplorationGoal()){
		// 	this->replan_ = false;
		// 	this->trajectoryReady_ = false;
		// 	geometry_msgs::Quaternion quat = this->waypoints_.poses.back().pose.orientation;
		// 	double yaw = AutoFlight::rpy_from_quaternion(quat);
		// 	cout << "[AutoFlight]: Reach exploration goal. Rotate and replan..." << endl;
		// 	this->moveToOrientation(yaw, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Finish rotation. Start to replan." << endl;
		// 	this->replan_ = true;
		// 	return;
		// }

		if (this->trajectoryReady_){
			// only exploration goals must be known free with a safety margin (DEP check); entry, return, inspection
			// and landing goals may lie in unobserved space (e.g. the lidar's blind cone), so only reject them when occupied
			if (this->missionState_ != EXPLORE){
				if (not this->isGoalValid()){
					this->trajectoryReady_ = false;
					this->replan_ = false;
					this->stop();
					cout << "[AutoFlight]: The goal of the current leg is occupied." << endl;
					this->onPathFinished(false, "goal occupied");
					return;
				}
			}
			else if (not this->expPlanner_->isPosValid(this->trajectory_.at(this->trajectory_.getDuration()))){
				this->trajectoryReady_ = false;
				this->replan_ = false;
				this->stop();
				cout << "[AutoFlight]: The goal of the current local trajectory is not safe." << endl;
				this->onPathFinished(false, "goal of local trajectory not safe");
				return;
			}

			if (this->hasCollision()){ // if trajectory not ready, do not replan
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for collision." << endl;
				return;
			}

			// replan for dynamic obstacles
			if (this->computeExecutionDistance() >= 0.3 and this->hasDynamicCollision()){
			// if (this->hasDynamicObstacle()){
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for dynamic obstacles." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 1.5 and AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) >= 3){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 0.3 and this->replanForDynamicObstacle()){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan for dynamic obstacles." << endl;
				return;
			}
		}	
	}

	void dynamicExploration::trajExeCB(const ros::TimerEvent&){
		if (this->trajectoryReady_){
			ros::Time currTime = ros::Time::now();
			double realTime = (currTime - this->trajStartTime_).toSec();
			this->trajTime_ = this->bsplineTraj_->getLinearReparamTime(realTime);
			double linearReparamFactor = this->bsplineTraj_->getLinearFactor();
			Eigen::Vector3d pos = this->trajectory_.at(this->trajTime_);
			Eigen::Vector3d vel = this->trajectory_.getDerivative().at(this->trajTime_) * linearReparamFactor;
			Eigen::Vector3d acc = this->trajectory_.getDerivative().getDerivative().at(this->trajTime_) * pow(linearReparamFactor, 2);
			double endTime = this->trajectory_.getDuration()/linearReparamFactor;
			this->lastTargetZ_ = pos(2);
			if (not (pos.allFinite() and vel.allFinite() and acc.allFinite())){
				cout << "\033[1;31m[AutoFlight]: Non-finite trajectory target. Hovering.\033[0m" << endl;
				this->trajectoryReady_ = false;
				this->stop();
				return;
			}

			double leftTime = endTime - realTime; 
			tracking_controller::Target target;
			if (leftTime <= 0.0){ // zero vel and zero acc if close to
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = 0.0;
				target.velocity.y = 0.0;
				target.velocity.z = 0.0;
				target.acceleration.x = 0.0;
				target.acceleration.y = 0.0;
				target.acceleration.z = 0.0;
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				this->updateTargetWithState(target);						
			}
			else{
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = vel(0);
				target.velocity.y = vel(1);
				target.velocity.z = vel(2);
				target.acceleration.x = acc(0);
				target.acceleration.y = acc(1);
				target.acceleration.z = acc(2);
				this->updateTargetWithState(target);						
			}
		}
	}

	void dynamicExploration::visCB(const ros::TimerEvent&){
		if (this->polyTrajMsg_.poses.size() != 0){
			this->polyTrajPub_.publish(this->polyTrajMsg_);
		}
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}
		if (this->inputTrajMsg_.poses.size() != 0){
			this->inputTrajPub_.publish(this->inputTrajMsg_);
		}
	}

	void dynamicExploration::freeMapCB(const ros::TimerEvent&){
		std::vector<onboardDetector::box3D> obstacles;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		this->detector_->getObstacles(obstacles);
		double fov = 1.57;
		for (onboardDetector::box3D ob: obstacles){
			if (this->detector_->isObstacleInSensorRange(ob, fov)){
				Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-0.3, ob.z-ob.z_width/2-0.3); // model origin can be its centre
				Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+0.3, ob.z+ob.z_width+0.3);
				freeRegions.push_back(std::make_pair(lowerBound, upperBound));
			}
		}
		this->map_->updateFreeRegions(freeRegions);
		this->map_->freeRegions(freeRegions);
	}

	void dynamicExploration::run(){
		// wait for the start command (mission GUI, or: rosservice call /exploration/start)
		cout << "\033[1;32m[AutoFlight]: Ready. Waiting for the start command (mission GUI or `rosservice call /exploration/start`).\033[0m" << endl;
		this->logEvent("Ready: waiting for Start");
		ros::Rate waitRate (20);
		while (ros::ok() and not this->startRequested_){
			ros::spinOnce();
			waitRate.sleep();
		}
		if (not ros::ok()) return;

		this->missionStartTime_ = ros::Time::now();
		this->homeGroundZ_ = this->odom_.pose.pose.position.z;
		this->homePos_ = Eigen::Vector3d (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->takeoffHgt_);
		this->missionState_ = TAKEOFF;
		this->logEvent("Taking off");
		this->takeoff();
		this->logEvent("Takeoff complete");

		bool recordRosbag = false; // a bag grows about 25 GB per run
		this->nh_.param<bool>("autonomous_flight/record_rosbag", recordRosbag, false);
		if (recordRosbag){
			int temp1 = system("mkdir ~/rosbag_exploration_info &");
			int temp2 = system("mv ~/rosbag_exploration_info/exploration_info.bag ~/rosbag_exploration_info/previous.bag &");
			int temp3 = system("rosbag record -O ~/rosbag_exploration_info/exploration_info.bag /camera/aligned_depth_to_color/image_raw_t /camera/color/image_raw_t /dynamic_map/inflated_voxel_map_t /onboard_detector/dynamic_bboxes /mavros/local_position/pose /dynamicExploration/bspline_trajectory /mavros/setpoint_position/local /tracking_controller/target_pose /dep/best_paths /dep/roadmap /dep/candidate_paths /dep/best_paths /dep/frontier_regions /dynamic_map/2D_occupancy_map __name:=exploration_bag_info &");
			if (temp1==-1 or temp2==-1 or temp3==-1){
				cout << "[AutoFlight]: Recording fails." << endl;
			}
		}

		this->initExplore();

		this->registerCallback();
	}

	void dynamicExploration::initExplore(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d c1 = startPos - this->freeRange_;
		Eigen::Vector3d c2 = startPos + this->freeRange_;
		this->map_->freeRegion(c1, c2);
		cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << this->freeRange_.transpose() << endl;

		if (this->initialScan_){
			cout << "[AutoFlight]: Start initial scan..." << endl;
			this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
						
			this->moveToOrientation(-PI_const, this->desiredAngularVel_);

			this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
			
			this->moveToOrientation(0, this->desiredAngularVel_);
			cout << "[AutoFlight]: End initial scan." << endl; 
		}		
	}

	void dynamicExploration::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
		/*	
			1. start velocity
			2. start acceleration (set to zero)
			3. end velocity
			4. end acceleration (set to zero) 
		*/

		Eigen::Vector3d currVel = this->currVel_;
		Eigen::Vector3d currAcc = this->currAcc_;
		Eigen::Vector3d endVel (0.0, 0.0, 0.0);
		Eigen::Vector3d endAcc (0.0, 0.0, 0.0);
		if (not currVel.allFinite()) currVel = Eigen::Vector3d (0.0, 0.0, 0.0);
		if (not currAcc.allFinite()) currAcc = Eigen::Vector3d (0.0, 0.0, 0.0);

		// if (not this->trajectoryReady_){
		// 	double yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		// 	Eigen::Vector3d direction (cos(yaw), sin(yaw), 0.0);
		// 	currVel = this->desiredVel_ * direction;
		// 	currAcc = this->desiredAcc_ * direction;
		// }
		startEndConditions.push_back(currVel);
		startEndConditions.push_back(endVel);
		startEndConditions.push_back(currAcc);
		startEndConditions.push_back(endAcc);
	}

	bool dynamicExploration::hasCollision(){
		if (this->trajectoryReady_){
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
				Eigen::Vector3d p = this->trajectory_.at(t);
				bool hasCollision = this->map_->isInflatedOccupied(p);
				if (hasCollision){
					return true;
				}
			}
		}
		return false;
	}

	bool dynamicExploration::hasDynamicCollision(){
		if (this->trajectoryReady_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useFakeDetector_){
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}

			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
				Eigen::Vector3d p = this->trajectory_.at(t);
				
				for (size_t i=0; i<obstaclesPos.size(); ++i){
					Eigen::Vector3d ob = obstaclesPos[i];
					Eigen::Vector3d size = obstaclesSize[i];
					Eigen::Vector3d lowerBound = ob - size/2;
					Eigen::Vector3d upperBound = ob + size/2;
					if (p(0) >= lowerBound(0) and p(0) <= upperBound(0) and
						p(1) >= lowerBound(1) and p(1) <= upperBound(1) and
						p(2) >= lowerBound(2) and p(2) <= upperBound(2)){
						return true;
					}					
				}
			}
		}
		return false;
	}

	void dynamicExploration::exploreReplan(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		// Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		// Eigen::Vector3d c1 = startPos - range;
		// Eigen::Vector3d c2 = startPos + range;
		// this->map_->freeRegion(c1, c2);
		// cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << range.transpose() << endl;

		// if (this->initialScan_){
		// 	cout << "[AutoFlight]: Start initial scan..." << endl;
		// 	this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();
						
		// 	this->moveToOrientation(-PI_const, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();

		// 	this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();
			
		// 	this->moveToOrientation(0, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: End initial scan." << endl;
		// }

		// planning thread: owns DEP. Plans exploration paths when a path is needed and handles
		// return home / inspection requests (they need roadmap paths, which can take a few seconds)
		int failCount = 0;
		while (ros::ok()){
			ros::Duration(0.05).sleep();

			int command = this->pendingCommand_.exchange(CMD_NONE);
			if (command == CMD_RETURN_HOME){
				this->expPlanner_->setMap(this->map_);
				nav_msgs::Path path;
				if (this->returnRetries_ == 0 and this->expPlanner_->findPathToPoint(this->homePos_, path)){ // a retry takes the plain climb
					this->setPendingPath(path, RETURN); // each pose faces the next leg
					this->logEvent("Returning home (" + std::to_string(int(path.poses.size()) - 1) + " legs)");
				}
				else{
					// no roadmap route: climb to the home height first, then fly across. Beams and slabs can span the pit, so in the pit
					// go around obstacles to where the map shows a clear column and climb there. Each retry keeps further from what stopped the last one
					Eigen::Vector3d p = this->currPos();
					double yawToHome = atan2(this->homePos_(1) - p(1), this->homePos_(0) - p(0));
					std::vector<Eigen::Vector3d> points {p};
					Eigen::Vector3d column = p;
					bool inPit = p(2) < this->progressRegionMax_(2) and p(0) > this->progressRegionMin_(0) and p(0) < this->progressRegionMax_(0) and p(1) > this->progressRegionMin_(1) and p(1) < this->progressRegionMax_(1);
					if (inPit){
						Eigen::Vector2d boxMin = this->progressRegionMin_.head<2>() + Eigen::Vector2d (1.0, 1.0), boxMax = this->progressRegionMax_.head<2>() - Eigen::Vector2d (1.0, 1.0);
						// keep the clearance the B-spline planner insists on beyond the inflated map, or it rejects the route
						double keep = 0.4;
						this->nh_.param<double>("bspline_traj/distance_threshold", keep, 0.4);
						auto blocked = [this, keep](const Eigen::Vector3d& q){
							if (this->map_->isInflatedOccupied(q)) return true;
							for (int axis=0; axis<3; ++axis){
								for (double sign : {-1.0, 1.0}){
									Eigen::Vector3d r = q;
									r(axis) += sign * keep;
									if (this->map_->isInflatedOccupied(r)) return true;
								}
							}
							return false;
						};
						std::vector<Eigen::Vector3d> route;
						if (not AutoFlight::findClimbRoute(p, this->homePos_(2), boxMin, boxMax, 0.2 + 0.2 * this->returnRetries_, blocked, route)){
							this->stop();
							this->stuckSince_ = ros::Time::now();
							this->logEvent("Cannot find a way home: no clear column to climb. Landing where it is in " + std::to_string(int(this->stuckLandTime_)) + " s unless you press Return home");
							continue;
						}
						if (not route.empty()) column = route.back();
						for (const Eigen::Vector3d& q : route){
							if ((q - points.back()).norm() > 0.2) points.push_back(q);
						}
					}
					points.push_back(Eigen::Vector3d (column(0), column(1), this->homePos_(2)));
					points.push_back(this->homePos_);
					this->setPendingPath(this->makePath(points, yawToHome), RETURN);
					std::ostringstream text;
					text << std::fixed << std::setprecision(1) << "Returning home: no roadmap route, climbing at x " << column(0) << ", y " << column(1);
					this->logEvent(text.str());
				}
				continue;
			}
			else if (command == CMD_INSPECT){
				Eigen::Vector3d target = this->inspectTarget_;
				Eigen::Vector3d viewpoint;
				if (not this->findInspectViewpoint(target, viewpoint)){
					this->logEvent("Inspect: no safe viewpoint near the selected point");
					if (this->missionState_ == EXPLORE) this->explorationReplan_ = true;
					continue;
				}
				double yawToTarget = atan2(target(1) - viewpoint(1), target(0) - viewpoint(0));
				this->expPlanner_->setMap(this->map_);
				nav_msgs::Path path;
				if (not this->expPlanner_->findPathToPoint(viewpoint, path)){
					this->logEvent("Inspect: no route to a viewpoint for the selected point");
					if (this->missionState_ == EXPLORE) this->explorationReplan_ = true;
					continue;
				}
				path.poses.back().pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, yawToTarget);
				this->setPendingPath(path, INSPECT);
				std::ostringstream text;
				text << std::fixed << std::setprecision(1) << "Inspecting (" << target(0) << ", " << target(1) << ", " << target(2) << ")";
				this->logEvent(text.str());
				continue;
			}

			if (this->missionState_ != EXPLORE or not this->explorationReplan_) continue;

			this->expPlanner_->setMap(this->map_);
			ros::WallTime startTime = ros::WallTime::now(); // wall clock: sim time reads short when the sim runs slower than real time
			bool replanSuccess = this->expPlanner_->makePlan();
			double planTime = (ros::WallTime::now() - startTime).toSec();
			this->frontierNum_ = this->expPlanner_->getFrontierNum();
			this->lastPlanSeconds_ = planTime;
			cout << "[AutoFlight]: DEP planning time: " << planTime << "s." << endl;
			if (this->missionState_ != EXPLORE or this->pendingCommand_ != CMD_NONE) continue; // a command arrived while planning

			if (replanSuccess){
				failCount = 0;
				this->explorationReplan_ = false;
				nav_msgs::Path bestPath = this->expPlanner_->getBestPath();
				bool tour = this->expPlanner_->isTourEnabled();
				this->recordPlan(bestPath, tour ? "tour" : "explore");
				// done when the last few flown plans barely added coverage. Counted in plans, not seconds: a long first
				// leg can show no gain for 30 s and still be useful. A plan dropped before flying says nothing about coverage
				const PlanRecord& newest = this->planLog_.back();
				int stallCount = 0, droppedInRow = 0;
				double laterCoverage = newest.coverage;
				bool countingDropped = true, countingStall = this->completeStallPlans_ > 0;
				for (int i=int(this->planLog_.size())-2; i>=0 and this->planLog_[i].run == newest.run; --i){
					const PlanRecord& p = this->planLog_[i];
					if (not p.flown){
						if (countingDropped) ++droppedInRow;
						continue;
					}
					countingDropped = false;
					if (not countingStall) break;
					if (laterCoverage - p.coverage >= this->completeMinGain_) break;
					laterCoverage = p.coverage;
					if (++stallCount >= this->completeStallPlans_) break;
				}
				bool stalled = not tour and this->completeStallPlans_ > 0 and stallCount >= this->completeStallPlans_; // a tour ends when its areas run out
				if (droppedInRow >= this->maxExplorationFailures_){
					this->logEvent("Exploration complete: " + std::to_string(droppedInRow) + " plans in a row could not be flown");
					this->explorationReplan_ = false;
					this->returnRetries_ = 0;
					this->pendingCommand_ = CMD_RETURN_HOME;
					continue;
				}
				if (stalled){
					std::ostringstream text;
					text << std::fixed << std::setprecision(0) << "Exploration complete: coverage stopped improving (pit " << this->exploredBlockPercent_.load() << "%, floor " << this->floorPercent_.load() << "%)";
					this->logEvent(text.str());
					this->explorationReplan_ = false;
					this->returnRetries_ = 0;
					this->pendingCommand_ = CMD_RETURN_HOME;
					continue;
				}
				this->setPendingPath(bestPath, EXPLORE);
			}
			else{
				if (this->expPlanner_->isTourDone()){
					failCount = 0;
					this->explorationReplan_ = false;
					std::ostringstream text;
					text << std::fixed << std::setprecision(0) << "Exploration complete: the tour has no areas left (pit " << this->exploredBlockPercent_.load() << "%, floor " << this->floorPercent_.load() << "%)";
					this->logEvent(text.str());
					this->returnRetries_ = 0;
					this->pendingCommand_ = CMD_RETURN_HOME;
					continue;
				}
				++failCount;
				if (this->frontierNum_ == 0 or failCount >= this->maxExplorationFailures_){
					failCount = 0;
					this->explorationReplan_ = false;
					this->logEvent(this->frontierNum_ == 0 ? "Exploration complete: no frontiers left" : "Exploration complete: no reachable frontiers left");
					this->pendingCommand_ = CMD_RETURN_HOME;
				}
				else{
					this->logEvent("No reachable frontier, retrying (" + std::to_string(failCount) + "/" + std::to_string(this->maxExplorationFailures_) + ")");
					ros::Duration(1.0).sleep(); // let the map grow before retrying
				}
			}
		}
	}

	void dynamicExploration::registerMissionInterface(){
		this->startSrv_ = this->nh_.advertiseService("/exploration/start", &dynamicExploration::startSrvCB, this);
		this->pauseSrv_ = this->nh_.advertiseService("/exploration/pause", &dynamicExploration::pauseSrvCB, this);
		this->resumeSrv_ = this->nh_.advertiseService("/exploration/resume", &dynamicExploration::resumeSrvCB, this);
		this->returnHomeSrv_ = this->nh_.advertiseService("/exploration/return_home", &dynamicExploration::returnHomeSrvCB, this);
		this->landSrv_ = this->nh_.advertiseService("/exploration/land", &dynamicExploration::landSrvCB, this);
		this->tourSrv_ = this->nh_.advertiseService("/exploration/tour", &dynamicExploration::tourSrvCB, this);
		this->staticSrv_ = this->nh_.advertiseService("/exploration/static", &dynamicExploration::staticSrvCB, this);
		this->inspectSub_ = this->nh_.subscribe("/exploration/inspect", 1, &dynamicExploration::inspectCB, this);
		this->statusPub_ = this->nh_.advertise<std_msgs::String>("/exploration/status", 10);
		this->landPub_ = this->nh_.advertise<std_msgs::Empty>(this->landTopic_, 1);
		this->takeoffPub_ = this->nh_.advertise<std_msgs::Empty>(this->takeoffTopic_, 1);
		this->planLogPub_ = this->nh_.advertise<std_msgs::String>("/exploration/plan_log", 1, true); // latched: a reloaded page gets the history
		this->unknownPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("/exploration/unknown_voxels", 1);
		this->floorSeenPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("/exploration/floor_seen", 1);
		this->unknownTimer_ = this->nh_.createTimer(ros::Duration(1.0), &dynamicExploration::unknownCB, this);
		this->restartTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicExploration::restartCheckCB, this);
		this->statusTimer_ = this->nh_.createTimer(ros::Duration(0.5), &dynamicExploration::statusCB, this);
		this->clearanceTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicExploration::clearanceCB, this);
	}

	bool dynamicExploration::startSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
		if (this->missionState_ == LANDED){
			this->restartMission();
			res.success = true;
			res.message = "Starting again, keeping the map";
			return true;
		}
		if (this->missionState_ != IDLE){
			res.success = false;
			res.message = "Mission already started (" + stateName(this->missionState_) + ")";
			return true;
		}
		this->startRequested_ = true;
		res.success = true;
		res.message = "Starting";
		return true;
	}

	bool dynamicExploration::tourSrvCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res){
		res.success = this->expPlanner_->setTourEnabled(req.data);
		if (not res.success){
			res.message = "Set gain_block_size in exploration_param.yaml to use the tour";
			return true;
		}
		res.message = req.data ? "Tour on: takes over from the next plan" : "Tour off: greedy goals from the next plan";
		this->logEvent(res.message);
		return true;
	}

	bool dynamicExploration::staticSrvCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res){
		this->expPlanner_->setStaticEnvironment(req.data);
		res.success = true;
		res.message = req.data ? "Static environment: any slope through fully known free space, from the next plan" : "Dynamic environment: slopes limited to what the lidar can see ahead, from the next plan";
		this->logEvent(res.message);
		return true;
	}

	bool dynamicExploration::isFloorInspected(double x, double y){
		for (double z=this->floorZ_-0.3; z<=this->floorZ_+0.3; z+=this->map_->getRes()){
			if (this->map_->isInspectedHit(Eigen::Vector3d (x, y, z))) return true;
		}
		return false;
	}

	bool dynamicExploration::isFloorHit(double x, double y){
		for (double z=this->floorZ_-0.3; z<=this->floorZ_+0.3; z+=this->map_->getRes()){
			Eigen::Vector3d p (x, y, z);
			if (this->map_->isInMap(p) and this->map_->isOccupied(p)) return true;
		}
		return false;
	}

	void dynamicExploration::restartMission(){
		// fly again after a landing: re-arm the motors and climb, then restartCheckCB continues. The map is kept
		Eigen::Vector3d p = this->currPos();
		bool nearHome = (p.head<2>() - this->homePos_.head<2>()).norm() < 1.0;
		this->finishedRuns_.push_back({(this->missionEndTime_ - this->missionStartTime_).toSec(), this->exploredPercent_, this->runLowestZ_});
		this->runLowestZ_ = std::numeric_limits<double>::infinity();
		this->runSamples_.clear();
		this->expPlanner_->resetTour();
		this->sampleClearance_ = std::numeric_limits<double>::infinity();
		this->runClearance_ = std::numeric_limits<double>::infinity();
		this->restartTargetZ_ = nearHome ? this->homePos_(2) : p(2) + this->takeoffHgt_;
		this->missionStartTime_ = ros::Time::now();
		this->missionEndTime_ = ros::Time();
		{
			std::lock_guard<std::mutex> lock (this->eventsMutex_);
			this->events_.clear();
		}
		this->pendingCommand_ = CMD_NONE;
		this->explorationReplan_ = false;
		this->pausedRemainingPath_.poses.clear();
		this->missionState_ = TAKEOFF;
		this->restartPending_ = true;
		this->targetPubEnabled_ = true;
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		ps.pose.position.z = this->restartTargetZ_;
		this->updateTarget(ps);
		this->takeoffPub_.publish(std_msgs::Empty());
		this->logEvent("Taking off again, keeping the map");
	}

	void dynamicExploration::restartCheckCB(const ros::TimerEvent&){
		if (not this->restartPending_ or std::abs(this->currPos()(2) - this->restartTargetZ_) >= 0.15) return;
		this->restartPending_ = false;
		this->logEvent("Takeoff complete");
		bool nearHome = (this->currPos().head<2>() - this->homePos_.head<2>()).norm() < 1.0;
		if (nearHome and this->initialWaypoints_.size() != 0){
			this->flyPath(this->makeEntryPath(), NUDGE);
			this->logEvent("Flying " + std::to_string(this->initialWaypoints_.size()) + " entry waypoints");
		}
		else{
			this->missionState_ = EXPLORE; // landed away from home: explore from here
			this->explorationReplan_ = true;
			this->logEvent("Exploring");
		}
	}

	bool dynamicExploration::pauseSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
		int state = this->missionState_;
		if (state != NUDGE and state != EXPLORE and state != INSPECT and state != RETURN and state != LAND){
			res.success = false;
			res.message = "Nothing to pause (" + stateName(state) + ")";
			return true;
		}
		// keep the rest of the current leg so Resume can continue it
		this->pausedRemainingPath_.poses.clear();
		int goalIdx = this->newWaypoints_ ? 1 : this->waypointIdx_ - 1;
		for (int i=std::max(goalIdx, 1); i<int(this->waypoints_.poses.size()); ++i){
			this->pausedRemainingPath_.poses.push_back(this->waypoints_.poses[i]);
		}
		{
			std::lock_guard<std::mutex> lock (this->pendingPathMutex_);
			this->pendingPathReady_ = false;
		}
		++this->motionGen_;
		this->stateBeforePause_ = state;
		this->missionState_ = PAUSED;
		this->explorationReplan_ = false;
		this->newWaypoints_ = false;
		this->replan_ = false;
		this->trajectoryReady_ = false;
		this->waypoints_.poses.clear();
		this->stop(); // hover here
		this->logEvent("Paused");
		res.success = true;
		res.message = "Paused";
		return true;
	}

	bool dynamicExploration::resumeSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
		if (this->missionState_ != PAUSED){
			res.success = false;
			res.message = "Not paused (" + stateName(this->missionState_) + ")";
			return true;
		}
		int state = this->stateBeforePause_;
		if (state == RETURN and this->pausedRemainingPath_.poses.size() == 0){
			this->missionState_ = RETURN; // paused while the way home was being planned
			this->pendingCommand_ = CMD_RETURN_HOME;
		}
		else if (state == EXPLORE or this->pausedRemainingPath_.poses.size() == 0){
			this->missionState_ = EXPLORE;
			this->explorationReplan_ = true;
		}
		else{
			nav_msgs::Path path;
			geometry_msgs::PoseStamped start;
			start.pose = this->odom_.pose.pose;
			path.poses.push_back(start);
			path.poses.insert(path.poses.end(), this->pausedRemainingPath_.poses.begin(), this->pausedRemainingPath_.poses.end());
			this->flyPath(path, state);
		}
		this->logEvent("Resumed: " + stateName(this->missionState_));
		res.success = true;
		res.message = "Resumed";
		return true;
	}

	bool dynamicExploration::returnHomeSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
		int state = this->missionState_;
		if (state == IDLE or state == TAKEOFF or state == LAND or state == LANDED){
			res.success = false;
			res.message = "Cannot return home now (" + stateName(state) + ")";
			return true;
		}
		// hover while the way home is planned, so the current leg cannot finish and be taken for "home reached"
		++this->motionGen_;
		{
			std::lock_guard<std::mutex> lock (this->pendingPathMutex_);
			this->pendingPathReady_ = false;
		}
		this->explorationReplan_ = false;
		this->newWaypoints_ = false;
		this->replan_ = false;
		this->trajectoryReady_ = false;
		this->waypoints_.poses.clear();
		this->stop();
		this->missionState_ = RETURN;
		this->returnRetries_ = 0;
		this->stuckSince_ = ros::Time();
		this->pendingCommand_ = CMD_RETURN_HOME;
		this->logEvent("Return home requested");
		res.success = true;
		res.message = "Planning the way home";
		return true;
	}

	bool dynamicExploration::landSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
		int state = this->missionState_;
		if (state == IDLE or state == TAKEOFF or state == LAND or state == LANDED){
			res.success = false;
			res.message = "Cannot land now (" + stateName(state) + ")";
			return true;
		}
		this->pendingCommand_ = CMD_NONE;
		this->stuckSince_ = ros::Time();
		this->startLanding();
		res.success = true;
		res.message = "Landing";
		return true;
	}

	void dynamicExploration::inspectCB(const geometry_msgs::PointStampedConstPtr& target){
		int state = this->missionState_;
		if (state != EXPLORE and state != NUDGE and state != INSPECT and state != PAUSED){
			this->logEvent("Inspect ignored while " + stateName(state));
			return;
		}
		this->inspectTarget_ = Eigen::Vector3d (target->point.x, target->point.y, target->point.z);
		if (state == PAUSED){
			this->missionState_ = EXPLORE; // the inspection replaces the paused leg
		}
		this->explorationReplan_ = false;
		this->pendingCommand_ = CMD_INSPECT;
	}

	void dynamicExploration::onPathFinished(bool success, const char* reason){
		// the current leg ended (reached its last waypoint, or was aborted): decide what comes next
		int state = this->missionState_;
		if (state == EXPLORE){
			this->lastPathEnd_ = reason;
			if (not success) this->logEvent(std::string("Plan dropped: ") + reason);
		}
		if (state == NUDGE){
			this->missionState_ = EXPLORE;
			this->explorationReplan_ = true;
			this->logEvent(success ? "Entry waypoints done: exploring" : "Entry waypoints aborted: exploring from here");
		}
		else if (state == EXPLORE){
			this->explorationReplan_ = true;
		}
		else if (state == INSPECT){
			if (success){
				this->logEvent("At inspection viewpoint, holding " + std::to_string(int(this->inspectHoldTime_)) + " s");
				this->waitTime(this->inspectHoldTime_);
				this->logEvent("Inspection done: exploring");
			}
			else{
				this->logEvent("Inspection leg aborted: exploring");
			}
			if (this->missionState_ == INSPECT){ // not changed by a command while holding
				this->missionState_ = EXPLORE;
				this->explorationReplan_ = true;
			}
		}
		else if (state == RETURN){
			if (success and (this->currPos() - this->homePos_).norm() < 1.0){
				this->logEvent("Home reached");
				this->startLanding();
			}
			else if (this->returnRetries_ < 5){
				++this->returnRetries_;
				this->logEvent("Return leg aborted, replanning (" + std::to_string(this->returnRetries_) + "/5)");
				this->pendingCommand_ = CMD_RETURN_HOME;
			}
			else{
				// never land on its own where it happens to be: that can be the bottom of the pit
				this->stop();
				this->stuckSince_ = ros::Time::now();
				this->logEvent("Cannot find a way home: hovering. Landing where it is in " + std::to_string(int(this->stuckLandTime_)) + " s unless you press Return home");
			}
		}
		else if (state == LAND){
			if (success){
				this->touchdown();
			}
			else{
				// descend straight down with position control; touchdown detection finishes the landing
				geometry_msgs::PoseStamped ps;
				ps.pose = this->odom_.pose.pose;
				ps.pose.position.z = this->landTarget_(2);
				this->updateTarget(ps);
				this->descentStarted_ = true;
				this->touchdownCheckTime_ = ros::Time::now();
			}
		}
	}

	void dynamicExploration::setPendingPath(const nav_msgs::Path& path, int state){
		if (path.poses.size() < 2) return;
		std::lock_guard<std::mutex> lock (this->pendingPathMutex_);
		this->pendingPath_ = path;
		this->pendingPathState_ = state;
		this->pendingPathReady_ = true;
	}

	void dynamicExploration::applyPendingPath(){
		nav_msgs::Path path;
		int state;
		{
			std::lock_guard<std::mutex> lock (this->pendingPathMutex_);
			if (not this->pendingPathReady_) return;
			path = this->pendingPath_;
			state = this->pendingPathState_;
			this->pendingPathReady_ = false;
		}
		int currState = this->missionState_;
		if (currState == PAUSED or currState == LAND or currState == LANDED) return; // a command overrode this path
		if (state == EXPLORE and currState != EXPLORE) return; // stale exploration path
		this->flyPath(path, state);
	}

	void dynamicExploration::flyPath(const nav_msgs::Path& path, int state){
		// main thread only
		if (path.poses.size() < 2) return;
		++this->motionGen_;
		this->waypoints_ = path;
		this->waypointIdx_ = 1;
		this->trajectoryReady_ = false;
		this->replan_ = false;
		this->missionState_ = state;
		if (state == LAND){
			this->descentStarted_ = false;
			this->touchdownCheckTime_ = ros::Time::now();
		}
		this->newWaypoints_ = true;
	}

	void dynamicExploration::startLanding(){
		// main thread only: descend to just above the mapped ground under the robot
		Eigen::Vector3d p = this->currPos();
		double res = this->map_->getRes();
		double groundTop = std::numeric_limits<double>::quiet_NaN();
		if ((Eigen::Vector2d (p(0), p(1)) - this->homePos_.head<2>()).norm() < 1.0){
			groundTop = this->homeGroundZ_; // the takeoff spot
		}
		else{
			for (double z=p(2); this->map_->isInMap(Eigen::Vector3d (p(0), p(1), z)); z-=res){
				if (this->map_->isOccupied(Eigen::Vector3d (p(0), p(1), z))){
					groundTop = z + res/2;
					break;
				}
			}
		}
		if (std::isnan(groundTop)){
			// ground under the robot is not mapped: descend until touchdown is detected
			groundTop = p(2) - 50.0;
			while (not this->map_->isInMap(Eigen::Vector3d (p(0), p(1), groundTop))) groundTop += res;
			this->logEvent("Landing: ground below not mapped, descending until touchdown");
		}
		this->landTarget_ = Eigen::Vector3d (p(0), p(1), std::min(groundTop + this->landClearance_, p(2)));
		this->pausedRemainingPath_.poses.clear();
		this->explorationReplan_ = false;
		this->touchdownCheckTime_ = ros::Time::now();
		if (p(2) - this->landTarget_(2) < 0.1){
			this->missionState_ = LAND;
			this->touchdown();
			return;
		}
		this->flyPath(this->makePath({p, this->landTarget_}, this->currYaw()), LAND);
		this->logEvent("Landing");
	}

	void dynamicExploration::touchdown(){
		++this->motionGen_;
		this->trajectoryReady_ = false;
		this->replan_ = false;
		this->newWaypoints_ = false;
		this->waypoints_.poses.clear();
		this->targetPubEnabled_ = false;
		this->landPub_.publish(std_msgs::Empty());
		this->missionState_ = LANDED;
		this->missionEndTime_ = ros::Time::now();
		this->saveRun();
		this->logEvent("Landed");
	}

	bool dynamicExploration::findInspectViewpoint(const Eigen::Vector3d& target, Eigen::Vector3d& viewpoint){
		// a known-free spot about inspect_distance from the target, preferring the side the robot is on,
		// with a clear line of sight to the target
		Eigen::Vector3d p = this->currPos();
		double baseYaw = atan2(p(1) - target(1), p(0) - target(0));
		std::vector<double> yawOffsets {0.0, 0.5, -0.5, 1.0, -1.0, 1.6, -1.6, 2.2, -2.2, PI_const};
		std::vector<double> distances {this->inspectDistance_, this->inspectDistance_ * 0.75, this->inspectDistance_ * 1.3};
		std::vector<double> heights {0.0, 1.0, -1.0};
		for (double dh : heights){
			for (double d : distances){
				for (double dyaw : yawOffsets){
					double yaw = baseYaw + dyaw;
					Eigen::Vector3d candidate = target + Eigen::Vector3d (d * cos(yaw), d * sin(yaw), dh);
					if (not this->expPlanner_->isPosValid(candidate)) continue;
					Eigen::Vector3d toTarget = target - candidate;
					Eigen::Vector3d nearTarget = target - toTarget.normalized() * 0.5; // the target itself is a surface
					if (not this->map_->isInflatedFreeLine(candidate, nearTarget)) continue;
					viewpoint = candidate;
					return true;
				}
			}
		}
		return false;
	}

	nav_msgs::Path dynamicExploration::makePath(const std::vector<Eigen::Vector3d>& points, double finalYaw){
		nav_msgs::Path path;
		for (size_t i=0; i<points.size(); ++i){
			geometry_msgs::PoseStamped ps;
			ps.header.frame_id = "map";
			ps.pose.position.x = points[i](0);
			ps.pose.position.y = points[i](1);
			ps.pose.position.z = points[i](2);
			ps.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, finalYaw);
			path.poses.push_back(ps);
		}
		return path;
	}

	void dynamicExploration::publishCloud(ros::Publisher& pub, const std::vector<Eigen::Vector3d>& points){
		sensor_msgs::PointCloud2 cloud;
		cloud.header.frame_id = "map";
		cloud.header.stamp = ros::Time::now();
		cloud.height = 1;
		sensor_msgs::PointCloud2Modifier modifier (cloud);
		modifier.setPointCloud2FieldsByString(1, "xyz");
		modifier.resize(points.size());
		sensor_msgs::PointCloud2Iterator<float> iterX (cloud, "x"), iterY (cloud, "y"), iterZ (cloud, "z");
		for (const Eigen::Vector3d& p : points){
			*iterX = p(0); *iterY = p(1); *iterZ = p(2);
			++iterX; ++iterY; ++iterZ;
		}
		pub.publish(cloud);
	}

	void dynamicExploration::clearanceCB(const ros::TimerEvent&){
		// drone centre to the surface of the nearest occupied voxel within 1.5 m. Faster than the 2 Hz log,
		// which can fly past a beam between samples
		if (this->missionStartTime_.isZero() or this->missionState_ == LANDED) return;
		Eigen::Vector3d p = this->currPos();
		double res = this->map_->getRes(), clearance = std::numeric_limits<double>::infinity();
		int range = int(std::ceil(1.5 / res));
		for (int i=-range; i<=range; ++i){
			for (int j=-range; j<=range; ++j){
				for (int k=-range; k<=range; ++k){
					Eigen::Vector3d offset (i * res, j * res, k * res);
					Eigen::Vector3d q = p + offset;
					if (not this->map_->isInMap(q) or not this->map_->isOccupied(q)) continue;
					Eigen::Vector3d gap = (offset.cwiseAbs().array() - res/2).max(0.0).matrix();
					clearance = std::min(clearance, gap.norm());
				}
			}
		}
		this->sampleClearance_ = std::min(this->sampleClearance_, clearance);
		if (this->missionState_ != TAKEOFF and this->missionState_ != LAND) this->runClearance_ = std::min(this->runClearance_, clearance);
	}

	void dynamicExploration::unknownCB(const ros::TimerEvent&){
		// a thin lidar ray leaves unknown voxels between known ones, so a single voxel says little: only a 0.6 m
		// block with no known voxel at all counts as unexplored
		double res = this->map_->getRes(), block = 3.0 * res;
		std::vector<Eigen::Vector3d> blocks;
		long total = 0;
		for (double x=this->progressRegionMin_(0)+block/2; x<this->progressRegionMax_(0); x+=block){
			for (double y=this->progressRegionMin_(1)+block/2; y<this->progressRegionMax_(1); y+=block){
				for (double z=this->progressRegionMin_(2)+block/2; z<this->progressRegionMax_(2); z+=block){
					++total;
					bool anyKnown = false, pending = false; // open air clears from any range, a surface only from inspect_range
					for (int i=-1; i<=1 and not pending; ++i){
						for (int j=-1; j<=1 and not pending; ++j){
							for (int k=-1; k<=1 and not pending; ++k){
								Eigen::Vector3d p (x + i * res, y + j * res, z + k * res);
								anyKnown = anyKnown or (this->map_->isInMap(p) and not this->map_->isUnknown(p));
								pending = this->map_->isInspectionPending(p);
							}
						}
					}
					if (not anyKnown or pending) blocks.push_back(Eigen::Vector3d (x, y, z));
				}
			}
		}
		if (total != 0) this->exploredBlockPercent_ = 100.0 * double(total - long(blocks.size())) / double(total);

		// needs no floor height, so it works on any structure. Reaches below the box so a floor just under it counts
		if (this->map_->getInspectRange() > 0.0){
			long confirmed = 0, pendingVoxels = 0;
			for (double x=this->progressRegionMin_(0); x<this->progressRegionMax_(0); x+=res){
				for (double y=this->progressRegionMin_(1); y<this->progressRegionMax_(1); y+=res){
					for (double z=this->progressRegionMin_(2)-block; z<this->progressRegionMax_(2); z+=res){
						Eigen::Vector3d p (x, y, z);
						if (this->map_->isInspectedHit(p)) ++confirmed;
						else if (this->map_->isInspectionPending(p)) ++pendingVoxels;
					}
				}
			}
			if (confirmed + pendingVoxels != 0) this->surfacePercent_ = 100.0 * double(confirmed) / double(confirmed + pendingVoxels);
		}
		if (this->unknownPub_.getNumSubscribers() != 0) this->publishCloud(this->unknownPub_, blocks);

		if (this->floorSeenPub_.getNumSubscribers() != 0){
			std::vector<Eigen::Vector3d> seen;
			{
				std::lock_guard<std::mutex> lock (this->floorSeenMutex_);
				if (not this->floorSeen_.empty()){
					int rows = int(this->floorSeen_.size()) / this->floorCols_;
					for (int i=0; i<this->floorCols_; ++i){
						for (int j=0; j<rows; ++j){
							if (this->floorSeen_[i * rows + j]) seen.push_back(Eigen::Vector3d (this->progressRegionMin_(0) + (i + 0.5) * res, this->progressRegionMin_(1) + (j + 0.5) * res, this->floorZ_));
						}
					}
				}
			}
			this->publishCloud(this->floorSeenPub_, seen);
		}
	}

	// Orthographic view of the flown path inside the pit wireframe, from the +x, -y side and 30 deg above. Boxes come
	// from the world model the launch file loads; the ground around the pit is left out. Sets height, returns SVG
	static std::string orthoView(const std::string& sdf, const std::vector<Eigen::Vector3d>& path, const std::vector<Eigen::Vector3d>& goals,
	                             const Eigen::Vector3d& home, double left, double top, double width, double& height){
		const double az = -35.0 * M_PI / 180.0, el = 30.0 * M_PI / 180.0;
		const Eigen::Vector3d right (-sin(az), cos(az), 0), up (-sin(el) * cos(az), -sin(el) * sin(az), cos(el));
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> boxes; // center, size
		size_t at = 0;
		while ((at = sdf.find("<collision name=", at)) != std::string::npos){
			size_t end = sdf.find("</collision>", at);
			if (end == std::string::npos) break;
			std::string block = sdf.substr(at, end - at);
			at = end;
			size_t pose = block.find("<pose>"), size = block.find("<size>");
			if (block.find("top_ground") != std::string::npos or pose == std::string::npos or size == std::string::npos) continue;
			std::istringstream poseText (block.substr(pose + 6)), sizeText (block.substr(size + 6));
			Eigen::Vector3d c, d;
			poseText >> c(0) >> c(1) >> c(2);
			sizeText >> d(0) >> d(1) >> d(2);
			if (poseText and sizeText) boxes.push_back({c, d});
		}

		// fit: same scale as the other views unless the view would be wider than they are
		double uMin = std::numeric_limits<double>::infinity(), uMax = -uMin, vMin = uMin, vMax = -uMin;
		auto fit = [&](const Eigen::Vector3d& p){
			uMin = std::min(uMin, p.dot(right)); uMax = std::max(uMax, p.dot(right));
			vMin = std::min(vMin, p.dot(up)); vMax = std::max(vMax, p.dot(up));
		};
		auto corner = [](const std::pair<Eigen::Vector3d, Eigen::Vector3d>& b, int i){
			return Eigen::Vector3d (b.first(0) + ((i & 1) ? 0.5 : -0.5) * b.second(0), b.first(1) + ((i & 2) ? 0.5 : -0.5) * b.second(1), b.first(2) + ((i & 4) ? 0.5 : -0.5) * b.second(2));
		};
		for (const auto& b : boxes) for (int i=0; i<8; ++i) fit(corner(b, i));
		for (const Eigen::Vector3d& p : path) fit(p);
		for (const Eigen::Vector3d& p : goals) fit(p);
		fit(home);
		double scale = std::min(40.0, width / std::max(uMax - uMin, 1.0));
		height = (vMax - vMin) * scale;
		auto sx = [&](const Eigen::Vector3d& p){return left + (p.dot(right) - uMin) * scale;};
		auto sy = [&](const Eigen::Vector3d& p){return top + (vMax - p.dot(up)) * scale;};

		std::ostringstream svg;
		svg << std::fixed << std::setprecision(1);
		svg << "<path fill='none' stroke='#9AA0A6' stroke-width='1' d='";
		for (const auto& b : boxes){
			for (int i=0; i<8; ++i){
				for (int axis=1; axis<8; axis*=2){
					if (i & axis) continue; // each edge once, from its lower corner
					Eigen::Vector3d p = corner(b, i), q = corner(b, i | axis);
					svg << "M" << sx(p) << " " << sy(p) << "L" << sx(q) << " " << sy(q);
				}
			}
		}
		svg << "'/>\n<polyline fill='none' stroke='#C41230' stroke-width='1.5' points='";
		for (const Eigen::Vector3d& p : path) svg << sx(p) << "," << sy(p) << " ";
		svg << "'/>\n";
		for (const Eigen::Vector3d& g : goals) svg << "<circle cx='" << sx(g) << "' cy='" << sy(g) << "' r='3' fill='#000'/>\n";
		svg << "<circle cx='" << sx(home) << "' cy='" << sy(home) << "' r='8' fill='none' stroke='#000' stroke-width='2'/>\n";
		return svg.str();
	}

	void dynamicExploration::saveRun(){
		// one CSV and one picture (flight path from above and from the side) per landed run
		if (this->runSamples_.empty()) return;
		if (this->runSaveDir_.empty()){
			const char* home = getenv("HOME");
			this->runSaveDir_ = std::string(home == NULL ? "." : home) + "/mission_runs";
			mkdir(this->runSaveDir_.c_str(), 0755);
		}
		char stamp[32];
		std::time_t now = std::time(NULL);
		std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
		int run = int(this->finishedRuns_.size()) + 1;
		std::string base = this->runSaveDir_ + "/" + stamp + (this->preset_.empty() ? "" : "_" + this->preset_) + (this->expPlanner_->isStaticEnvironment() ? "_static" : "") + "_run" + std::to_string(run);

		std::ofstream csv (base + ".csv");
		csv << std::fixed << std::setprecision(2);
		csv << "run,t_s,type,state,x_m,y_m,z_m,z_target_m,pit_pct,pit_block_pct,floor_pct,floor_now_pct,kind,n,goal_x,goal_y,goal_z,dist,path_length,legs,turn_deg,score,predicted_voxels,learned_voxels,path_time_s,candidates,revisits,roadmap_nodes,reachable_nodes,plan_time_s,flown,plan_end,tour_cells_left,tour_areas_given_up,min_clearance_m,surface_pct\n";
		std::vector<std::pair<double, std::string>> rows;
		for (const RunSample& r : this->runSamples_){
			std::ostringstream row;
			row << std::fixed << std::setprecision(2) << run << "," << r.t << ",height," << r.state << "," << r.x << "," << r.y << "," << r.z << ",";
			if (not std::isnan(r.zTarget)) row << r.zTarget;
			row << "," << r.pit << "," << r.pitBlock << "," << r.floor << "," << r.floorNow << ",,,,,,,,,,,,,,,,,,,,,,,";
			if (not std::isinf(r.clearance)) row << r.clearance;
			row << ",";
			if (r.surface >= 0.0) row << r.surface;
			rows.push_back({r.t, row.str()});
		}
		std::vector<Eigen::Vector3d> goals;
		for (size_t i=0; i<this->planLog_.size(); ++i){
			const PlanRecord& p = this->planLog_[i];
			if (p.run != run) continue;
			goals.push_back(p.goal);
			std::ostringstream row;
			row << std::fixed << std::setprecision(2) << run << "," << p.t << ",waypoint,,,,,,,,,," << p.kind << "," << i + 1 << "," << p.goal(0) << "," << p.goal(1) << "," << p.goal(2) << "," << (p.goal - p.from).norm() << ","
				<< p.pathLength << "," << p.legs << "," << p.turn << "," << p.score << "," << p.gain << "," << p.learned << "," << p.pathTime << "," << p.candidates << "," << p.revisits << "," << p.nodes << "," << p.reachable << "," << p.planSeconds << "," << (p.flown ? 1 : 0) << "," << p.ended << ",";
			if (p.kind == "tour") row << p.tourCellsLeft << "," << p.tourGivenUp;
			else row << ",";
			row << ",,";
			rows.push_back({p.t, row.str()});
		}
		std::stable_sort(rows.begin(), rows.end(), [](const std::pair<double, std::string>& a, const std::pair<double, std::string>& b){return a.first < b.first;});
		for (const std::pair<double, std::string>& row : rows) csv << row.second << "\n";

		// picture: same scale in both views, pit outline, home, chosen goals and the flown path
		Eigen::Vector3d lo = this->progressRegionMin_, hi = this->progressRegionMax_;
		double lowest = std::numeric_limits<double>::infinity();
		for (const RunSample& r : this->runSamples_){
			lo = lo.cwiseMin(Eigen::Vector3d (r.x, r.y, r.z));
			hi = hi.cwiseMax(Eigen::Vector3d (r.x, r.y, r.z));
			lowest = std::min(lowest, r.z);
		}
		lo = lo.cwiseMin(this->homePos_) - Eigen::Vector3d (1, 1, 1);
		hi = hi.cwiseMax(this->homePos_) + Eigen::Vector3d (1, 1, 1);
		double scale = 40.0, margin = 50.0, topY = 90.0;
		double width = (hi(1) - lo(1)) * scale;
		double xyHeight = (hi(0) - lo(0)) * scale, xzHeight = (hi(2) - lo(2)) * scale;
		// long pit axis runs left to right in both views: horizontal is y
		auto sx = [&](double y){return margin + (y - lo(1)) * scale;};
		auto syTop = [&](double x){return topY + (hi(0) - x) * scale;};
		double sideTop = topY + xyHeight + 70.0;
		auto sySide = [&](double z){return sideTop + (hi(2) - z) * scale;};
		std::string worldSdf;
		this->nh_.getParam("autonomous_flight/world_sdf", worldSdf);
		std::vector<Eigen::Vector3d> flown;
		for (const RunSample& r : this->runSamples_) flown.push_back(Eigen::Vector3d (r.x, r.y, r.z));
		double orthoTop = sideTop + xzHeight + 70.0, orthoHeight = 0.0;
		std::string ortho = orthoView(worldSdf, flown, goals, this->homePos_, margin, orthoTop, width, orthoHeight);
		std::ofstream svg (base + ".svg");
		svg << std::fixed << std::setprecision(1);
		svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width + 2 * margin << "' height='" << orthoTop + orthoHeight + 40 << "' font-family='sans-serif' font-size='14'>\n";
		svg << "<rect width='100%' height='100%' fill='white'/>\n";
		svg << "<text x='" << margin << "' y='28' font-size='18' font-weight='bold'>Run " << run << ", " << stamp << (this->preset_.empty() ? "" : ", " + this->preset_) << "</text>\n";
		svg << "<text x='" << margin << "' y='52'>" << this->runSamples_.back().t << " s, pit " << this->exploredBlockPercent_.load() << "% (voxels " << this->exploredPercent_ << "%), floor " << this->floorPercent_.load() << "%, lowest z " << lowest << " m, closest obstacle ";
		if (std::isinf(this->runClearance_)) svg << "over 1.5"; else svg << std::setprecision(2) << this->runClearance_ << std::setprecision(1);
		svg << " m, " << goals.size() << " goals</text>\n";
		svg << "<text x='" << margin << "' y='" << topY - 8 << "' font-weight='bold'>From above (horizontal y, vertical x)</text>\n";
		svg << "<text x='" << margin << "' y='" << sideTop - 8 << "' font-weight='bold'>From the side (horizontal y, vertical z)</text>\n";
		svg << "<rect x='" << sx(this->progressRegionMin_(1)) << "' y='" << syTop(this->progressRegionMax_(0)) << "' width='" << (this->progressRegionMax_(1) - this->progressRegionMin_(1)) * scale << "' height='" << (this->progressRegionMax_(0) - this->progressRegionMin_(0)) * scale << "' fill='#E0E0E0' stroke='#6D6E71'/>\n";
		svg << "<rect x='" << sx(this->progressRegionMin_(1)) << "' y='" << sySide(this->progressRegionMax_(2)) << "' width='" << (this->progressRegionMax_(1) - this->progressRegionMin_(1)) * scale << "' height='" << (this->progressRegionMax_(2) - this->progressRegionMin_(2)) * scale << "' fill='#E0E0E0' stroke='#6D6E71'/>\n";
		svg << "<polyline fill='none' stroke='#C41230' stroke-width='1.5' points='";
		for (const RunSample& r : this->runSamples_) svg << sx(r.y) << "," << syTop(r.x) << " ";
		svg << "'/>\n<polyline fill='none' stroke='#C41230' stroke-width='1.5' points='";
		for (const RunSample& r : this->runSamples_) svg << sx(r.y) << "," << sySide(r.z) << " ";
		svg << "'/>\n";
		for (const Eigen::Vector3d& g : goals){
			svg << "<circle cx='" << sx(g(1)) << "' cy='" << syTop(g(0)) << "' r='3' fill='#000'/><circle cx='" << sx(g(1)) << "' cy='" << sySide(g(2)) << "' r='3' fill='#000'/>\n";
		}
		svg << "<circle cx='" << sx(this->homePos_(1)) << "' cy='" << syTop(this->homePos_(0)) << "' r='8' fill='none' stroke='#000' stroke-width='2'/><circle cx='" << sx(this->homePos_(1)) << "' cy='" << sySide(this->homePos_(2)) << "' r='8' fill='none' stroke='#000' stroke-width='2'/>\n";
		svg << "<text x='" << margin << "' y='" << orthoTop - 8 << "' font-weight='bold'>Orthographic view (from +x, -y, 30 deg above)</text>\n" << ortho;
		svg << "<text x='" << margin << "' y='" << orthoTop + orthoHeight + 24 << "' fill='#6D6E71'>Red: flown path. Black dots: chosen goals. Ring: home. Gray: pit interior, and pit walls, columns and beams in the orthographic view.</text>\n</svg>\n";
		this->logEvent("Saved this run: " + base + ".svg and .csv");
	}

	void dynamicExploration::statusCB(const ros::TimerEvent&){
		// no way home and nobody stepped in: land where it is, so an unattended run ends and saves its log
		if (not this->stuckSince_.isZero() and (ros::Time::now() - this->stuckSince_).toSec() >= this->stuckLandTime_){
			this->stuckSince_ = ros::Time();
			if (this->missionState_ == RETURN){
				this->logEvent("No way home after " + std::to_string(int(this->stuckLandTime_)) + " s: landing where it is");
				this->pendingCommand_ = CMD_NONE;
				this->startLanding();
			}
		}
		// % explored: known voxels in the progress region (main thread: owns the map)
		Eigen::Vector3d size = this->progressRegionMax_ - this->progressRegionMin_;
		if (size.minCoeff() > 0){
			double res = this->map_->getRes();
			long known = 0, total = 0;
			for (double x=this->progressRegionMin_(0)+res/2; x<this->progressRegionMax_(0); x+=res){
				for (double y=this->progressRegionMin_(1)+res/2; y<this->progressRegionMax_(1); y+=res){
					for (double z=this->progressRegionMin_(2)+res/2; z<this->progressRegionMax_(2); z+=res){
						Eigen::Vector3d p (x, y, z);
						if (not this->map_->isInMap(p)) continue;
						++total;
						if (not this->map_->isUnknown(p)) ++known;
					}
				}
			}
			this->exploredPercent_ = total == 0 ? 0.0 : 100.0 * double(known) / double(total);
			this->exploredVolume_ = double(known) * pow(res, 3);
		}

		// % floor covered: floor columns of the progress region where the lidar has hit the floor
		if (not std::isnan(this->floorZ_) and size.minCoeff() > 0){
			double res = this->map_->getRes();
			int cols = int(std::ceil(size(0)/res)), rows = int(std::ceil(size(1)/res));
			long seen = 0, now = 0;
			std::lock_guard<std::mutex> lock (this->floorSeenMutex_);
			if (this->floorSeen_.empty()){
				this->floorSeen_.assign(cols * rows, 0);
				this->floorCols_ = cols;
			}
			for (int i=0; i<cols; ++i){
				for (int j=0; j<rows; ++j){
					double cellX = this->progressRegionMin_(0) + (i + 0.5) * res, cellY = this->progressRegionMin_(1) + (j + 0.5) * res;
					if (this->isFloorHit(cellX, cellY)) ++now;
					if (this->isFloorInspected(cellX, cellY)) this->floorSeen_[i * rows + j] = 1;
					if (this->floorSeen_[i * rows + j]) ++seen;
				}
			}
			this->floorPercent_ = 100.0 * double(seen) / double(cols * rows);
			this->floorNowPercent_ = 100.0 * double(now) / double(cols * rows);
		}

		Eigen::Vector3d p = this->currPos();
		double elapsed = this->missionStartTime_.isZero() ? 0.0 : ((this->missionEndTime_.isZero() ? ros::Time::now() : this->missionEndTime_) - this->missionStartTime_).toSec();
		if (not this->missionStartTime_.isZero() and this->missionState_ != LANDED){
			this->runLowestZ_ = std::min(this->runLowestZ_, p(2));
			this->runSamples_.push_back({stateName(this->missionState_), elapsed, p(0), p(1), p(2), this->lastTargetZ_, this->exploredPercent_, this->exploredBlockPercent_.load(), this->floorPercent_.load(), this->floorNowPercent_, this->sampleClearance_, this->surfacePercent_.load()});
			this->sampleClearance_ = std::numeric_limits<double>::infinity();
		}
		std::ostringstream json;
		json << std::fixed << std::setprecision(2);
		json << "{\"state\":\"" << stateName(this->missionState_) << "\"";
		json << ",\"paused_state\":\"" << (this->missionState_ == PAUSED ? stateName(this->stateBeforePause_) : "") << "\"";
		json << ",\"elapsed\":" << elapsed;
		json << ",\"run\":" << this->finishedRuns_.size() + 1;
		json << ",\"finished_runs\":[";
		for (size_t i=0; i<this->finishedRuns_.size(); ++i){
			json << (i == 0 ? "" : ",") << "{\"elapsed\":" << this->finishedRuns_[i][0] << ",\"explored_pct\":" << this->finishedRuns_[i][1] << ",\"lowest_z\":" << this->finishedRuns_[i][2] << "}";
		}
		json << "]";
		json << ",\"explored_pct\":" << this->exploredPercent_;
		json << ",\"floor_pct\":" << (std::isnan(this->floorZ_) ? -1.0 : this->floorPercent_.load());
		json << ",\"explored_block_pct\":" << this->exploredBlockPercent_.load();
		json << ",\"z_target\":";
		if (std::isnan(this->lastTargetZ_)) json << "null"; else json << this->lastTargetZ_;
		json << ",\"floor_now_pct\":" << (std::isnan(this->floorZ_) ? -1.0 : this->floorNowPercent_);
		json << ",\"inspect_range\":" << this->map_->getInspectRange();
		json << ",\"preset\":\"" << this->preset_ << "\"";
		json << ",\"surface_pct\":" << this->surfacePercent_.load();
		json << ",\"tour\":" << (this->expPlanner_ and this->expPlanner_->isTourEnabled() ? "true" : "false");
		json << ",\"static_env\":" << (this->expPlanner_ and this->expPlanner_->isStaticEnvironment() ? "true" : "false");
		json << ",\"explored_m3\":" << this->exploredVolume_;
		json << ",\"frontiers\":" << this->frontierNum_;
		json << ",\"map_res\":" << this->map_->getRes();
		json << ",\"position\":[" << p(0) << "," << p(1) << "," << p(2) << "]";
		json << ",\"yaw\":" << this->currYaw();
		json << ",\"home\":[" << this->homePos_(0) << "," << this->homePos_(1) << "," << this->homePos_(2) << "]";
		json << ",\"region_min\":[" << this->progressRegionMin_(0) << "," << this->progressRegionMin_(1) << "," << this->progressRegionMin_(2) << "]";
		json << ",\"region_max\":[" << this->progressRegionMax_(0) << "," << this->progressRegionMax_(1) << "," << this->progressRegionMax_(2) << "]";
		json << ",\"events\":[";
		{
			std::lock_guard<std::mutex> lock (this->eventsMutex_);
			for (size_t i=0; i<this->events_.size(); ++i){
				json << (i == 0 ? "" : ",") << "{\"t\":" << this->events_[i].first << ",\"text\":\"" << this->events_[i].second << "\"}";
			}
		}
		json << "]}";
		std_msgs::String msg;
		msg.data = json.str();
		this->statusPub_.publish(msg);
	}

	void dynamicExploration::recordPlan(const nav_msgs::Path& path, const std::string& kind){
		// planning thread
		PlanRecord r;
		r.kind = kind;
		r.run = int(this->finishedRuns_.size()) + 1;
		r.t = (ros::Time::now() - this->missionStartTime_).toSec();
		r.from = this->currPos();
		const geometry_msgs::Point& g = path.poses.back().pose.position;
		r.goal = Eigen::Vector3d (g.x, g.y, g.z);
		r.pathLength = 0.0;
		r.legs = int(path.poses.size()) - 1;
		for (size_t i=0; i+1<path.poses.size(); ++i){
			const geometry_msgs::Point& a = path.poses[i].pose.position;
			const geometry_msgs::Point& b = path.poses[i+1].pose.position;
			r.pathLength += Eigen::Vector3d (b.x - a.x, b.y - a.y, b.z - a.z).norm();
		}
		Eigen::Vector3d delta = r.goal - r.from;
		r.heading = atan2(delta(1), delta(0)) * 180.0 / PI_const;
		r.turn = 0.0;
		r.revisits = 0;
		double res = this->map_->getRes();
		r.knownAtPlan = this->exploredVolume_ / pow(res, 3);
		r.coverage = this->exploredBlockPercent_.load(); // decides when to stop: no floor % here, the planner does not know the floor
		r.learned = -1.0;
		r.flown = false;
		r.planSeconds = this->lastPlanSeconds_;
		r.tourCellsLeft = r.tourGivenUp = 0;
		if (kind == "tour") this->expPlanner_->getTourStats(r.tourCellsLeft, r.tourGivenUp);
		this->expPlanner_->getBestPathScore(r.score, r.gain, r.pathTime, r.candidates);
		this->expPlanner_->getRoadmapStats(r.nodes, r.reachable);
		if (not this->planLog_.empty() and this->planLog_.back().run == r.run){
			r.turn = std::abs(fmod(r.heading - this->planLog_.back().heading + 540.0, 360.0) - 180.0);
		}
		if (not this->planLog_.empty()){
			PlanRecord& previous = this->planLog_.back();
			previous.learned = r.knownAtPlan - previous.knownAtPlan;
			previous.flown = previous.run == r.run and (r.from - previous.from).norm() >= 0.75; // more than hover drift or overshoot
			previous.ended = this->lastPathEnd_.exchange("");
		}
		for (const PlanRecord& p : this->planLog_){
			if (p.run == r.run and (p.goal - r.goal).norm() < 1.0) ++r.revisits;
		}
		this->planLog_.push_back(r);
		cout << "[Plan " << this->planLog_.size() << "]: goal " << r.goal.transpose() << " delta " << delta.transpose() << " turn " << r.turn << " deg, score " << r.score << ", predicted voxels " << r.gain << ", path time " << r.pathTime << "s, revisits " << r.revisits << ", roadmap reachable " << r.reachable << "/" << r.nodes << endl;

		std::ostringstream json;
		json << std::fixed << std::setprecision(2) << "[";
		for (size_t i=0; i<this->planLog_.size(); ++i){
			const PlanRecord& p = this->planLog_[i];
			json << (i == 0 ? "" : ",") << "{\"kind\":\"" << p.kind << "\",\"run\":" << p.run << ",\"t\":" << p.t
				<< ",\"from\":[" << p.from(0) << "," << p.from(1) << "," << p.from(2) << "]"
				<< ",\"goal\":[" << p.goal(0) << "," << p.goal(1) << "," << p.goal(2) << "]"
				<< ",\"path_length\":" << p.pathLength << ",\"legs\":" << p.legs << ",\"turn\":" << p.turn
				<< ",\"score\":" << p.score << ",\"gain\":" << p.gain << ",\"path_time\":" << p.pathTime
				<< ",\"candidates\":" << p.candidates << ",\"learned\":" << p.learned << ",\"revisits\":" << p.revisits << ",\"nodes\":" << p.nodes << ",\"reachable\":" << p.reachable << "}";
		}
		json << "]";
		std_msgs::String msg;
		msg.data = json.str();
		this->planLogPub_.publish(msg);
	}

	void dynamicExploration::logEvent(const std::string& text){
		cout << "\033[1;36m[Mission]: " << text << "\033[0m" << endl;
		double t = this->missionStartTime_.isZero() ? 0.0 : (ros::Time::now() - this->missionStartTime_).toSec();
		std::lock_guard<std::mutex> lock (this->eventsMutex_);
		this->events_.push_back(std::make_pair(t, text));
		while (this->events_.size() > 30) this->events_.pop_front();
	}

	std::string dynamicExploration::stateName(int state){
		switch (state){
			case IDLE: return "IDLE";
			case TAKEOFF: return "TAKEOFF";
			case NUDGE: return "ENTRY";
			case EXPLORE: return "EXPLORING";
			case PAUSED: return "PAUSED";
			case INSPECT: return "INSPECTING";
			case RETURN: return "RETURNING";
			case LAND: return "LANDING";
			case LANDED: return "LANDED";
		}
		return "UNKNOWN";
	}

	Eigen::Vector3d dynamicExploration::currPos(){
		return Eigen::Vector3d (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
	}

	double dynamicExploration::currYaw(){
		return AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
	}

	double dynamicExploration::computeExecutionDistance(){
		if (this->trajectoryReady_ and not this->replan_){
			Eigen::Vector3d prevP, currP;
			bool firstTime = true;
			double totalDistance = 0.0;
			double remainDistance = 0.0;
			for (double t=0.0; t<=this->trajectory_.getDuration(); t+=0.1){
				currP = this->trajectory_.at(t);
				if (firstTime){
					firstTime = false;
				}
				else{
					if (t <= this->trajTime_){
						totalDistance += (currP - prevP).norm();
					}
					else{
						remainDistance += (currP - prevP).norm();
					}
				}
				prevP = currP;
			}
			if (remainDistance <= 1.0){ // no replan when less than 1m
				return -1.0;
			}
			return totalDistance;
		}
		return -1.0;
	}

	bool dynamicExploration::replanForDynamicObstacle(){
		ros::Time currTime = ros::Time::now();
		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		if (this->useFakeDetector_){
			this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}
		else{ 
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}

		bool replan = false;
		bool hasDynamicObstacle = (obstaclesPos.size() != 0);
		if (hasDynamicObstacle){
			double timePassed = (currTime - this->lastDynamicObstacleTime_).toSec();
			if (timePassed >= this->replanTimeForDynamicObstacle_){
				replan = true;
				this->lastDynamicObstacleTime_ = currTime;
			}
		}
		return replan;
	}

	bool dynamicExploration::reachExplorationGoal(){
		if (this->waypoints_.poses.size() == 0) return false;
		double distThresh = 0.1;
		double yawDiffThresh = 0.1;
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d targetPos (this->waypoints_.poses.back().pose.position.x, this->waypoints_.poses.back().pose.position.y, this->waypoints_.poses.back().pose.position.z);
		double yawTarget = AutoFlight::rpy_from_quaternion(this->waypoints_.poses.back().pose.orientation); 
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		if ((currPos - targetPos).norm() <= distThresh and std::abs(currYaw - yawTarget) >= yawDiffThresh){
			return true;
		}
		return false;
	}

	bool dynamicExploration::isGoalValid(){
		Eigen::Vector3d pGoal (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
		if (this->map_->isInflatedOccupied(pGoal)){
			return false;
		}
		else{
			return true;
		}
	}

	nav_msgs::Path dynamicExploration::getCurrentTraj(double dt){
		nav_msgs::Path currentTraj;
		currentTraj.header.frame_id = "map";
		currentTraj.header.stamp = ros::Time::now();
	
		if (this->trajectoryReady_){
			// include the current pose
			// geometry_msgs::PoseStamped psCurr;
			// psCurr.pose = this->odom_.pose.pose;
			// currentTraj.poses.push_back(psCurr);
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=dt){
				Eigen::Vector3d pos = this->trajectory_.at(t);
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = pos(0);
				ps.pose.position.y = pos(1);
				ps.pose.position.z = pos(2);
				currentTraj.poses.push_back(ps);
			}		
		}
		return currentTraj;
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, pDiff) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(const Eigen::Vector3d& pos){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr = pos;
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, pDiff) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(const Eigen::Vector3d& pos, double yaw){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr = pos;
		Eigen::Vector3d direction (cos(yaw), sin(yaw), 0);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, direction) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	void dynamicExploration::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize){
		std::vector<onboardDetector::box3D> obstacles;
		this->detector_->getObstaclesInSensorRange(PI_const, obstacles);
		for (onboardDetector::box3D ob : obstacles){
			Eigen::Vector3d pos (ob.x, ob.y, ob.z);
			Eigen::Vector3d vel (ob.Vx, ob.Vy, 0.0);
			Eigen::Vector3d size (ob.x_width, ob.y_width, ob.z_width);
			obstaclesPos.push_back(pos);
			obstaclesVel.push_back(vel);
			obstaclesSize.push_back(size);
		}
	}


	void dynamicExploration::waitTime(double time){
		ros::Rate r (30);
		ros::Time startTime = ros::Time::now();
		ros::Time currTime = ros::Time::now();
		double passtime = 0.0;
		while (ros::ok() and passtime < time){
			currTime = ros::Time::now();
			passtime = (currTime - startTime).toSec();
			ros::spinOnce();
			r.sleep();
		}
	}
}
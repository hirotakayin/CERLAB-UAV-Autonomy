/*
*	File: dep.cpp
*	---------------
*   dynamic exploration planner implementation
*/

#include <global_planner/dep.h>
#include <random>


namespace globalPlanner{
	DEP::DEP(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "/DEP";
		this->hint_ = "[DEP]";
		this->initParam();
		this->initModules();
		this->registerPub();
		this->registerCallback();
	}

	void DEP::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
	}

	void DEP::loadVelocity(double vel, double angularVel){
		this->vel_ = vel;
		this->angularVel_ = angularVel;
	}

	void DEP::initParam(){
		// odom topic name
		if (not this->nh_.getParam(this->ns_ + "/odom_topic", this->odomTopic_)){
			this->odomTopic_ = "/CERLAB/quadcopter/odom";
			cout << this->hint_ << ": No odom topic name. Use default: /CERLAB/quadcopter/odom" << endl;
		}
		else{
			cout << this->hint_ << ": Odom topic: " << this->odomTopic_ << endl;
		}

		// local sample region min
		std::vector<double> localRegionMinTemp;	
		if (not this->nh_.getParam(this->ns_ + "/local_region_min", localRegionMinTemp)){
			this->localRegionMin_(0) = -5.0;
			this->localRegionMin_(1) = -5.0;
			this->localRegionMin_(2) = -2.0;
			cout << this->hint_ << ": No local region min param. Use default: [-5 -5 -2]" <<endl;
		}
		else{
			this->localRegionMin_(0) = localRegionMinTemp[0];
			this->localRegionMin_(1) = localRegionMinTemp[1];
			this->localRegionMin_(2) = localRegionMinTemp[2];
			cout << this->hint_ << ": Local Region Min: " << this->localRegionMin_[0] <<" " <<this->localRegionMin_[1]<<" "<< this->localRegionMin_[2]<< endl;
		}

		// local sample region max
		std::vector<double> localRegionMaxTemp;	
		if (not this->nh_.getParam(this->ns_ + "/local_region_max", localRegionMaxTemp)){
			this->localRegionMax_(0) = 5.0;
			this->localRegionMax_(1) = 5.0;
			this->localRegionMax_(2) = 2.0;
			cout << this->hint_ << ": No local region max param. Use default: [5 5 2]" <<endl;
		}
		else{
			this->localRegionMax_(0) = localRegionMaxTemp[0];
			this->localRegionMax_(1) = localRegionMaxTemp[1];
			this->localRegionMax_(2) = localRegionMaxTemp[2];
			cout << this->hint_ << ": Local Region Max: " << this->localRegionMax_[0] <<" " <<this->localRegionMax_[1]<<" "<< this->localRegionMax_[2]<< endl;
		}

		// global sample region min
		std::vector<double> globalRegionMinTemp;	
		if (not this->nh_.getParam(this->ns_ + "/global_region_min", globalRegionMinTemp)){
			this->globalRegionMin_(0) = -20.0;
			this->globalRegionMin_(1) = -20.0;
			this->globalRegionMin_(2) = 0.0;
			cout << this->hint_ << ": No global region min param. Use default: [-20 -20 0]" <<endl;
		}
		else{
			this->globalRegionMin_(0) = globalRegionMinTemp[0];
			this->globalRegionMin_(1) = globalRegionMinTemp[1];
			this->globalRegionMin_(2) = globalRegionMinTemp[2];
			cout << this->hint_ << ": Global Region Min: " << this->globalRegionMin_[0] <<" "<< this->globalRegionMin_[1]<<" "<< this->globalRegionMin_[2]<< endl;
		}

		// global sample region max
		std::vector<double> globalRegionMaxTemp;	
		if (not this->nh_.getParam(this->ns_ + "/global_region_max", globalRegionMaxTemp)){
			this->globalRegionMax_(0) = 20.0;
			this->globalRegionMax_(1) = 20.0;
			this->globalRegionMax_(2) = 3.0;
			cout << this->hint_ << ": No global region max param. Use default: [20 20 3]" <<endl;
		}
		else{
			this->globalRegionMax_(0) = globalRegionMaxTemp[0];
			this->globalRegionMax_(1) = globalRegionMaxTemp[1];
			this->globalRegionMax_(2) = globalRegionMaxTemp[2];
			cout << this->hint_ << ": Global Region Max: " << this->globalRegionMax_[0] <<" "<< this->globalRegionMax_[1]<<" "<< this->globalRegionMax_[2]<< endl;
		}

		// Local Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/local_sample_thresh", this->localSampleThresh_)){
			this->localSampleThresh_ = 50;
			cout << this->hint_ << ": No local sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Local sample Thresh: " << this->localSampleThresh_ << endl;
		}
		
		// Global Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/global_sample_thresh", this->globalSampleThresh_)){
			this->globalSampleThresh_ = 50;
			cout << this->hint_ << ": No global sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Global sample Thresh: " << this->globalSampleThresh_ << endl;
		}

		// Frontier Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/frontier_sample_thresh", this->frontierSampleThresh_)){
			this->frontierSampleThresh_ = 50;
			cout << this->hint_ << ": No frontier sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Frontier sample Thresh: " << this->frontierSampleThresh_ << endl;
		}		

		// minimum distance for node sampling
		if (not this->nh_.getParam(this->ns_ + "/dist_thresh", this->distThresh_)){
			this->distThresh_ = 0.8;
			cout << this->hint_ << ": No distance thresh param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Distance Thresh: " << this->distThresh_ << endl;
		}

		// safety distance for random sampling in xy
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_xy", this->safeDistXY_)){
			this->safeDistXY_ = 0.3;
			cout << this->hint_ << ": No safe distance in XY param. Use default: 0.3" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance in XY: " << this->safeDistXY_ << endl;
		}

		// safety distance for random sampling in z
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_z", this->safeDistZ_)){
			this->safeDistZ_ = 0.0;
			cout << this->hint_ << ": No safe distance in Z param. Use default: 0.0" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance in Z: " << this->safeDistZ_ << endl;
		}

		// safety distance check unknown
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_check_unknown", this->safeDistCheckUnknown_)){
			this->safeDistCheckUnknown_ = true;
			cout << this->hint_ << ": No safe distance check unknown param. Use default: true" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance check unknown: " << this->safeDistCheckUnknown_ << endl;
		}

		//Camera Parameters	
		if (not this->nh_.getParam(this->ns_ + "/horizontal_FOV", this->horizontalFOV_)){
			this->horizontalFOV_ = 0.8;
			cout << this->hint_ << ": No Horizontal FOV param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Horizontal FOV: " << this->horizontalFOV_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/vertical_FOV", this->verticalFOV_)){
			this->verticalFOV_ = 0.8;
			cout << this->hint_ << ": No Vertical FOV param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Vertical FOV: " << this->verticalFOV_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/dmin", this->dmin_)){
			this->dmin_ = 0.0;
			cout << this->hint_ << ": No min depth param. Use default: 0.0" << endl;
		}
		else{
			cout << this->hint_ << ": Min Depth: " << this->dmin_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/dmax", this->dmax_)){
			this->dmax_ = 1.0;
			cout << this->hint_ << ": No max depth param. Use default: 1.0" << endl;
		}
		else{
			cout << this->hint_ << ": Max Depth: " << this->dmax_ << endl;
		}

		// sensor model: camera (horizontal/vertical FOV cone) or lidar (360 deg, tilted elevation band)
		std::string sensorType;
		if (not this->nh_.getParam(this->ns_ + "/sensor_type", sensorType)){
			sensorType = "camera";
			cout << this->hint_ << ": No sensor type param. Use default: camera" << endl;
		}
		else{
			cout << this->hint_ << ": Sensor type: " << sensorType << endl;
		}
		this->useOmniModel_ = (sensorType == "omni360");
		this->useLidarModel_ = (sensorType == "lidar") or this->useOmniModel_; // omni360 reuses the full-sphere search

		if (this->useOmniModel_){
			double blindDeg, minAngleDeg;
			this->nh_.param<double>(this->ns_ + "/omni_blind_cone", blindDeg, 35.0);
			this->nh_.param<double>(this->ns_ + "/omni_flow_range_s", this->omniFlowRange_, 10.0);
			this->nh_.param<double>(this->ns_ + "/omni_speed", this->omniSpeed_, 1.0);
			this->nh_.param<double>(this->ns_ + "/omni_min_angle", minAngleDeg, 5.0);
			this->omniBlindSin_ = -cos(blindDeg * PI_const / 180.0); // lowest z of a unit direction that is still seen
			this->omniMinSin_ = sin(minAngleDeg * PI_const / 180.0);
			this->lidarFovUp_ = PI_const / 2; this->lidarFovDown_ = -PI_const / 2; this->lidarPitch_ = 0.0; // unused, kept sane
			cout << this->hint_ << ": 360 camera (optic flow): blind cone below " << blindDeg << " deg, range " << this->omniFlowRange_
			     << " s x " << this->omniSpeed_ << " m/s x sin(angle from travel), min angle " << minAngleDeg << " deg" << endl;
		}
		else if (this->useLidarModel_){
			double fovUpDeg, fovDownDeg, pitchDeg;
			if (not this->nh_.getParam(this->ns_ + "/lidar_fov_up", fovUpDeg)){
				fovUpDeg = 52.0;
				cout << this->hint_ << ": No lidar fov up param. Use default: 52.0 deg (Mid-360)" << endl;
			}
			if (not this->nh_.getParam(this->ns_ + "/lidar_fov_down", fovDownDeg)){
				fovDownDeg = -7.0;
				cout << this->hint_ << ": No lidar fov down param. Use default: -7.0 deg (Mid-360)" << endl;
			}
			if (not this->nh_.getParam(this->ns_ + "/lidar_pitch", pitchDeg)){
				pitchDeg = 0.0;
				cout << this->hint_ << ": No lidar pitch param. Use default: 0.0 deg" << endl;
			}
			this->lidarFovUp_ = fovUpDeg * PI_const / 180.0;
			this->lidarFovDown_ = fovDownDeg * PI_const / 180.0;
			this->lidarPitch_ = pitchDeg * PI_const / 180.0;
			cout << this->hint_ << ": Lidar elevation FOV: [" << fovDownDeg << ", " << fovUpDeg << "] deg, pitch (nose-down positive): " << pitchDeg << " deg" << endl;
		}

		// steepest roadmap edge allowed (edges must still be known free)
		double maxEdgeUpDeg, maxEdgeDownDeg;
		this->maxEdgeUpAngle_ = this->verticalFOV_/2;
		this->maxEdgeDownAngle_ = this->verticalFOV_/2;
		if (this->nh_.getParam(this->ns_ + "/max_edge_up_angle", maxEdgeUpDeg)) this->maxEdgeUpAngle_ = maxEdgeUpDeg * PI_const / 180.0;
		if (this->nh_.getParam(this->ns_ + "/max_edge_down_angle", maxEdgeDownDeg)) this->maxEdgeDownAngle_ = maxEdgeDownDeg * PI_const / 180.0;
		bool staticEnvironment;
		this->nh_.param<bool>(this->ns_ + "/static_environment", staticEnvironment, false);
		this->staticEnvironment_ = staticEnvironment;
		cout << this->hint_ << ": Max edge angle up " << this->maxEdgeUpAngle_ * 180.0 / PI_const << " deg, down " << this->maxEdgeDownAngle_ * 180.0 / PI_const << " deg, static environment: " << staticEnvironment << endl;

		// nearest neighbor number
		if (not this->nh_.getParam(this->ns_ + "/nearest_neighbor_number", this->nnNum_)){
			this->nnNum_ = 15;
			cout << this->hint_ << ": No nearest neighbor param. Use default: 15" << endl;
		}
		else{
			cout << this->hint_ << ": Nearest neighbor number is set to: " << this->nnNum_ << endl;
		}

		// frontier nearest neighbor number
		if (not this->nh_.getParam(this->ns_ + "/frontier_nearest_neighbor_number", this->nnNumFrontier_)){
			this->nnNumFrontier_ = 15;
			cout << this->hint_ << ": No frontier nearest neighbor param. Use default: 15" << endl;
		}
		else{
			cout << this->hint_ << ": Frontier nearest neighbor number is set to: " << this->nnNumFrontier_ << endl;
		}

		// node connection max distances
		if (not this->nh_.getParam(this->ns_ + "/max_connect_dist", this->maxConnectDist_)){
			this->maxConnectDist_ = 1.5;
			cout << this->hint_ << ": No max conect distance param. Use default: 1.5m." << endl;
		}
		else{
			cout << this->hint_ << ": Max connect distance is set to: " << this->maxConnectDist_ << endl;
		}

		// number of yaw angles
		int yawNum = 32;
		if (not this->nh_.getParam(this->ns_ + "/num_yaw_angles", yawNum)){
			for (int i=0; i<32; ++i){
				this->yaws_.push_back(i*2*PI_const/32);
			}					
			cout << this->hint_ << ": No number of yaw angles param. Use default: 32." << endl;
		}
		else{
			for (int i=0; i<yawNum; ++i){
				this->yaws_.push_back(i*2*PI_const/yawNum);
			}
			cout << this->hint_ << ": Number of yaw angles is set to: " << yawNum << endl;
		}

		// lidar orientation for each candidate yaw: world_R_lidar = Rz(yaw) * Ry(pitch)
		if (this->useLidarModel_){
			for (double yaw : this->yaws_){
				Eigen::Matrix3d worldRLidar = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(this->lidarPitch_, Eigen::Vector3d::UnitY())).toRotationMatrix();
				this->yawLidarRotInv_.push_back(worldRLidar.transpose());
			}
		}

		// minimum threshold of voxels
		if (not this->nh_.getParam(this->ns_ + "/min_voxel_thresh", this->minVoxelThresh_)){
			this->minVoxelThresh_ = 0.1;
			cout << this->hint_ << ": No minimum threshold of voxels param. Use default: 0.1." << endl;
		}
		else{
			cout << this->hint_ << ": Minimum threshold of voxels is set to: " << this->minVoxelThresh_ << endl;
		}

		// minimum number of goal candidates
		if (not this->nh_.getParam(this->ns_ + "/min_goal_candidates", this->minCandidateNum_)){
			this->minCandidateNum_ = 10;
			cout << this->hint_ << ": No minimum number of goal candidates param. Use default: 10." << endl;
		}
		else{
			cout << this->hint_ << ": Minimum number of goal candidates is set to: " << this->minCandidateNum_ << endl;
		}

		// maximum number of goal candidates
		if (not this->nh_.getParam(this->ns_ + "/max_goal_candidates", this->maxCandidateNum_)){
			this->maxCandidateNum_ = 30;
			cout << this->hint_ << ": No maximum number of goal candidates param. Use default: 30." << endl;
		}
		else{
			cout << this->hint_ << ": Maximum number of goal candidates is set to: " << this->maxCandidateNum_ << endl;
		}

		// Information gain update  distance
		if (not this->nh_.getParam(this->ns_ + "/information_gain_update_distance", this->updateDist_)){
			this->updateDist_ = 1.0;
			cout << this->hint_ << ": No information gain update distance param. Use default: 1.0m." << endl;
		}
		else{
			cout << this->hint_ << ": Information gain update distance is set to: " << this->updateDist_ << endl;
		}

		// yaw penalty weight
		if (not this->nh_.getParam(this->ns_ + "/yaw_penalty_weight", this->yawPenaltyWeight_)){
			this->yawPenaltyWeight_ = 1.0;
			cout << this->hint_ << ": No yaw penalty weight param. Use default: 1.0." << endl;
		}
		else{
			cout << this->hint_ << ": Yaw penalty weight is set to: " << this->yawPenaltyWeight_ << endl;
		}

		// gain region: leave out volume the sensor can never observe (inside walls, under the ground),
		// its unknown voxels never clear and would attract the planner forever
		std::vector<double> gainRegionTemp;
		this->gainRegionMin_ = this->globalRegionMin_;
		this->gainRegionMax_ = this->globalRegionMax_;
		if (this->nh_.getParam(this->ns_ + "/gain_region_min", gainRegionTemp) and gainRegionTemp.size() == 3){
			this->gainRegionMin_ = Eigen::Vector3d (gainRegionTemp[0], gainRegionTemp[1], gainRegionTemp[2]);
		}
		if (this->nh_.getParam(this->ns_ + "/gain_region_max", gainRegionTemp) and gainRegionTemp.size() == 3){
			this->gainRegionMax_ = Eigen::Vector3d (gainRegionTemp[0], gainRegionTemp[1], gainRegionTemp[2]);
		}
		cout << this->hint_ << ": Gain region: [" << this->gainRegionMin_.transpose() << "] to [" << this->gainRegionMax_.transpose() << "]" << endl;

		// minimum goal distance
		this->nh_.param<double>(this->ns_ + "/min_goal_distance", this->minGoalDist_, 0.0);
		cout << this->hint_ << ": Minimum goal distance: " << this->minGoalDist_ << "m." << endl;

		// depth bias and revisit penalty
		this->nh_.param<double>(this->ns_ + "/max_unknown_gap", this->maxUnknownGap_, 0.0);
		cout << this->hint_ << ": Max unknown gap on edges: " << this->maxUnknownGap_ << "m." << endl;
		this->nh_.param<double>(this->ns_ + "/depth_bias", this->depthBias_, 0.0);
		this->nh_.param<double>(this->ns_ + "/revisit_penalty", this->revisitPenalty_, 0.0);
		this->nh_.param<double>(this->ns_ + "/gain_block_size", this->gainBlockSize_, 0.0);
		cout << this->hint_ << ": Gain block size: " << this->gainBlockSize_ << "m." << endl;
		bool tourEnabled = false;
		this->nh_.param<bool>(this->ns_ + "/tour_enabled", tourEnabled, false);
		this->nh_.param<double>(this->ns_ + "/tour_bin_size", this->tourBinSize_, 3.0);
		this->nh_.param<int>(this->ns_ + "/tour_min_cells", this->tourMinCells_, 3);
		this->nh_.param<double>(this->ns_ + "/tour_min_share", this->tourMinShare_, 0.25);
		this->nh_.param<int>(this->ns_ + "/tour_max_visits", this->tourMaxVisits_, 3);
		this->tourEnabled_ = tourEnabled and this->gainBlockSize_ > 0.0;
		cout << this->hint_ << ": Tour: " << (this->tourEnabled_ ? "on" : "off") << endl;
		cout << this->hint_ << ": Depth bias: " << this->depthBias_ << ", revisit penalty: " << this->revisitPenalty_ << endl;
	}

	void DEP::initModules(){
		// initialize roadmap
		this->roadmap_.reset(new PRM::KDTree ());
	}

	void DEP::registerPub(){
		// roadmap visualization publisher
		this->roadmapPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/roadmap", 10);

		// candidate paths publisher
		this->candidatePathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/candidate_paths", 10);

		// best path publisher
		this->bestPathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/best_paths", 10);

		// fronteir vis publisher
		this->frontierVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/frontier_regions", 10);

		// tour order publisher
		this->tourPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/tour", 10);
	}

	void DEP::registerCallback(){
		// odom subscriber
		this->odomSub_ = this->nh_.subscribe(this->odomTopic_, 1000, &DEP::odomCB, this);
	
		// visualization timer
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &DEP::visCB, this);

	}

	bool DEP::makePlan(){
		if (not this->odomReceived_) return false;
		// cout << "start detecting frontier" << endl;
		// ros::Time frontierStartTime = ros::Time::now();
		this->detectFrontierRegion(this->frontierPointPairs_);
		// ros::Time frontierEndTime = ros::Time::now();
		// cout << "frontier detection time: " << (frontierEndTime - frontierStartTime).toSec() << endl;


		// cout << "start building roadmap" << endl;
		// ros::Time buildStartTime = ros::Time::now();
		this->buildRoadMap();
		// ros::Time buildEndTime = ros::Time::now();
		// cout << "build roadmap time: " << (buildEndTime - buildStartTime).toSec() << endl;

		// cout << "start pruning nodes" << endl;
		// ros::Time updateStartTime = ros::Time::now();
		this->pruneNodes();

		// cout << "start update information gain" << endl;
		this->updateGainCells();
		this->updateInformationGain();
		this->tourPlanActive_ = this->tourEnabled_ and this->useBlockGain();
		if (this->tourPlanActive_) return this->planTour();
		this->tourDone_ = false;
		this->tourStops_.clear();
		// ros::Time updateEndTime = ros::Time::now();
		// cout << "update time: " << (updateEndTime - updateStartTime).toSec() << endl;

		// cout << "start get goal candidates" << endl;
		// ros::Time pathStartTime = ros::Time::now();
		this->getBestViewCandidates(this->goalCandidates_);

		// cout << "finish best view candidate with size: " << this->goalCandidates_.size() << endl;

		bool findCandidatePathSuccess = this->findCandidatePath(this->goalCandidates_, this->candidatePaths_);

		// cout << "finish candidate path with size: " << this->candidatePaths_.size() << endl;
		if (not findCandidatePathSuccess){
			// cout << "Find candidate paths fails. need generate more samples." << endl;
			return false;
		}

		this->findBestPath(this->candidatePaths_, this->bestPath_);
		// ros::Time pathEndTime = ros::Time::now();
		// cout << "path time: " << (pathEndTime - pathStartTime).toSec() << endl;
		// cout << "found best path with size: " << this->bestPath_.size() << endl;
		return true;
	}

	nav_msgs::Path DEP::getBestPath(){
		nav_msgs::Path bestPath;
		for (int i=0; i<int(this->bestPath_.size()); ++i){
			std::shared_ptr<PRM::Node> currNode = this->bestPath_[i];
			geometry_msgs::PoseStamped p;
			p.pose.position.x = currNode->pos(0);
			p.pose.position.y = currNode->pos(1);
			p.pose.position.z = currNode->pos(2);
			if (i < int(this->bestPath_.size())-1){
				std::shared_ptr<PRM::Node> nextNode = this->bestPath_[i+1];
				Eigen::Vector3d diff = nextNode->pos - currNode->pos;
				double angle = atan2(diff(1), diff(0));
				p.pose.orientation = globalPlanner::quaternion_from_rpy(0, 0, angle);
			}
			bestPath.poses.push_back(p);
		}
		
		// get the best yaw for the last pose
		double bestYaw = this->tourPlanActive_ ? this->tourGoalYaw_ : this->bestPath_.back()->getBestYaw();
		bestPath.poses.back().pose.orientation = globalPlanner::quaternion_from_rpy(0, 0, bestYaw);

		return bestPath;
	}

	bool DEP::edgeAllowed(const Eigen::Vector3d& from, const Eigen::Vector3d& to){
		// edges are stored whatever their slope and checked when searched, as the limit differs by direction and can be toggled
		Eigen::Vector3d direction = to - from;
		double length = direction.norm();
		if (length < 1e-6) return true;
		double angle = asin(std::abs(direction(2)) / length);
		if (angle <= (direction(2) > 0.0 ? this->maxEdgeUpAngle_ : this->maxEdgeDownAngle_)) return true;
		// steeper than the lidar can see ahead: only with nothing moving and nothing unknown on the way
		return this->staticEnvironment_ and this->map_->isInflatedFreeLine(from, to);
	}

	void DEP::setStaticEnvironment(bool enabled){
		this->staticEnvironment_ = enabled;
	}

	bool DEP::isStaticEnvironment(){
		return this->staticEnvironment_;
	}

	// create sensor check for 
	// vert, horz FOV, collision, and sensor distance range
	// for yaw angles in vector3d:  cos(yaw), sin(yaw), 0
	// horz angle between yaw angle vector and direction (x y 0) vector for horz FOV
	// Vert angle yaw angle vector and yaw angle vector (c s z) z is direction.z()
	bool DEP::sensorFOVCondition(const Eigen::Vector3d& sample, const Eigen::Vector3d& pos){
		Eigen::Vector3d direction = sample - pos;
		double distance = direction.norm();
		if (distance > this->dmax_){
			return false;
		}
		bool hasCollision = this->map_->isInflatedOccupiedLine(sample, pos);
		if (hasCollision == true){
			return false;
		}
		return true;
	}

	// lidar check: rotate the direction into the lidar frame at this yaw and test its elevation
	// against the lidar's vertical FOV (full 360 deg in azimuth)
	bool DEP::lidarVisible(const Eigen::Vector3d& direction, int yawIdx){
		if (this->useOmniModel_) return this->omniVisible(direction, this->yaws_[yawIdx]);
		Eigen::Vector3d dirLidar = this->yawLidarRotInv_[yawIdx] * direction;
		double elevation = atan2(dirLidar.z(), dirLidar.head<2>().norm());
		return elevation >= this->lidarFovDown_ and elevation <= this->lidarFovUp_;
	}



	bool isNodeRequireUpdate(std::shared_ptr<PRM::Node> n, std::vector<std::shared_ptr<PRM::Node>> path, double& leastDistance){
		double distanceThresh = 2;
		leastDistance = std::numeric_limits<double>::max();
		for (std::shared_ptr<PRM::Node>& waypoint: path){
			double currentDistance = (n->pos - waypoint->pos).norm();
			if (currentDistance < leastDistance){
				leastDistance = currentDistance;
			}
		}
		if (leastDistance <= distanceThresh){
			return true;
		}
		else{
			return false;	
		}
		
	}

	void DEP::detectFrontierRegion(std::vector<std::pair<Eigen::Vector3d, double>>& frontierPointPairs){
		frontierPointPairs.clear();

		Eigen::Vector3d mapMin, mapMax;
		this->map_->getCurrMapRange(mapMin, mapMax);
		int numRow = (mapMax(1) - mapMin(1))/this->map_->getRes() + 1;
		int numCol = (mapMax(0) - mapMin(0))/this->map_->getRes() + 1;

		cv::SimpleBlobDetector::Params params;
		// params.filterByColor = true;
		// params.blobColor = 255;
		// params.filterByConvexity = true;
  //   	params.minConvexity = 0.1;;

		params.filterByColor = true;
		params.blobColor = 255;  // Blobs should be white
		params.filterByArea = true;
		params.minArea = pow(1/this->map_->getRes(), 2);
		params.maxArea = numRow * numCol;
		params.filterByCircularity = false;
		params.minCircularity = 1;
		params.filterByConvexity = true;
		params.minConvexity = 0.1;

		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		std::vector<cv::Mat> imgVec;
		// find height levels to slice the map
		double heightRes = 0.3;
		int col = 0;
		int row = 0;
		for (double h=this->globalRegionMin_(2); h<=this->globalRegionMax_(2); h+=heightRes){
			row = 0;
			cv::Mat im (numRow, numCol, CV_8UC1);
			for (double y=mapMin(1); y<=mapMax(1) and row < numRow; y+=this->map_->getRes()){
				col = 0;
				for (double x=mapMin(0); x<=mapMax(0) and col < numCol; x+=this->map_->getRes()){
					Eigen::Vector3d p (x, y, h);
					if (this->map_->isInflatedOccupied(p)){
						im.at<uchar>(row, col) = 0;
					}
					else if (this->map_->isInflatedFree(p)){
						// im.at<uchar>(row, col) = 255/2;
						// im.at<uchar>(row, col) = 255;
						im.at<uchar>(row, col) = 0;
					}
					else{
						// im.at<uchar>(row, col) = 255/2;
						im.at<uchar>(row, col) = 255;
					}
					++col;
				}
				++row;
			}
			imgVec.push_back(im);
		}



		// detect each image and find the corresponding 3D positions
		double height = this->globalRegionMin_(2);
		for (cv::Mat img : imgVec){
			std::vector<cv::KeyPoint> keypoints;

			cv::Rect rect(0, 0, numRow, numCol);
			cv::rectangle(img, rect, cv::Scalar(0, 0, 0), 3);
			detector->detect(img, keypoints);

			// convert keypoints back to the map coordinate
			for (cv::KeyPoint keypoint : keypoints){
				Eigen::Vector3d p (mapMin(0) + keypoint.pt.x * this->map_->getRes(), mapMin(1) + keypoint.pt.y * this->map_->getRes(), height);
				double dist = keypoint.size * this->map_->getRes();
				if ((p.array() < this->gainRegionMin_.array()).any() or (p.array() > this->gainRegionMax_.array()).any()) continue;
				frontierPointPairs.push_back({p, dist});
			}

			height += heightRes;

			// cv::Mat imgWithKeypoints;
			// cv::drawKeypoints(img, keypoints, imgWithKeypoints,  cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

			// cv::imshow("Blobs with Keypoints", imgWithKeypoints);
			// cv::imwrite("/home/zhefan/Desktop/temp/test.jpg", img);
			// cv::waitKey(0);
		} 
	}

	void DEP::buildRoadMap(){
		bool saturate = false;
		bool regionSaturate = false;
		int countSample = 0;
		std::shared_ptr<PRM::Node> n;
		std::vector<std::shared_ptr<PRM::Node>> newNodes;

		// while does reach sampling threshold (fail time) 
		// 1. sample point from frontier region by weighted sampling
		// 2. find several nearest node
		// 3. for each node, extend random [mindist, maxdist] distance. if success, add new sample 
		std::vector<double> sampleWeights;
		for (int i=0; i<int(this->frontierPointPairs_.size()); ++i){
			double size = this->frontierPointPairs_[i].second;
			sampleWeights.push_back(pow(size, 2));
		}
		int countFrontierFailure = 0;
		while (ros::ok() and countFrontierFailure < this->frontierSampleThresh_ and sampleWeights.size() != 0){
			std::shared_ptr<PRM::Node> fn = this->sampleFrontierPoint(sampleWeights);
			// a. find N nearest neighbors
			std::vector<std::shared_ptr<PRM::Node>> fnNeighbors = this->roadmap_->kNearestNeighbor(fn, this->nnNumFrontier_);

			// b. for each neighbor extend them and check the validity
			if (int(fnNeighbors.size()) > 0){
				int countSampleOnce = 0;
				for (std::shared_ptr<PRM::Node> fnNN : fnNeighbors){
					n = this->extendNode(fnNN, fn);
					if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
						std::shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
						double distToNN = (n->pos - nn->pos).norm();
						if (distToNN >= this->distThresh_){
							this->roadmap_->insert(n);
							newNodes.push_back(n);
							// this->prmNodeVec_.push_back(n);
							this->prmNodeVec_.insert(n);
							++countSample;	
							++countSampleOnce;
						}
					}
				}
				if (countSampleOnce == 0){
					++countFrontierFailure;
				}

			}
			else{ // not enough neighbor
				break;
			}
		}
		// cout << "added sample from frontier:  " << countSample << endl;

		while (ros::ok() and not saturate){
			if (regionSaturate){
				int countFailureGlobal = 0;
				// Generate new node
				while (ros::ok()){
					if (countFailureGlobal > this->globalSampleThresh_){
						saturate = true;
						break;
					}
					n = this->randomConfigBBox(this->globalRegionMin_, this->globalRegionMax_);
					// Check how close new node is other nodes
					double distToNN;
					if (this->roadmap_->getSize() != 0){
						shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
						distToNN = (n->pos - nn->pos).norm();
					}
					else{
						distToNN = this->distThresh_;
					}
					if (distToNN < this->distThresh_){
						++countFailureGlobal;
					}
					else{
						this->roadmap_->insert(n);
						newNodes.push_back(n);
						// this->prmNodeVec_.push_back(n);
						this->prmNodeVec_.insert(n);
						++countSample;
					}
				}
			}
			else{
				if (true){
					int countFailureLocal = 0;
					// Generate new node
					while (ros::ok() and true){
						//cout << "failure number: " << countFailureLocal << endl;
						if (countFailureLocal > this->localSampleThresh_){
							regionSaturate = true;
							break;
						}
						Eigen::Vector3d localSampleMin = this->position_+this->localRegionMin_;
						Eigen::Vector3d localSampleMax = this->position_+this->localRegionMax_;
						n = this->randomConfigBBox(localSampleMin, localSampleMax);
						// Check how close new node is other nodes
						double distToNN;

						if (this->roadmap_->getSize() != 0){
							std::shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
							distToNN = (n->pos - nn->pos).norm();
						}
						else{
							distToNN = this->distThresh_;
						}
						if (distToNN < this->distThresh_){
							++countFailureLocal;
						}
						else{
							this->roadmap_->insert(n);
							newNodes.push_back(n);
							// this->prmNodeVec_.push_back(n);
							this->prmNodeVec_.insert(n);
							++countSample;
						}
					}
				}
			}
		}


		
		// node connection
		for (std::shared_ptr<PRM::Node>& n : newNodes){
			std::vector<std::shared_ptr<PRM::Node>> knn = this->roadmap_->kNearestNeighbor(n, this->nnNum_);
			for (std::shared_ptr<PRM::Node>& nearestNeighborNode : knn){ // Check collision last if all other conditions are satisfied
				double distance2knn = (n->pos - nearestNeighborNode->pos).norm();
				if (distance2knn < this->maxConnectDist_){
					bool hasCollision = not PRM::isClearLine(this->map_, n->pos, nearestNeighborNode->pos, this->maxUnknownGap_);
					if (hasCollision == false){
						n->adjNodes.insert(nearestNeighborNode);
						nearestNeighborNode->adjNodes.insert(n);
					}
				}
			}
			n->newNode = true;
		}

		// edges are only made through known-free space, so retry old nodes as the map fills in;
		// otherwise nodes created before their surroundings were observed stay disconnected forever
		for (const std::shared_ptr<PRM::Node>& n : this->prmNodeVec_){
			std::vector<std::shared_ptr<PRM::Node>> knn = this->roadmap_->kNearestNeighbor(n, this->nnNum_);
			for (std::shared_ptr<PRM::Node>& nearestNeighborNode : knn){
				if (nearestNeighborNode == n or n->adjNodes.count(nearestNeighborNode) != 0) continue;
				double distance2knn = (n->pos - nearestNeighborNode->pos).norm();
				if (distance2knn < this->maxConnectDist_ and PRM::isClearLine(this->map_, n->pos, nearestNeighborNode->pos, this->maxUnknownGap_)){
					n->adjNodes.insert(nearestNeighborNode);
					nearestNeighborNode->adjNodes.insert(n);
				}
			}
		}
	}	 

	void DEP::pruneNodes(){
		// record the invalid nodes
		std::unordered_set<std::shared_ptr<PRM::Node>> invalidSet;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){ // new nodes
			if (not this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){// 1. new nodes
			// if (this->map_->isInflatedOccupied(n->pos)){// 1. new nodes
				invalidSet.insert(n);
			}	
		}

		// remove invalid nodes
		for (std::shared_ptr<PRM::Node> in : invalidSet){
			this->prmNodeVec_.erase(in);
			this->roadmap_->remove(in);
		}


		//  remove invalid edges
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			std::vector<std::shared_ptr<PRM::Node>> eraseVec;
			for (std::shared_ptr<PRM::Node> neighbor : n->adjNodes){
				if (invalidSet.find(neighbor) != invalidSet.end()){
					eraseVec.push_back(neighbor);
				}
			}

			for (std::shared_ptr<PRM::Node> en : eraseVec){
				n->adjNodes.erase(en);
			}
		}
	}

	void DEP::updateInformationGain(){
		// iterate through all current nodes (ignore update by path now)
		// two types of nodes need update:
		// 1. new nodes
		// 2. nodes close to the historical trajectory
		std::unordered_set<std::shared_ptr<PRM::Node>> updateSet;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){ // new nodes
			if (n->newNode == true or this->useBlockGain()){// 1. new nodes. Block gain is cheap enough to keep every node current
				updateSet.insert(n);
			}	
		}

		for (Eigen::Vector3d& histPos : this->histTraj_){ // traj update nodes
			std::shared_ptr<PRM::Node> histN;
			histN.reset(new PRM::Node(histPos));
			std::vector<std::shared_ptr<PRM::Node>> nns = this->roadmap_->kNearestNeighbor(histN, 10);
			for (std::shared_ptr<PRM::Node>& nn : nns){
				if ((nn->pos - histN->pos).norm() <= this->updateDist_){
					updateSet.insert(nn);
				}
			}
		}

		for (std::shared_ptr<PRM::Node> updateN : updateSet){ // update information gain
			std::unordered_map<double, int> yawNumVoxels;
			int unknownVoxelNum = this->calculateUnknown(updateN, yawNumVoxels);
			updateN->numVoxels = unknownVoxelNum;
			updateN->yawNumVoxels = yawNumVoxels;
			updateN->newNode = false;
		}
		this->histTraj_.clear(); // clear history
	}

	void DEP::getBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates){
		goalCandidates.clear();
		bool firstNode = true;
		std::priority_queue<std::shared_ptr<PRM::Node>, std::vector<std::shared_ptr<PRM::Node>>, PRM::GainCompareNode> gainPQ;

		if (this->prmNodeVec_.empty()) return;

		// only nodes connected to the robot can be goals: otherwise the best-gain nodes can all sit in
		// cut-off parts of the roadmap and every plan fails while reachable frontiers remain
		std::unordered_set<std::shared_ptr<PRM::Node>> reachable;
		std::vector<std::shared_ptr<PRM::Node>> searchStack {this->getConnectedNode(this->position_)};
		reachable.insert(searchStack[0]);
		while (not searchStack.empty()){
			std::shared_ptr<PRM::Node> n = searchStack.back();
			searchStack.pop_back();
			for (const std::shared_ptr<PRM::Node>& neighbor : n->adjNodes){
				if (this->edgeAllowed(n->pos, neighbor->pos) and reachable.insert(neighbor).second) searchStack.push_back(neighbor);
			}
		}
		// gain per time always favours the nearest node with any gain, which makes many tiny hops
		this->reachableNodeNum_ = int(reachable.size());
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			if (reachable.count(n) != 0 and n->numVoxels > 0 and (n->pos - this->position_).norm() >= this->minGoalDist_){
				gainPQ.push(n);
			}
		}
		if (gainPQ.empty()){
			for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
				if (reachable.count(n) != 0 and n->numVoxels > 0){ // no gain: nothing to learn there
					gainPQ.push(n);
				}
			}
		}

		// select candidates from the priority queue
		int maxNumVoxel = 0;
		while (ros::ok()){
			if (gainPQ.size() == 0){
				break;
			}


			std::shared_ptr<PRM::Node> n = gainPQ.top();
			
			if (firstNode){
				// if ((n->pos - this->position_).norm() >= 1.0){
				if ((n->pos - this->position_).norm() >= 0.0){
					maxNumVoxel = n->numVoxels;
					firstNode = false;
				}
			}

			if (double(n->numVoxels) < double(maxNumVoxel) * this->minVoxelThresh_){
				break;
			}
			// if ((n->pos - this->position_).norm() >= 1.0){
			if ((n->pos - this->position_).norm() >= 0.0){			
				if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
					goalCandidates.push_back(n);
					// cout << "Valid goal candidate: " << n->pos.transpose() << " voxel: " << n->numVoxels  << endl;
				}
			}
			gainPQ.pop();
			
			if (int(goalCandidates.size()) >= this->maxCandidateNum_){
				break;
			}
		}

		// cout << "current pos: " << this->position_.transpose() << endl;
		while (int(goalCandidates.size()) < this->minCandidateNum_){
			if (gainPQ.size() == 0){
				break;
			}

			if (int(goalCandidates.size()) >= this->maxCandidateNum_){
				break;
			}

			std::shared_ptr<PRM::Node> n = gainPQ.top();
			gainPQ.pop();
			// if ((n->pos - this->position_).norm() >= 1.0){ 
			if ((n->pos - this->position_).norm() >= 0.0){	
				// cout << "candidate goal: " << n->pos.transpose() << endl;	
				if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
					goalCandidates.push_back(n);
					// cout << "Valid goal candidate: " << n->pos.transpose() << " voxel: " << n->numVoxels  << endl;
				}			
			}
		}

		// candidates above are ranked by raw gain, which favours high nodes: always offer the deepest ones too
		if (this->depthBias_ > 0.0){
			std::vector<std::shared_ptr<PRM::Node>> deepNodes;
			for (const std::shared_ptr<PRM::Node>& n : reachable){
				if (n->numVoxels > 0 and (n->pos - this->position_).norm() >= this->minGoalDist_ and this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_) and
					std::find(goalCandidates.begin(), goalCandidates.end(), n) == goalCandidates.end()){
					deepNodes.push_back(n);
				}
			}
			std::sort(deepNodes.begin(), deepNodes.end(), [](const std::shared_ptr<PRM::Node>& a, const std::shared_ptr<PRM::Node>& b){
				return a->pos(2) < b->pos(2);
			});
			for (size_t i=0; i<deepNodes.size() and i<4; ++i) goalCandidates.push_back(deepNodes[i]);
		}
	}

	bool DEP::findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates, std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths){
		bool findPath = false;
		// find nearest node of current location
		std::shared_ptr<PRM::Node> currPos;
		currPos.reset(new PRM::Node (this->position_));
		std::shared_ptr<PRM::Node> start = this->getConnectedNode(this->position_);

		candidatePaths.clear();
		for (std::shared_ptr<PRM::Node> goal : goalCandidates){
			std::vector<std::shared_ptr<PRM::Node>> path = PRM::AStar(this->roadmap_, start, goal, this->map_, this->maxUnknownGap_, [this](const Eigen::Vector3d& from, const Eigen::Vector3d& to){return this->edgeAllowed(from, to);});
			if (int(path.size()) != 0){
				findPath = true;
			}
			else{
				continue;
			}
			path.insert(path.begin(), currPos);
			std::vector<std::shared_ptr<PRM::Node>> pathSc;
			this->shortcutPath(path, pathSc);
			candidatePaths.push_back(pathSc);
		}
		return findPath;
	}

	std::shared_ptr<PRM::Node> DEP::getConnectedNode(const Eigen::Vector3d& pos){
		// the nearest node can sit in unobserved space with no edges (e.g. the lidar's blind cone under the robot),
		// so prefer the closest nearby node that is connected and reachable in a straight known-free line
		std::shared_ptr<PRM::Node> query;
		query.reset(new PRM::Node (pos));
		std::shared_ptr<PRM::Node> nearest = this->roadmap_->nearestNeighbor(query);
		std::vector<std::shared_ptr<PRM::Node>> candidates = this->roadmap_->kNearestNeighbor(query, 15);
		std::sort(candidates.begin(), candidates.end(), [&pos](const std::shared_ptr<PRM::Node>& a, const std::shared_ptr<PRM::Node>& b){
			return (a->pos - pos).norm() < (b->pos - pos).norm();
		});
		for (std::shared_ptr<PRM::Node>& n : candidates){
			if (n->adjNodes.size() != 0 and this->edgeAllowed(pos, n->pos) and PRM::isClearLine(this->map_, pos, n->pos, this->maxUnknownGap_)){
				return n;
			}
		}
		return nearest;
	}

	bool DEP::findPathToPoint(const Eigen::Vector3d& goal, nav_msgs::Path& path){
		// roadmap path from the current position to an arbitrary point (return home, inspection viewpoint)
		path.poses.clear();
		if (not this->odomReceived_ or this->roadmap_->getSize() == 0) return false;

		std::vector<std::shared_ptr<PRM::Node>> nodePath;
		std::shared_ptr<PRM::Node> currNode;
		currNode.reset(new PRM::Node (this->position_));
		std::shared_ptr<PRM::Node> goalNode;
		goalNode.reset(new PRM::Node (goal));

		if (this->edgeAllowed(this->position_, goal) and PRM::isClearLine(this->map_, this->position_, goal, this->maxUnknownGap_)){
			nodePath = {currNode, goalNode}; // straight shot
		}
		else{
			std::shared_ptr<PRM::Node> start = this->getConnectedNode(this->position_);
			std::shared_ptr<PRM::Node> end = this->getConnectedNode(goal);
			if (not this->edgeAllowed(end->pos, goal) or not PRM::isClearLine(this->map_, goal, end->pos, this->maxUnknownGap_)) return false; // goal not reachable from the roadmap
			std::vector<std::shared_ptr<PRM::Node>> roadmapPath = PRM::AStar(this->roadmap_, start, end, this->map_, this->maxUnknownGap_, [this](const Eigen::Vector3d& from, const Eigen::Vector3d& to){return this->edgeAllowed(from, to);});
			if (roadmapPath.size() == 0) return false;
			nodePath.push_back(currNode);
			nodePath.insert(nodePath.end(), roadmapPath.begin(), roadmapPath.end());
			nodePath.push_back(goalNode);
		}

		std::vector<std::shared_ptr<PRM::Node>> nodePathSc;
		this->shortcutPath(nodePath, nodePathSc);
		for (size_t i=0; i<nodePathSc.size(); ++i){
			geometry_msgs::PoseStamped p;
			p.header.frame_id = "map";
			p.pose.position.x = nodePathSc[i]->pos(0);
			p.pose.position.y = nodePathSc[i]->pos(1);
			p.pose.position.z = nodePathSc[i]->pos(2);
			double yaw = 0.0;
			if (i+1 < nodePathSc.size()){
				Eigen::Vector3d diff = nodePathSc[i+1]->pos - nodePathSc[i]->pos;
				yaw = atan2(diff(1), diff(0));
			}
			else if (i > 0){
				Eigen::Vector3d diff = nodePathSc[i]->pos - nodePathSc[i-1]->pos;
				yaw = atan2(diff(1), diff(0));
			}
			p.pose.orientation = globalPlanner::quaternion_from_rpy(0, 0, yaw);
			path.poses.push_back(p);
		}
		return path.poses.size() >= 2;
	}

	void DEP::getBestPathScore(double& score, double& gain, double& pathTime, int& candidateNum){
		score = this->bestScore_;
		gain = this->bestGain_;
		pathTime = this->bestPathTime_;
		candidateNum = this->candidatePathNum_;
	}

	void DEP::getRoadmapStats(int& nodeNum, int& reachableNum){
		nodeNum = int(this->prmNodeVec_.size());
		reachableNum = this->reachableNodeNum_;
	}

	// 360 camera ranging by optic flow while travelling along heading yaw: the drone blocks a cone below it, and flow
	// gives distance only to D <= flowRange * speed * sin(angle from the travel direction) (none straight ahead/behind)
	bool DEP::omniVisible(const Eigen::Vector3d& direction, double yaw){
		double distance = direction.norm();
		if (distance < 1e-6) return false;
		Eigen::Vector3d unit = direction / distance;
		if (unit.z() < this->omniBlindSin_) return false;
		double cosToTravel = unit.x() * cos(yaw) + unit.y() * sin(yaw);
		double sinToTravel = sqrt(std::max(0.0, 1.0 - cosToTravel * cosToTravel));
		if (sinToTravel < this->omniMinSin_) return false;
		return distance <= this->omniFlowRange_ * this->omniSpeed_ * sinToTravel;
	}

	bool DEP::isInLidarFov(const Eigen::Vector3d& direction, double yaw){
		if (not this->useLidarModel_) return false;
		if (this->useOmniModel_) return this->omniVisible(direction, yaw);
		Eigen::Matrix3d worldRLidar = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(this->lidarPitch_, Eigen::Vector3d::UnitY())).toRotationMatrix();
		Eigen::Vector3d dirLidar = worldRLidar.transpose() * direction;
		double elevation = atan2(dirLidar.z(), dirLidar.head<2>().norm());
		return elevation >= this->lidarFovDown_ and elevation <= this->lidarFovUp_;
	}

	int DEP::getFrontierNum(){
		return int(this->frontierPointPairs_.size());
	}

	void DEP::findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths, std::vector<std::shared_ptr<PRM::Node>>& bestPath){
		// find path highest unknown
		bestPath.clear();
		double highestScore = -1;
		bool blockGain = this->useBlockGain();
		for (int n=0; n<int(candidatePaths.size()); ++n){
			std::vector<std::shared_ptr<PRM::Node>> path = candidatePaths[n]; 
			if (int(path.size()) == 0) continue;
			double yawDist = 0;
			double prevYaw = this->currYaw_;
			int unknownVoxel = 0;
			std::unordered_set<int> pathCells;
			for (int i=0; i<int(path.size())-1; ++i){
				std::shared_ptr<PRM::Node> currNode = path[i];
				std::shared_ptr<PRM::Node> nextNode = path[i+1];
				Eigen::Vector3d diff = nextNode->pos - currNode->pos;
				double angle = atan2(diff(1), diff(0));

				// reevaluate the unknowns for intermediate points
				if (blockGain){
					if (i > 0) this->addVisibleGainCells(currNode->pos, angle, pathCells); // the robot already sees what its start node does
				}
				else{
					std::unordered_map<double, int> yawNumVoxels;
					int unknownVoxelNum = this->calculateUnknown(currNode, yawNumVoxels);
					currNode->numVoxels = unknownVoxelNum;
					currNode->yawNumVoxels = yawNumVoxels;


					unknownVoxel += currNode->getUnknownVoxels(angle);
				}
				yawDist += globalPlanner::angleDiff(prevYaw, angle);
				prevYaw = angle;
			}
			// reevaluate the goal node. A 360 camera ranging by flow sees what its arrival heading allows, not what the best
			// heading would: it does not turn, and it slows to a stop there
			double goalYaw = this->useOmniModel_ ? prevYaw : path.back()->getBestYaw();
			if (blockGain){
				this->addVisibleGainCells(path.back()->pos, goalYaw, pathCells);
				unknownVoxel = int(pathCells.size()); // each cell once, however many nodes on the path see it
			}
			else{
				std::unordered_map<double, int> yawNumVoxels;
				int unknownVoxelNum = this->calculateUnknown(path.back(), yawNumVoxels);
				path.back()->numVoxels = unknownVoxelNum;
				path.back()->yawNumVoxels = yawNumVoxels;
				unknownVoxel += this->useOmniModel_ ? path.back()->getUnknownVoxels(goalYaw) : path.back()->getBestYawVoxel();
			}
			yawDist += globalPlanner::angleDiff(prevYaw, goalYaw);
			if (this->useOmniModel_) yawDist = 0.0; // it holds its yaw (hold_yaw): turning costs nothing

			double distance = this->calculatePathLength(path);
			// cout << "total is distance is: " << distance << " total yaw distance is: " << yawDist << " voxel: " << path.back()->numVoxels << endl;
			double pathTime = distance/this->vel_ + this->yawPenaltyWeight_ * yawDist/this->angularVel_;
			double score = double(unknownVoxel)/pathTime; 
			// the floor can only be seen from low down (lowest ray is about 30 deg below level), so prefer deep goals
			double regionHeight = this->gainRegionMax_(2) - this->gainRegionMin_(2);
			if (regionHeight > 0.0){
				double depth = std::max(0.0, std::min(1.0, (this->gainRegionMax_(2) - path.back()->pos(2))/regionHeight));
				score *= 1.0 + this->depthBias_ * depth * depth;
			}
			// a goal chosen before did not clear its predicted voxels, or it would have no gain left
			int visits = 0;
			for (const Eigen::Vector3d& pastGoal : this->pastGoals_){
				if ((pastGoal - path.back()->pos).norm() < 1.0) ++visits;
			}
			score /= 1.0 + this->revisitPenalty_ * visits;
			// cout << "unknown for path: " << n <<  " is: " << unknownVoxel << " score: " << score << " distance: " << distance << " Time: " << pathTime <<  " Last total unknown: " << path.back()->numVoxels << " last best: " << path.back()->getBestYawVoxel() << endl;
			if (score > highestScore){
				highestScore = score;
				bestPath = path;
				this->bestScore_ = score;
				this->bestGain_ = unknownVoxel;
				this->bestPathTime_ = pathTime;
				this->candidatePathNum_ = int(candidatePaths.size());
			}
		}
		if (highestScore == 0){
			cout << "[DEP]: Current score is 0. The exploration might complete." << endl;
		}
		if (not bestPath.empty()) this->pastGoals_.push_back(bestPath.back()->pos);
	}


	void DEP::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		this->position_ = Eigen::Vector3d (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		this->currYaw_ = globalPlanner::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		this->odomReceived_ = true;

		if (this->histTraj_.size() == 0){
			this->histTraj_.push_back(this->position_);
		}
		else{
			Eigen::Vector3d lastPos = this->histTraj_.back();
			double dist = (this->position_ - lastPos).norm();
			if (dist >= 0.5){
				if (this->histTraj_.size() >= 100){
					this->histTraj_.pop_front();
					this->histTraj_.push_back(this->position_);
				}
				else{
					this->histTraj_.push_back(this->position_);
				}
			}
		}
	}

	void DEP::visCB(const ros::TimerEvent&){
		if (this->prmNodeVec_.size() != 0){
			this->publishRoadmap();
		}

		if (this->candidatePaths_.size() != 0){
			this->publishCandidatePaths();
		}

		if (this->bestPath_.size() != 0){
			this->publishBestPath();
		}

		if (this->tourStops_.size() != 0){
			this->publishTour();
		}

		if (this->frontierPointPairs_.size() != 0){
			this->publishFrontier();
		}
	}


	bool DEP::isPosValid(const Eigen::Vector3d& p){
		for (double x=p(0)-this->safeDistXY_; x<=p(0)+this->safeDistXY_; x+=this->map_->getRes()){
			for (double y=p(1)-this->safeDistXY_; y<=p(1)+this->safeDistXY_; y+=this->map_->getRes()){
				for (double z=p(2)-this->safeDistZ_; z<=p(2)+this->safeDistZ_; z+=this->map_->getRes()){
					if (this->safeDistCheckUnknown_ and this->maxUnknownGap_ <= 0.0){
						if (not this->map_->isInflatedFree(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
					else{
						if (this->map_->isInflatedOccupied(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
				}
			}
		}
		if (this->safeDistCheckUnknown_ and not this->map_->isInflatedFree(p)) return false; // the node itself must be observed
		return true;			
	}


	bool DEP::isPosValid(const Eigen::Vector3d& p, double safeDistXY, double safeDistZ){
		for (double x=p(0)-safeDistXY; x<=p(0)+safeDistXY; x+=this->map_->getRes()){
			for (double y=p(1)-safeDistXY; y<=p(1)+safeDistXY; y+=this->map_->getRes()){
				for (double z=p(2)-safeDistZ; z<=p(2)+safeDistZ; z+=this->map_->getRes()){
					if (this->safeDistCheckUnknown_ and this->maxUnknownGap_ <= 0.0){
						if (not this->map_->isInflatedFree(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
					else{
						if (this->map_->isInflatedOccupied(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
				}
			}
		}
		if (this->safeDistCheckUnknown_ and not this->map_->isInflatedFree(p)) return false; // the node itself must be observed
		return true;		
	}

	std::shared_ptr<PRM::Node> DEP::randomConfigBBox(const Eigen::Vector3d& minRegion, const Eigen::Vector3d& maxRegion){
		Eigen::Vector3d mapMinRegion, mapMaxRegion, minSampleRegion, maxSampleRegion;
		this->map_->getCurrMapRange(mapMinRegion, mapMaxRegion);
		// cout << "current map range is: " << mapMinRegion.transpose() << ", " << mapMaxRegion.transpose() << endl;
		minSampleRegion(0) = std::max(mapMinRegion(0), minRegion(0));
		minSampleRegion(1) = std::max(mapMinRegion(1), minRegion(1));
		minSampleRegion(2) = std::max(mapMinRegion(2), minRegion(2));
		maxSampleRegion(0) = std::min(mapMaxRegion(0), maxRegion(0));
		maxSampleRegion(1) = std::min(mapMaxRegion(1), maxRegion(1));
		maxSampleRegion(2) = std::min(mapMaxRegion(2), maxRegion(2));

		minSampleRegion(0) = std::max(minSampleRegion(0), this->globalRegionMin_(0));
		minSampleRegion(1) = std::max(minSampleRegion(1), this->globalRegionMin_(1));
		minSampleRegion(2) = std::max(minSampleRegion(2), this->globalRegionMin_(2));
		maxSampleRegion(0) = std::min(maxSampleRegion(0), this->globalRegionMax_(0));
		maxSampleRegion(1) = std::min(maxSampleRegion(1), this->globalRegionMax_(1));
		maxSampleRegion(2) = std::min(maxSampleRegion(2), this->globalRegionMax_(2));


		bool valid = false;
		Eigen::Vector3d p;
		while (valid == false){	
			p(0) = globalPlanner::randomNumber(minSampleRegion(0), maxSampleRegion(0));
			p(1) = globalPlanner::randomNumber(minSampleRegion(1), maxSampleRegion(1));
			p(2) = globalPlanner::randomNumber(minSampleRegion(2), maxSampleRegion(2));

			valid = this->isPosValid(p, this->safeDistXY_, this->safeDistZ_);

			// valid = this->map_->isInflatedFree(p);
		}

		std::shared_ptr<PRM::Node> newNode (new PRM::Node(p));
		return newNode;
	}

	bool DEP::setTourEnabled(bool enabled){
		if (enabled and not this->useBlockGain()) return false; // areas are built from the gain cells
		this->tourEnabled_ = enabled;
		this->tourReset_ = true;
		return true;
	}

	bool DEP::isTourEnabled(){
		return this->tourEnabled_;
	}

	bool DEP::isTourDone(){
		return this->tourPlanActive_ and this->tourDone_;
	}

	void DEP::resetTour(){
		this->tourReset_ = true;
	}

	void DEP::getTourStats(int& cellsLeft, int& givenUp){
		cellsLeft = this->tourCellsLeft_;
		givenUp = this->tourGivenUp_;
	}

	bool DEP::planTour(){
		if (this->tourReset_.exchange(false)){
			this->tourUsedViews_.clear();
			this->tourOrder_.clear();
		}
		this->tourDone_ = false;
		this->tourStops_.clear();
		this->tourCellsLeft_ = int(this->gainCells_.size());
		this->tourGivenUp_ = 0;

		// fixed bins rather than connected groups: early on the unknown is one huge group, and a bin keeps its identity between plans
		std::map<std::array<int, 3>, std::vector<int>> bins;
		for (size_t i=0; i<this->gainCells_.size(); ++i){
			Eigen::Vector3d rel = (this->gainCells_[i] - this->gainRegionMin_) / this->tourBinSize_;
			bins[{int(std::floor(rel(0))), int(std::floor(rel(1))), int(std::floor(rel(2)))}].push_back(int(i));
		}

		// roadmap nodes connected to the robot, as a plain graph. Climbs cost extra, as they do in flight
		std::shared_ptr<PRM::Node> start = this->getConnectedNode(this->position_);
		std::vector<std::shared_ptr<PRM::Node>> nodes {start};
		std::unordered_map<std::shared_ptr<PRM::Node>, int> nodeIdx {{start, 0}};
		for (size_t i=0; i<nodes.size(); ++i){
			std::shared_ptr<PRM::Node> n = nodes[i];
			for (const std::shared_ptr<PRM::Node>& neighbor : n->adjNodes){
				if (this->edgeAllowed(n->pos, neighbor->pos) and nodeIdx.emplace(neighbor, int(nodes.size())).second) nodes.push_back(neighbor);
			}
		}
		this->reachableNodeNum_ = int(nodes.size());
		std::vector<std::vector<std::pair<int, double>>> graph (nodes.size());
		for (size_t i=0; i<nodes.size(); ++i){
			for (const std::shared_ptr<PRM::Node>& neighbor : nodes[i]->adjNodes){
				if (not this->edgeAllowed(nodes[i]->pos, neighbor->pos)) continue;
				Eigen::Vector3d diff = neighbor->pos - nodes[i]->pos;
				graph[i].push_back({nodeIdx[neighbor], diff.norm() + 0.5 * std::abs(diff(2))});
			}
		}

		// largest areas first. Left to itself a shortest route clears every nearby scrap in tiny hops
		// before it goes anywhere, so small areas wait until they are among the largest left
		typedef std::pair<const std::array<int, 3>, std::vector<int>> Bin;
		std::vector<const Bin*> ordered;
		int liveBins = 0;
		size_t largest = 0;
		for (const Bin& bin : bins){
			if (int(bin.second.size()) < this->tourMinCells_) continue;
			if (int(this->tourUsedViews_[bin.first].size()) >= this->tourMaxVisits_){
				++this->tourGivenUp_;
				continue;
			}
			++liveBins;
			largest = std::max(largest, bin.second.size());
			ordered.push_back(&bin);
		}
		if (liveBins == 0){
			this->tourDone_ = true;
			return false;
		}
		std::sort(ordered.begin(), ordered.end(), [](const Bin* a, const Bin* b){return a->second.size() > b->second.size();});

		double gainRange = this->gainRange();
		struct Stop {std::array<int, 3> bin; int node; double yaw; int cells;};
		std::vector<Stop> stops;
		auto chooseStops = [&](size_t threshold){
			stops.clear();
			std::set<std::array<int, 3>> covered; // seen from a stop already chosen: no stop of its own
			for (size_t b=0; b<ordered.size(); ++b){
				const Bin& bin = *ordered[b];
				if (bin.second.size() < threshold or covered.count(bin.first) != 0) continue;
				const std::vector<Eigen::Vector3d>& used = this->tourUsedViews_[bin.first];
				int stride = std::max(1, int(bin.second.size()) / 12);
				Eigen::Vector3d centre (0.0, 0.0, 0.0);
				for (int idx : bin.second) centre += this->gainCells_[idx];
				centre /= double(bin.second.size());

				double bestScore = 0.0;
				int bestNode = -1, bestYawIdx = 0;
				for (size_t i=0; i<nodes.size(); ++i){
					const Eigen::Vector3d& pos = nodes[i]->pos;
					if ((pos - centre).norm() > gainRange + this->tourBinSize_) continue;
					if ((pos - this->position_).norm() < this->minGoalDist_) continue;
					bool usedBefore = false;
					for (const Eigen::Vector3d& view : used) usedBefore = usedBefore or (view - pos).norm() < 1.0;
					if (usedBefore) continue;
					std::vector<int> yawCount (this->yaws_.size(), 0);
					for (size_t j=0; j<bin.second.size(); j+=stride){
						const Eigen::Vector3d& cell = this->gainCells_[bin.second[j]];
						Eigen::Vector3d direction = cell - pos;
						if (direction.norm() > gainRange) continue;
						bool visibleAtAnyYaw = false;
						std::vector<bool> visibleAtYaw (this->yaws_.size(), false);
						for (size_t k=0; k<this->yaws_.size(); ++k){
							visibleAtYaw[k] = this->lidarVisible(direction, k);
							visibleAtAnyYaw = visibleAtAnyYaw or visibleAtYaw[k];
						}
						if (not visibleAtAnyYaw or not this->gainCellInSight(cell, pos)) continue;
						for (size_t k=0; k<this->yaws_.size(); ++k){
							if (visibleAtYaw[k]) ++yawCount[k];
						}
					}
					int yawIdx = int(std::max_element(yawCount.begin(), yawCount.end()) - yawCount.begin());
					if (yawCount[yawIdx] == 0) continue;
					double score = yawCount[yawIdx] - 0.01 * (pos - this->position_).norm(); // equal views: the nearer one
					if (bestNode >= 0 and score <= bestScore) continue;
					if (not this->isPosValid(pos, this->safeDistXY_, this->safeDistZ_)) continue;
					bestScore = score;
					bestNode = int(i);
					bestYawIdx = yawIdx;
				}
				if (bestNode < 0) continue;
				stops.push_back({bin.first, bestNode, this->yaws_[bestYawIdx], int(bin.second.size())});

				// smaller areas this stop sees most of need no stop of their own
				const Eigen::Vector3d& view = nodes[bestNode]->pos;
				for (size_t o=b+1; o<ordered.size(); ++o){
					const Bin& other = *ordered[o];
					int otherStride = std::max(1, int(other.second.size()) / 12), samples = 0, seen = 0;
					for (size_t j=0; j<other.second.size(); j+=otherStride){
						const Eigen::Vector3d& cell = this->gainCells_[other.second[j]];
						Eigen::Vector3d direction = cell - view;
						++samples;
						if (direction.norm() <= gainRange and this->lidarVisible(direction, bestYawIdx) and this->gainCellInSight(cell, view)) ++seen;
					}
					if (seen * 2 >= samples) covered.insert(other.first);
				}
			}
		};
		chooseStops(std::max(size_t(this->tourMinCells_), size_t(std::ceil(this->tourMinShare_ * largest))));
		if (stops.empty()) chooseStops(size_t(this->tourMinCells_)); // large areas out of reach for now: take what can be reached
		if (stops.empty()) return false; // areas left but none in view of the roadmap yet: it grows with each plan

		// travel cost between the robot (index 0) and every stop (index i + 1)
		int stopNum = int(stops.size());
		std::vector<std::vector<double>> cost (stopNum + 1, std::vector<double>(stopNum + 1, 0.0));
		for (int s=0; s<=stopNum; ++s){
			std::vector<double> dist (nodes.size(), std::numeric_limits<double>::infinity());
			int source = s == 0 ? 0 : stops[s-1].node;
			dist[source] = 0.0;
			typedef std::pair<double, int> QueueItem;
			std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
			queue.push({0.0, source});
			while (not queue.empty()){
				QueueItem top = queue.top();
				queue.pop();
				if (top.first > dist[top.second]) continue;
				for (const std::pair<int, double>& edge : graph[top.second]){
					if (top.first + edge.second < dist[edge.first]){
						dist[edge.first] = top.first + edge.second;
						queue.push({dist[edge.first], edge.first});
					}
				}
			}
			for (int t=0; t<stopNum; ++t) cost[s][t+1] = dist[stops[t].node];
			cost[s][0] = dist[0];
		}
		auto orderCost = [&cost](const std::vector<int>& order){ // an open path from the robot: no need to come back
			double total = 0.0;
			int previous = 0;
			for (int stop : order){
				total += cost[previous][stop];
				previous = stop;
			}
			return total;
		};

		// fresh order: nearest stop first, then undo crossings (2-opt)
		std::vector<int> fresh;
		std::vector<bool> taken (stopNum + 1, false);
		int previous = 0;
		for (int i=0; i<stopNum; ++i){
			int nearest = -1;
			for (int t=1; t<=stopNum; ++t){
				if (not taken[t] and (nearest < 0 or cost[previous][t] < cost[previous][nearest])) nearest = t;
			}
			taken[nearest] = true;
			fresh.push_back(nearest);
			previous = nearest;
		}
		bool improved = true;
		for (int pass=0; improved and pass<30; ++pass){
			improved = false;
			for (int i=0; i<stopNum-1; ++i){
				for (int j=i+1; j<stopNum; ++j){
					int before = i == 0 ? 0 : fresh[i-1];
					double change = cost[before][fresh[j]] - cost[before][fresh[i]];
					if (j + 1 < stopNum) change += cost[fresh[i]][fresh[j+1]] - cost[fresh[j]][fresh[j+1]];
					if (change < -1e-6){
						std::reverse(fresh.begin() + i, fresh.begin() + j + 1);
						improved = true;
					}
				}
			}
		}

		// keep the last order where it is not much worse: a tour that reshuffles every plan zigzags like a greedy one
		std::vector<int> kept;
		for (const std::array<int, 3>& bin : this->tourOrder_){
			for (int t=0; t<stopNum; ++t){
				if (stops[t].bin == bin) kept.push_back(t + 1);
			}
		}
		for (int t=1; t<=stopNum; ++t){
			if (std::find(kept.begin(), kept.end(), t) != kept.end()) continue;
			size_t bestAt = kept.size();
			double bestChange = std::numeric_limits<double>::infinity();
			for (size_t at=0; at<=kept.size(); ++at){
				int before = at == 0 ? 0 : kept[at-1];
				double change = cost[before][t] - (at < kept.size() ? cost[before][kept[at]] : 0.0) + (at < kept.size() ? cost[t][kept[at]] : 0.0);
				if (change < bestChange){
					bestChange = change;
					bestAt = at;
				}
			}
			kept.insert(kept.begin() + bestAt, t);
		}
		const std::vector<int>& order = orderCost(kept) <= 1.15 * orderCost(fresh) ? kept : fresh;

		this->tourOrder_.clear();
		this->goalCandidates_.clear();
		for (int t : order){
			this->tourOrder_.push_back(stops[t-1].bin);
			this->tourStops_.push_back(nodes[stops[t-1].node]);
			this->goalCandidates_.push_back(nodes[stops[t-1].node]);
		}

		// first stop the roadmap can still route to. A stop that fails uses up one of its area's tries
		std::shared_ptr<PRM::Node> currPos;
		currPos.reset(new PRM::Node (this->position_));
		for (int t : order){
			const Stop& stop = stops[t-1];
			std::shared_ptr<PRM::Node> goal = nodes[stop.node];
			this->tourUsedViews_[stop.bin].push_back(goal->pos);
			std::vector<std::shared_ptr<PRM::Node>> path = PRM::AStar(this->roadmap_, start, goal, this->map_, this->maxUnknownGap_, [this](const Eigen::Vector3d& from, const Eigen::Vector3d& to){return this->edgeAllowed(from, to);});
			if (path.empty()) continue;
			path.insert(path.begin(), currPos);
			std::vector<std::shared_ptr<PRM::Node>> pathSc;
			this->shortcutPath(path, pathSc);
			this->candidatePaths_ = {pathSc};
			this->bestPath_ = pathSc;
			this->tourGoalYaw_ = stop.yaw;
			double pathTime = std::max(0.1, this->calculatePathLength(pathSc) / this->vel_);
			this->bestGain_ = stop.cells;
			this->bestPathTime_ = pathTime;
			this->bestScore_ = stop.cells / pathTime;
			this->candidatePathNum_ = stopNum;
			return true;
		}
		return false;
	}

	void DEP::publishTour(){
		visualization_msgs::MarkerArray tourMarkers;
		visualization_msgs::Marker line;
		line.header.frame_id = "map";
		line.header.stamp = ros::Time::now();
		line.ns = "tour";
		line.id = 0;
		line.type = visualization_msgs::Marker::LINE_LIST;
		line.action = visualization_msgs::Marker::ADD;
		line.pose.orientation.w = 1.0;
		line.scale.x = 0.05;
		line.color.a = 1.0;
		line.color.r = 1.0;
		line.color.g = 0.5;
		Eigen::Vector3d previous = this->position_;
		int id = 0;
		for (const std::shared_ptr<PRM::Node>& n : this->tourStops_){
			geometry_msgs::Point p1, p2;
			p1.x = previous(0); p1.y = previous(1); p1.z = previous(2);
			p2.x = n->pos(0); p2.y = n->pos(1); p2.z = n->pos(2);
			line.points.push_back(p1);
			line.points.push_back(p2);
			previous = n->pos;
			visualization_msgs::Marker stop;
			stop.header = line.header;
			stop.ns = "tour_stop";
			stop.id = id++;
			stop.type = visualization_msgs::Marker::SPHERE;
			stop.action = visualization_msgs::Marker::ADD;
			stop.pose.position = p2;
			stop.pose.orientation.w = 1.0;
			stop.scale.x = stop.scale.y = stop.scale.z = 0.25;
			stop.color = line.color;
			tourMarkers.markers.push_back(stop);
		}
		tourMarkers.markers.push_back(line);
		this->tourPub_.publish(tourMarkers);
	}

	double DEP::gainRange(){
		// cells are mostly surfaces by the end, and those only count from inspect_range
		double inspectRange = this->map_->getInspectRange();
		return inspectRange > 0.0 ? std::min(this->dmax_, inspectRange) : this->dmax_;
	}

	bool DEP::useBlockGain(){
		return this->gainBlockSize_ > 0.0 and this->useLidarModel_;
	}

	void DEP::updateGainCells(){
		// a thin lidar ray leaves unknown voxels between known ones and most of those never clear, so counting voxels
		// never reaches zero: count blocks with no known voxel at all
		if (not this->useBlockGain()) return;
		this->gainCells_.clear();
		double res = this->map_->getRes();
		int half = int(std::round(this->gainBlockSize_ / res)) / 2;
		for (double x=this->gainRegionMin_(0)+this->gainBlockSize_/2; x<this->gainRegionMax_(0); x+=this->gainBlockSize_){
			for (double y=this->gainRegionMin_(1)+this->gainBlockSize_/2; y<this->gainRegionMax_(1); y+=this->gainBlockSize_){
				for (double z=this->gainRegionMin_(2)+this->gainBlockSize_/2; z<this->gainRegionMax_(2); z+=this->gainBlockSize_){
					bool anyKnown = false, pending = false; // open air clears from any range, a surface only from inspect_range
					for (int i=-half; i<=half and not pending; ++i){
						for (int j=-half; j<=half and not pending; ++j){
							for (int k=-half; k<=half and not pending; ++k){
								Eigen::Vector3d p (x + i * res, y + j * res, z + k * res);
								anyKnown = anyKnown or (this->map_->isInMap(p) and not this->map_->isUnknown(p));
								pending = this->map_->isInspectionPending(p);
							}
						}
					}
					if (not anyKnown or pending) this->gainCells_.push_back(Eigen::Vector3d (x, y, z));
				}
			}
		}
	}

	bool DEP::gainCellInSight(const Eigen::Vector3d& cell, const Eigen::Vector3d& pos){
		// real occupancy, not inflated: the inflation would hide everything near a wall or the floor.
		// Stops short of the cell so the floor under a floor patch does not block its own view
		Eigen::Vector3d direction = cell - pos;
		double distance = direction.norm();
		if (distance < 1e-3) return true;
		direction /= distance;
		double step = 2.0 * this->map_->getRes();
		for (double d=step; d<distance-0.4; d+=step){
			Eigen::Vector3d p = pos + d * direction;
			if (this->map_->isInMap(p) and this->map_->isOccupied(p)) return false;
		}
		return true;
	}

	void DEP::addVisibleGainCells(const Eigen::Vector3d& pos, double yaw, std::unordered_set<int>& cells){
		int yawIdx = 0;
		for (size_t i=1; i<this->yaws_.size(); ++i){
			if (globalPlanner::angleDiff(yaw, this->yaws_[i]) < globalPlanner::angleDiff(yaw, this->yaws_[yawIdx])) yawIdx = int(i);
		}
		double gainRange = this->gainRange();
		for (size_t i=0; i<this->gainCells_.size(); ++i){
			Eigen::Vector3d direction = this->gainCells_[i] - pos;
			if (direction.norm() <= gainRange and this->lidarVisible(direction, yawIdx) and this->gainCellInSight(this->gainCells_[i], pos)) cells.insert(int(i));
		}
	}

	int DEP::calculateUnknown(const shared_ptr<PRM::Node>& n, std::unordered_map<double, int>& yawNumVoxels){
		for (double yaw : this->yaws_){
			yawNumVoxels[yaw] = 0;
		}
		// Position:
		Eigen::Vector3d p = n->pos;

		if (this->useBlockGain()){
			int countCells = 0;
			double gainRange = this->gainRange();
			for (const Eigen::Vector3d& cell : this->gainCells_){
				Eigen::Vector3d direction = cell - p;
				if (direction.norm() > gainRange) continue;
				bool visibleAtAnyYaw = false;
				std::vector<bool> visibleAtYaw (this->yaws_.size(), false);
				for (size_t i=0; i<this->yaws_.size(); ++i){
					visibleAtYaw[i] = this->lidarVisible(direction, i);
					visibleAtAnyYaw = visibleAtAnyYaw or visibleAtYaw[i];
				}
				if (not visibleAtAnyYaw or not this->gainCellInSight(cell, p)) continue;
				++countCells;
				for (size_t i=0; i<this->yaws_.size(); ++i){
					if (visibleAtYaw[i]) yawNumVoxels[this->yaws_[i]] += 1;
				}
			}
			return countCells;
		}

		// a tilted 360 deg lidar sees well above and below the robot, so search the full range in z
		double zRange = this->useLidarModel_ ? this->dmax_ : this->dmax_ * tan(this->verticalFOV_/2.0);
		int countTotalUnknown = 0;
		for (double z = p(2) - zRange; z <= p(2) + zRange; z += this->map_->getRes()){
			for (double y = p(1) - this->dmax_; y <= p(1)+ this->dmax_; y += this->map_->getRes()){
				for (double x = p(0) - this->dmax_; x <= p(0) + this->dmax_; x += this->map_->getRes()){
					Eigen::Vector3d nodePoint (x, y, z);
					if (nodePoint(0) < this->gainRegionMin_(0) or nodePoint(0) > this->gainRegionMax_(0) or
						nodePoint(1) < this->gainRegionMin_(1) or nodePoint(1) > this->gainRegionMax_(1) or
						nodePoint(2) < this->gainRegionMin_(2) or nodePoint(2) > this->gainRegionMax_(2)){
						// not in gain range
						continue;
					}

					if (this->map_->isUnknown(nodePoint) and not this->map_->isInflatedOccupied(nodePoint)){
						if (this->useLidarModel_){
							Eigen::Vector3d direction = nodePoint - p;
							// cheap elevation test first, the line-of-sight raycast in sensorFOVCondition last
							bool visibleAtAnyYaw = false;
							std::vector<bool> visibleAtYaw (this->yaws_.size(), false);
							for (size_t i=0; i<this->yaws_.size(); ++i){
								visibleAtYaw[i] = this->lidarVisible(direction, i);
								visibleAtAnyYaw = visibleAtAnyYaw or visibleAtYaw[i];
							}
							if (visibleAtAnyYaw and this->sensorFOVCondition(nodePoint, p)){
								++countTotalUnknown;
								for (size_t i=0; i<this->yaws_.size(); ++i){
									if (visibleAtYaw[i]){
										yawNumVoxels[this->yaws_[i]] += 1;
									}
								}
							}
						}
						else if (this->sensorFOVCondition(nodePoint, p)){
							++countTotalUnknown;
							for (double yaw: this->yaws_){
								Eigen::Vector3d yawDirection (cos(yaw), sin(yaw), 0);
								Eigen::Vector3d direction = nodePoint - p;
								Eigen::Vector3d face (direction(0), direction(1), 0);
								double angleToYaw = angleBetweenVectors(face, yawDirection);
								if (angleToYaw <= this->horizontalFOV_/2){
									yawNumVoxels[yaw] += 1;
								}
							}
						}
					}
				}
			}
		}
		return countTotalUnknown;
	}
	double DEP::calculatePathLength(const std::vector<shared_ptr<PRM::Node>>& path){
		int idx1 = 0;
		double length = 0;
		for (size_t idx2=1; idx2<=path.size()-1; ++idx2){
			length += (path[idx2]->pos - path[idx1]->pos).norm();
			++idx1;
		}
		return length;
	}

	void DEP::shortcutPath(const std::vector<std::shared_ptr<PRM::Node>>& path, std::vector<std::shared_ptr<PRM::Node>>& pathSc){
		size_t ptr1 = 0; size_t ptr2 = 2;
		pathSc.push_back(path[ptr1]);

		if (path.size() == 1){
			return;
		}

		if (path.size() == 2){
			pathSc.push_back(path[1]);
			return;
		}

		while (ros::ok()){
			if (ptr2 > path.size()-1){
				break;
			}
			std::shared_ptr<PRM::Node> p1 = path[ptr1];
			std::shared_ptr<PRM::Node> p2 = path[ptr2];
			Eigen::Vector3d pos1 = p1->pos;
			Eigen::Vector3d pos2 = p2->pos;
			bool lineValidCheck;
			// lineValidCheck = not this->map_->isInflatedOccupiedLine(pos1, pos2);
			lineValidCheck = PRM::isClearLine(this->map_, pos1, pos2, this->maxUnknownGap_);
			// double maxDistance = std::numeric_limits<double>::max();
			// double maxDistance = 3.0;
			// if (lineValidCheck and (pos1 - pos2).norm() <= maxDistance){
			if (lineValidCheck){
				if (ptr2 == path.size()-1){
					pathSc.push_back(p2);
					break;
				}
				++ptr2;
			}
			else{
				pathSc.push_back(path[ptr2-1]);
				if (ptr2 == path.size()-1){
					pathSc.push_back(p2);
					break;
				}
				ptr1 = ptr2-1;
				ptr2 = ptr1+2;
			}
		}		
	}

	int DEP::weightedSample(const std::vector<double>& weights){
		double total = std::accumulate(weights.begin(), weights.end(), 0.0);
		std::vector<double> normalizedWeights;

		 for (const double weight : weights){
		 	normalizedWeights.push_back(weight/total);
		 }

		std::random_device rd;
		std::mt19937 gen(rd());
		std::discrete_distribution<int> distribution(normalizedWeights.begin(), normalizedWeights.end());
		return distribution(gen);
	}


	std::shared_ptr<PRM::Node> DEP::sampleFrontierPoint(const std::vector<double>& sampleWeights){
		// choose the frontier region (random sample by frontier area) 
		int idx = weightedSample(sampleWeights);

		// sample a frontier point in the region
		Eigen::Vector3d frontierCenter = this->frontierPointPairs_[idx].first;
		double frontierSize = this->frontierPointPairs_[idx].second;
		double xmin = std::max(frontierCenter(0) - frontierSize/sqrt(2), this->globalRegionMin_(0));
		double xmax = std::min(frontierCenter(0) + frontierSize/sqrt(2), this->globalRegionMax_(0));
		double ymin = std::max(frontierCenter(1) - frontierSize/sqrt(2), this->globalRegionMin_(1));
		double ymax = std::min(frontierCenter(1) + frontierSize/sqrt(2), this->globalRegionMax_(1));
		double zmin = frontierCenter(2);
		double zmax = frontierCenter(2);
		Eigen::Vector3d frontierPoint;
		frontierPoint(0) = globalPlanner::randomNumber(xmin, xmax);
		frontierPoint(1) = globalPlanner::randomNumber(ymin, ymax);
		frontierPoint(2) = globalPlanner::randomNumber(zmin, zmax);
		std::shared_ptr<PRM::Node> frontierNode (new PRM::Node(frontierPoint));
		return frontierNode;
	}

	std::shared_ptr<PRM::Node> DEP::extendNode(const std::shared_ptr<PRM::Node>& n, const std::shared_ptr<PRM::Node>& target){
		double extendDist = randomNumber(this->distThresh_, this->maxConnectDist_);
		Eigen::Vector3d p = n->pos + (target->pos - n->pos)/(target->pos - n->pos).norm() * extendDist;
		p(0) = std::max(this->globalRegionMin_(0), std::min(p(0), this->globalRegionMax_(0)));
		p(1) = std::max(this->globalRegionMin_(1), std::min(p(1), this->globalRegionMax_(1)));
		p(2) = std::max(this->globalRegionMin_(2), std::min(p(2), this->globalRegionMax_(2)));
		std::shared_ptr<PRM::Node> extendedNode (new PRM::Node(p));
		return extendedNode;
	}

	void DEP::publishRoadmap(){
		visualization_msgs::MarkerArray roadmapMarkers;

		// PRM nodes and edges
		int countPointNum = 0;
		int countEdgeNum = 0;
		int countVoxelNumText = 0;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			// std::shared_ptr<PRM::Node> n = this->prmNodeVec_[i];

			// Node point
			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "prm_point";
			point.id = countPointNum;
			point.type = visualization_msgs::Marker::SPHERE;
			point.action = visualization_msgs::Marker::ADD;
			point.pose.position.x = n->pos(0);
			point.pose.position.y = n->pos(1);
			point.pose.position.z = n->pos(2);
			point.lifetime = ros::Duration(0.5);
			point.scale.x = 0.1;
			point.scale.y = 0.1;
			point.scale.z = 0.1;
			point.color.a = 1.0;
			point.color.r = 1.0;
			point.color.g = 0.0;
			point.color.b = 0.0;
			++countPointNum;
			roadmapMarkers.markers.push_back(point);

			// number of voxels for each node
			visualization_msgs::Marker voxelNumText;
			voxelNumText.ns = "num_voxel_text";
			voxelNumText.header.frame_id = "map";
			voxelNumText.id = countVoxelNumText;
			voxelNumText.header.stamp = ros::Time::now();
			voxelNumText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			voxelNumText.action = visualization_msgs::Marker::ADD;
			voxelNumText.pose.position.x = n->pos(0);
			voxelNumText.pose.position.y = n->pos(1);
			voxelNumText.pose.position.z = n->pos(2)+0.1;
			voxelNumText.scale.x = 0.1;
			voxelNumText.scale.y = 0.1;
			voxelNumText.scale.z = 0.1;
			voxelNumText.color.a = 1.0;
			voxelNumText.text = std::to_string(n->numVoxels);
			voxelNumText.lifetime = ros::Duration(0.5);
			++countVoxelNumText;
			roadmapMarkers.markers.push_back(voxelNumText);


			// Edges
			visualization_msgs::Marker line;
			line.ns = "edge";
			line.header.frame_id = "map";
			line.type = visualization_msgs::Marker::LINE_LIST;
			line.header.stamp = ros::Time::now();
			for (std::shared_ptr<PRM::Node> adjNode : n->adjNodes){
				if (not this->edgeAllowed(n->pos, adjNode->pos) and not this->edgeAllowed(adjNode->pos, n->pos)) continue;
				geometry_msgs::Point p1, p2;
				p1.x = n->pos(0);
				p1.y = n->pos(1);
				p1.z = n->pos(2);
				p2.x = adjNode->pos(0);
				p2.y = adjNode->pos(1);
				p2.z = adjNode->pos(2);				
				line.points.push_back(p1);
				line.points.push_back(p2);
				line.id = countEdgeNum;
				line.scale.x = 0.05;
				line.scale.y = 0.05;
				line.scale.z = 0.05;
				line.color.r = 0.0;
				line.color.g = 1.0;
				line.color.b = 0.0;
				line.color.a = 1.0;
				line.lifetime = ros::Duration(0.5);
				++countEdgeNum;
				roadmapMarkers.markers.push_back(line);
			}
		}

		int countGoalCandidateNum = 0;
		for (size_t i=0; i<this->goalCandidates_.size(); ++i){
			std::shared_ptr<PRM::Node> n = this->goalCandidates_[i];

			// Goal candidates
			visualization_msgs::Marker goalCandidatePoint;
			goalCandidatePoint.ns = "goal_candidate";
			goalCandidatePoint.header.frame_id = "map";
			goalCandidatePoint.header.stamp = ros::Time::now();
			goalCandidatePoint.id = countGoalCandidateNum;
			goalCandidatePoint.type = visualization_msgs::Marker::SPHERE;
			goalCandidatePoint.action = visualization_msgs::Marker::ADD;
			goalCandidatePoint.pose.position.x = n->pos(0);
			goalCandidatePoint.pose.position.y = n->pos(1);
			goalCandidatePoint.pose.position.z = n->pos(2);
			goalCandidatePoint.lifetime = ros::Duration(0.5);
			goalCandidatePoint.scale.x = 0.2;
			goalCandidatePoint.scale.y = 0.2;
			goalCandidatePoint.scale.z = 0.2;
			goalCandidatePoint.color.a = 1.0;
			goalCandidatePoint.color.r = 1.0;
			goalCandidatePoint.color.g = 0.0;
			goalCandidatePoint.color.b = 1.0;
			++countGoalCandidateNum;
			roadmapMarkers.markers.push_back(goalCandidatePoint);
		}

		this->roadmapPub_.publish(roadmapMarkers);
	}

	void DEP::publishCandidatePaths(){
		visualization_msgs::MarkerArray candidatePathMarkers;
		int countNodeNum = 0;
		int countLineNum = 0;
		for (std::vector<std::shared_ptr<PRM::Node>>& path : this->candidatePaths_){
			for (size_t i=0; i<path.size(); ++i){
				std::shared_ptr<PRM::Node> n = path[i];
				visualization_msgs::Marker point;
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "candidate_path_node";
				point.id = countNodeNum;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = n->pos(0);
				point.pose.position.y = n->pos(1);
				point.pose.position.z = n->pos(2);
				point.lifetime = ros::Duration(0.1);
				point.scale.x = 0.15;
				point.scale.y = 0.15;
				point.scale.z = 0.15;
				point.color.a = 1.0;
				point.color.r = 1.0;
				point.color.g = 1.0;
				point.color.b = 0.0;
				++countNodeNum;
				candidatePathMarkers.markers.push_back(point);

				if (i<path.size()-1){
					std::shared_ptr<PRM::Node> nNext = path[i+1];
					visualization_msgs::Marker line;
					line.ns = "candidate_path";
					line.header.frame_id = "map";
					line.type = visualization_msgs::Marker::LINE_LIST;
					line.header.stamp = ros::Time::now();
					geometry_msgs::Point p1, p2;
					p1.x = n->pos(0);
					p1.y = n->pos(1);
					p1.z = n->pos(2);
					p2.x = nNext->pos(0);
					p2.y = nNext->pos(1);
					p2.z = nNext->pos(2);				
					line.points.push_back(p1);
					line.points.push_back(p2);
					line.id = countLineNum;
					line.scale.x = 0.1;
					line.scale.y = 0.1;
					line.scale.z = 0.1;
					line.color.r = 0.0;
					line.color.g = 0.0;
					line.color.b = 0.0;
					line.color.a = 1.0;
					line.lifetime = ros::Duration(0.5);
					++countLineNum;
					candidatePathMarkers.markers.push_back(line);				
				}
			}
		}
		this->candidatePathPub_.publish(candidatePathMarkers);		
	}
	
	void DEP::publishBestPath(){
		visualization_msgs::MarkerArray bestPathMarkers;
		int countNodeNum = 0;
		int countLineNum = 0;
		for (size_t i=0; i<this->bestPath_.size(); ++i){
			std::shared_ptr<PRM::Node> n = this->bestPath_[i];
			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "best_path_node";
			point.id = countNodeNum;
			point.type = visualization_msgs::Marker::SPHERE;
			point.action = visualization_msgs::Marker::ADD;
			point.pose.position.x = n->pos(0);
			point.pose.position.y = n->pos(1);
			point.pose.position.z = n->pos(2);
			point.lifetime = ros::Duration(0.5);
			point.scale.x = 0.2;
			point.scale.y = 0.2;
			point.scale.z = 0.2;
			point.color.a = 1.0;
			point.color.r = 1.0;
			point.color.g = 1.0;
			point.color.b = 1.0;
			++countNodeNum;
			bestPathMarkers.markers.push_back(point);

			if (i<this->bestPath_.size()-1){
				std::shared_ptr<PRM::Node> nNext = this->bestPath_[i+1];
				visualization_msgs::Marker line;
				line.ns = "best_path";
				line.header.frame_id = "map";
				line.type = visualization_msgs::Marker::LINE_LIST;
				line.header.stamp = ros::Time::now();
				geometry_msgs::Point p1, p2;
				p1.x = n->pos(0);
				p1.y = n->pos(1);
				p1.z = n->pos(2);
				p2.x = nNext->pos(0);
				p2.y = nNext->pos(1);
				p2.z = nNext->pos(2);				
				line.points.push_back(p1);
				line.points.push_back(p2);
				line.id = countLineNum;
				line.scale.x = 0.2;
				line.scale.y = 0.2;
				line.scale.z = 0.2;
				line.color.r = 1.0;
				line.color.g = 0.0;
				line.color.b = 0.0;
				line.color.a = 1.0;
				line.lifetime = ros::Duration(0.5);
				++countLineNum;
				bestPathMarkers.markers.push_back(line);				
			}
		}
		this->bestPathPub_.publish(bestPathMarkers);		
	}

	void DEP::publishFrontier(){
		visualization_msgs::MarkerArray frontierMarkers;
		int frontierRangeCount = 0;
		for (int i=0; i<int(this->frontierPointPairs_.size()); ++i){
			visualization_msgs::Marker range;

			Eigen::Vector3d p = this->frontierPointPairs_[i].first;
			double dist = this->frontierPointPairs_[i].second;

			range.header.frame_id = "map";
			range.header.stamp = ros::Time::now();
			range.ns = "frontier range";
			range.id = frontierRangeCount;
			range.type = visualization_msgs::Marker::SPHERE;
			range.action = visualization_msgs::Marker::ADD;
			range.pose.position.x = p(0);
			range.pose.position.y = p(1);
			range.pose.position.z = p(2);
			range.lifetime = ros::Duration(0.5);
			range.scale.x = dist;
			range.scale.y = dist;
			range.scale.z = 0.1;
			range.color.a = 0.4;
			range.color.r = 0.0;
			range.color.g = 0.0;
			range.color.b = 1.0;
			++frontierRangeCount;
			frontierMarkers.markers.push_back(range);			
		}
		this->frontierVisPub_.publish(frontierMarkers);
	}

}
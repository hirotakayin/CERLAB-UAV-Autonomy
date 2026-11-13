/*
*	File: dep.h
*	---------------
*   dynamic exploration planner header file
*/

#ifndef DEP_H
#define DEP_H

#include <map_manager/dynamicMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <global_planner/PRMKDTree.h>
#include <global_planner/PRMAstar.h>
#include <global_planner/utils.h>
#include <opencv2/opencv.hpp>
#include <array>
#include <atomic>
#include <map>
#include <set>


namespace globalPlanner{
	class DEP{
	private:
		std::string ns_;
		std::string hint_;

		ros::NodeHandle nh_;
		ros::Publisher roadmapPub_;
		ros::Publisher candidatePathPub_;
		ros::Publisher bestPathPub_;
		ros::Publisher frontierVisPub_;
		ros::Publisher tourPub_;
		ros::Subscriber odomSub_;
		ros::Timer visTimer_;

		nav_msgs::Odometry odom_;
		std::shared_ptr<mapManager::occMap> map_; 
		std::shared_ptr<PRM::KDTree> roadmap_;




		// parameters
		double vel_ = 1.0;
		double angularVel_ = 1.0;
		std::string odomTopic_;
		Eigen::Vector3d globalRegionMin_, globalRegionMax_;
		Eigen::Vector3d localRegionMin_, localRegionMax_;
		int localSampleThresh_;
		int globalSampleThresh_;
		int frontierSampleThresh_;
		double distThresh_;
		double safeDistXY_;
		double safeDistZ_;
		bool safeDistCheckUnknown_;
		double horizontalFOV_;
		double verticalFOV_;
		double dmin_;
		double dmax_;
		bool useLidarModel_; // sensor_type: lidar, or omni360 (which shares the lidar's full-sphere gain search)
		double lidarFovUp_, lidarFovDown_, lidarPitch_; // rad, elevation limits in the lidar frame and nose-down pitch
		// sensor_type: omni360, a 360 camera ranging by optic flow: sees the whole sphere except a cone below the drone,
		// out to D <= flowRange * speed * sin(angle from the travel heading); the candidate yaw is the travel heading
		bool useOmniModel_ {false};
		double omniBlindSin_; // sin of the lowest elevation seen: the drone blocks the cone straight down
		double omniFlowRange_; // s, eps / sigma_w: the same as flow_limited_cloud.py's flow_range_s
		double omniSpeed_; // m/s, cruise speed the gain assumes
		double omniMinSin_; // sin of the smallest angle from the travel heading that is ranged
		bool omniVisible(const Eigen::Vector3d& direction, double yaw);
		std::vector<Eigen::Matrix3d> yawLidarRotInv_; // lidar-from-world rotation for each candidate yaw
		double maxEdgeUpAngle_, maxEdgeDownAngle_; // rad, steepest roadmap edge in flight direction
		std::atomic<bool> staticEnvironment_ {false}; // no moving obstacles: any slope through fully known free space
		int nnNum_;
		int nnNumFrontier_;
		double maxConnectDist_;
		std::vector<double> yaws_;
		double minVoxelThresh_;
		int minCandidateNum_;
		int maxCandidateNum_;
		double updateDist_;
		double yawPenaltyWeight_;
		Eigen::Vector3d gainRegionMin_, gainRegionMax_; // only unknown voxels here count as gain and frontiers
		double bestScore_ = 0.0, bestGain_ = 0.0, bestPathTime_ = 0.0; // the chosen path, for logging
		int candidatePathNum_ = 0;
		double depthBias_; // goal score multiplier grows with depth in the gain region; 0 = plain DEP
		double revisitPenalty_; // goal score is divided by 1 + penalty * (earlier goals within 1 m); 0 = plain DEP
		std::vector<Eigen::Vector3d> pastGoals_;
		double gainBlockSize_; // m; > 0: gain counts unexplored blocks and unseen floor patches instead of unknown voxels (lidar model only)
		std::vector<Eigen::Vector3d> gainCells_; // block centres and floor patches left to see, rebuilt each plan
		// Tour: gain cells are binned into areas, each area gets a viewpoint, and the viewpoints are flown in a short order
		std::atomic<bool> tourEnabled_ {false}, tourReset_ {false};
		double tourBinSize_; // m, side of an area
		int tourMinCells_; // smaller leftovers are not worth a trip
		double tourMinShare_; // areas smaller than this share of the largest wait until the large ones are done
		int tourMaxVisits_; // viewpoints tried for an area before giving up on it
		std::vector<std::shared_ptr<PRM::Node>> tourStops_; // current order, for the dashboard
		std::vector<std::array<int, 3>> tourOrder_; // areas in that order, kept between plans so the order stays stable
		std::map<std::array<int, 3>, std::vector<Eigen::Vector3d>> tourUsedViews_;
		bool tourDone_ = false, tourPlanActive_ = false;
		double tourGoalYaw_ = 0.0;
		int tourCellsLeft_ = 0, tourGivenUp_ = 0;
		double maxUnknownGap_; // m, longest unknown stretch allowed on a roadmap edge; 0 = every voxel must be known free
		int reachableNodeNum_ = 0;
		double minGoalDist_; // m, goals closer than this are only used when nothing further has gain

		// data
		bool odomReceived_ = false;
		Eigen::Vector3d position_;
		double currYaw_;
		std::deque<Eigen::Vector3d> histTraj_; // historic trajectory for information gain update 
		// std::vector<std::shared_ptr<PRM::Node>> prmNodeVec_; // all nodes		
		std::unordered_set<std::shared_ptr<PRM::Node>> prmNodeVec_; // all nodes
		std::vector<std::shared_ptr<PRM::Node>> goalCandidates_;
		std::vector<std::vector<std::shared_ptr<PRM::Node>>> candidatePaths_;
		std::vector<std::shared_ptr<PRM::Node>> bestPath_;
		std::vector<std::pair<Eigen::Vector3d, double>> frontierPointPairs_;


	public:
		DEP(const ros::NodeHandle& nh);

		void setMap(const std::shared_ptr<mapManager::occMap>& map);
		void loadVelocity(double vel, double angularVel);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		bool makePlan();
		nav_msgs::Path getBestPath();
		void detectFrontierRegion(std::vector<std::pair<Eigen::Vector3d, double>>& frontierPointPairs);
		void buildRoadMap();
		void pruneNodes();
		void updateInformationGain();
		void getBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates);
		bool findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates,  std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths);
		void findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths, std::vector<std::shared_ptr<PRM::Node>>& bestPath);
		std::shared_ptr<PRM::Node> getConnectedNode(const Eigen::Vector3d& pos);
		bool findPathToPoint(const Eigen::Vector3d& goal, nav_msgs::Path& path);
		int getFrontierNum();
		bool setTourEnabled(bool enabled);
		bool isTourEnabled();
		void setStaticEnvironment(bool enabled);
		bool isStaticEnvironment();
		bool isTourDone();
		void resetTour();
		void getTourStats(int& cellsLeft, int& givenUp);
		bool isInLidarFov(const Eigen::Vector3d& direction, double yaw);
		void getRoadmapStats(int& nodeNum, int& reachableNum);
		void getBestPathScore(double& score, double& gain, double& pathTime, int& candidateNum);
		

		// callback functions
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		void visCB(const ros::TimerEvent&);



		// help function
		bool isPosValid(const Eigen::Vector3d& p);
		bool isPosValid(const Eigen::Vector3d& p, double safeDistXY, double safeDistZ);
		std::shared_ptr<PRM::Node> randomConfigBBox(const Eigen::Vector3d& minRegion, const Eigen::Vector3d& maxRegion);
		bool edgeAllowed(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
		bool sensorFOVCondition(const Eigen::Vector3d& sample, const Eigen::Vector3d& pos);
		bool lidarVisible(const Eigen::Vector3d& direction, int yawIdx);
		int calculateUnknown(const shared_ptr<PRM::Node>& n, std::unordered_map<double, int>& yawNumVoxels);
		bool useBlockGain();
		double gainRange();
		bool planTour();
		void publishTour();
		void updateGainCells();
		bool gainCellInSight(const Eigen::Vector3d& cell, const Eigen::Vector3d& pos);
		void addVisibleGainCells(const Eigen::Vector3d& pos, double yaw, std::unordered_set<int>& cells);
		double calculatePathLength(const std::vector<shared_ptr<PRM::Node>>& path);
		void shortcutPath(const std::vector<std::shared_ptr<PRM::Node>>& path, std::vector<std::shared_ptr<PRM::Node>>& pathSc);
		int weightedSample(const std::vector<double>& weights);
		std::shared_ptr<PRM::Node> sampleFrontierPoint(const std::vector<double>& sampleWeights);
		std::shared_ptr<PRM::Node> extendNode(const std::shared_ptr<PRM::Node>& n, const std::shared_ptr<PRM::Node>& target);

		// visualization functions
		void publishRoadmap();
		void publishCandidatePaths();
		void publishBestPath();
		void publishFrontier();
	};
}


#endif



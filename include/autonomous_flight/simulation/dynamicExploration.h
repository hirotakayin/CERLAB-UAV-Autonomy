/*
	FILE: dynamicExploration.h
	-----------------------------
	header of dynamic exploration
*/
#ifndef DYNAMIC_EXPLORATION
#define DYNAMIC_EXPLORATION
#include <autonomous_flight/simulation/flightBase.h>
#include <global_planner/dep.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <map_manager/dynamicMap.h>
#include <onboard_detector/fakeDetector.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <atomic>
#include <ctime>
#include <fstream>
#include <sys/stat.h>
#include <mutex>
#include <deque>
#include <array>
#include <limits>
#include <vector>


namespace AutoFlight{
	class dynamicExploration : flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<onboardDetector::fakeDetector> detector_;
		std::shared_ptr<globalPlanner::DEP> expPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer explorationTimer_;
		ros::Timer plannerTimer_;
		ros::Timer replanCheckTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;
		ros::Timer freeRobotRegionTimer_;

		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;
		ros::Publisher inputTrajPub_;
		
		// parameters
		bool useFakeDetector_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		double wpStablizeTime_;
		bool initialScan_;
		double replanTimeForDynamicObstacle_;
		Eigen::Vector3d freeRange_;
		double reachGoalDistance_;
		std::vector<Eigen::Vector4d> initialWaypoints_; // x, y, z, yaw (rad) flown before exploration starts
		int maxExplorationFailures_; // consecutive failed DEP plans before exploration counts as complete
		double inspectHoldTime_; // s, hover time at an inspection viewpoint
		double inspectDistance_; // m, viewpoint distance from the inspected point
		double landClearance_; // m, descend to this height above the mapped ground, then cut motors
		std::string landTopic_;
		Eigen::Vector3d progressRegionMin_, progressRegionMax_; // region used for the % explored figure

		// mission (GUI / services): every leg is a waypoint path flown by the trajectory planner
		enum MissionState {IDLE, TAKEOFF, NUDGE, EXPLORE, PAUSED, INSPECT, RETURN, LAND, LANDED};
		enum MissionCommand {CMD_NONE, CMD_RETURN_HOME, CMD_INSPECT};
		std::atomic<int> missionState_ {IDLE};
		int stateBeforePause_ = IDLE;
		nav_msgs::Path pausedRemainingPath_;
		std::atomic<int> pendingCommand_ {CMD_NONE}; // handled by the planning thread (it owns DEP)
		std::atomic<bool> startRequested_ {false};
		std::mutex pendingPathMutex_; // planning thread -> main thread handoff, never held while spinning
		nav_msgs::Path pendingPath_;
		int pendingPathState_ = IDLE;
		bool pendingPathReady_ = false;
		Eigen::Vector3d homePos_; // hover point above the takeoff spot
		double homeGroundZ_ = 0.0; // height of the takeoff spot
		Eigen::Vector3d inspectTarget_;
		int returnRetries_ = 0;
		ros::Time stuckSince_; // when the way home was given up; after stuckLandTime_ the drone lands where it is
		double stuckLandTime_ = 15.0;
		Eigen::Vector3d landTarget_;
		ros::Time touchdownCheckTime_;
		bool descentStarted_ = false; // touchdown only counts once the robot has actually been descending
		ros::Time missionStartTime_;
		ros::Time missionEndTime_; // set on touchdown so the mission clock stops
		double runLowestZ_ = std::numeric_limits<double>::infinity();
		std::vector<std::array<double, 3>> finishedRuns_; // elapsed, explored %, lowest z
		bool restartPending_ = false; // re-taking off after a landing, entry path follows
		double restartTargetZ_ = 0.0;
		std::mutex eventsMutex_;
		std::deque<std::pair<double, std::string>> events_; // (mission time, text) for the GUI
		double exploredPercent_ = 0.0;
		double floorZ_ = std::numeric_limits<double>::quiet_NaN(); // top of the floor, NaN = no floor figure
		std::atomic<double> floorPercent_ {0.0}; // floor cells hit at any time so far
		double floorNowPercent_ = 0.0; // floor cells occupied in the map right now: grazing rays clear them again
		std::vector<char> floorSeen_; // per floor cell of the progress region
		int floorCols_ = 0;
		std::mutex floorSeenMutex_;
		double completeMinGain_; // a plan adding less coverage than this (percentage points) counts as stalled
		int completeStallPlans_; // this many stalled plans in a row: exploration is done
		std::atomic<double> exploredVolume_ {0.0};
		struct PlanRecord { // one chosen exploration goal, for the GUI
			std::string kind; // explore or tour
			int run; double t;
			Eigen::Vector3d from, goal;
			double pathLength; int legs;
			double heading, turn; // deg: straight-line travel direction, change from the previous goal's
			double score, gain, pathTime; int candidates;
			double learned; // voxels newly known by the next plan, -1 until then
			double knownAtPlan;
			double coverage; // pit_block_pct + floor_pct when planned
			bool flown; // the robot had moved by the next plan
			double planSeconds;
			std::string ended; // why its flight ended: reached, or the reason it was dropped
			int tourCellsLeft, tourGivenUp; // tour plans: gain cells still to see, areas given up on
			int nodes, reachable; // roadmap size and how much of it connects to the robot
			int revisits; // earlier goals within 1 m in this run
		};
		std::vector<PlanRecord> planLog_;
		std::atomic<const char*> lastPathEnd_ {""};
		// peering (sensor:=omni): before each exploration plan, a short sideways sweep and back, so a camera ranging by
		// optic flow sees ahead (flow gives no range in the direction of travel). peer_amplitude 0 = off
		double peerAmplitude_ {0.0};
		std::atomic<bool> peering_ {false}; // the path being flown is a peer, not a plan
		bool peeredSincePlan_ {false};
		// hold_yaw (sensor:=omni): a 360 camera needs no turning, so never rotate to face a leg or a waypoint's heading.
		// Turning in place gives optic flow no depth, and costs time at zero speed
		bool holdYaw_ {false};
		double travelHeading_ {0.0}; // direction of the last leg, which peering sweeps across
		bool makePeerPath(nav_msgs::Path& path);
		double lastPlanSeconds_ = 0.0;
		ros::Publisher planLogPub_;
		std::atomic<int> frontierNum_ {-1};

		ros::ServiceServer startSrv_, pauseSrv_, resumeSrv_, returnHomeSrv_, landSrv_, tourSrv_;
		ros::ServiceServer staticSrv_;
		ros::Subscriber inspectSub_;
		ros::Publisher statusPub_, landPub_, takeoffPub_;
		ros::Publisher unknownPub_, floorSeenPub_;
		std::atomic<double> exploredBlockPercent_ {0.0}; // share of 0.6 m blocks with at least one known voxel
		double lastTargetZ_ = std::numeric_limits<double>::quiet_NaN(); // height the trajectory asks for
		struct RunSample {std::string state; double t, x, y, z, zTarget, pit, pitBlock, floor, floorNow, clearance, surface;};
		std::vector<RunSample> runSamples_; // written to ~/mission_runs on landing
		std::string preset_; // config name from the launch file, for the dashboard and run files
		std::atomic<double> surfacePercent_ {-1.0}; // surface voxels confirmed within inspect_range, of all hit
		double sampleClearance_ = std::numeric_limits<double>::infinity(); // closest mapped obstacle since the last sample
		double runClearance_ = std::numeric_limits<double>::infinity(); // and over the run, leaving out takeoff and landing
		std::string runSaveDir_;
		ros::Timer statusTimer_, restartTimer_, unknownTimer_, clearanceTimer_;
		std::string takeoffTopic_;

		// exploration data
		std::atomic<bool> explorationReplan_ {false};
		bool replan_ = false;
		bool newWaypoints_ = false;
		int waypointIdx_ = 1;
		nav_msgs::Path waypoints_; // latest waypoints from exploration planner
		nav_msgs::Path inputTrajMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
		bool trajectoryReady_ = false;
		ros::Time trajStartTime_;
		double trajTime_; // current trajectory time
		trajPlanner::bspline trajectory_;
		ros::Time lastDynamicObstacleTime_;
	
	public:
		std::thread exploreReplanWorker_;
		dynamicExploration();
		dynamicExploration(const ros::NodeHandle& nh);

		void initParam();
		void initModules();
		void registerCallback();
		void registerPub();

		void explorationCB(const ros::TimerEvent&);
		void plannerCB(const ros::TimerEvent&);
		void replanCheckCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&); // using fake detector
		void freeRobotRegionCB(const ros::TimerEvent&); // the robot's own volume is free space

		// mission interface
		void registerMissionInterface();
		bool startSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool pauseSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool resumeSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool returnHomeSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool landSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		void inspectCB(const geometry_msgs::PointStampedConstPtr& target);
		void statusCB(const ros::TimerEvent&);
		void unknownCB(const ros::TimerEvent&);
		void clearanceCB(const ros::TimerEvent&);
		void saveRun();
		void publishCloud(ros::Publisher& pub, const std::vector<Eigen::Vector3d>& points);
		bool isFloorInspected(double x, double y);
		bool tourSrvCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
		bool staticSrvCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
		bool isFloorHit(double x, double y);
		void restartMission();
		void restartCheckCB(const ros::TimerEvent&);
		nav_msgs::Path makeEntryPath();
		void onPathFinished(bool success, const char* reason="reached");
		void applyPendingPath();
		void setPendingPath(const nav_msgs::Path& path, int state);
		void flyPath(const nav_msgs::Path& path, int state);
		void startLanding();
		void touchdown();
		bool findInspectViewpoint(const Eigen::Vector3d& target, Eigen::Vector3d& viewpoint);
		nav_msgs::Path makePath(const std::vector<Eigen::Vector3d>& points, double finalYaw);
		void recordPlan(const nav_msgs::Path& path, const std::string& kind="explore");
		void logEvent(const std::string& text);
		static std::string stateName(int state);
		Eigen::Vector3d currPos();
		double currYaw();

		void run();
		void initExplore();
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions);
		bool hasCollision();
		bool hasDynamicCollision();
		void exploreReplan();
		double computeExecutionDistance();
		bool replanForDynamicObstacle();
		bool reachExplorationGoal();
		bool isGoalValid();
		nav_msgs::Path getCurrentTraj(double dt);
		nav_msgs::Path getRestGlobalPath();
		nav_msgs::Path getRestGlobalPath(const Eigen::Vector3d& pos);
		nav_msgs::Path getRestGlobalPath(const Eigen::Vector3d& pos, double yaw);
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize);
		void waitTime(double time);
	};
}
#endif
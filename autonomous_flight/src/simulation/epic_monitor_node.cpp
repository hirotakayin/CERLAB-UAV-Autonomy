/*
	FILE: epic_monitor_node.cpp
	-----------------------------
	Scores an EPIC flight like the CERLAB presets: runs the same occupancy map on EPIC's lidar and odometry,
	publishes the dashboard's status and writes the same CSV and picture when the drone lands
*/

#include <autonomous_flight/simulation/terminalRelay.h>
#include <map_manager/occupancyMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <sys/stat.h>
#include <vector>

namespace{
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

	struct RunSample {std::string state; double t, x, y, z, pit, pitBlock, floor, floorNow, clearance, surface;};
	struct PlanRecord {double t; Eigen::Vector3d from, goal; double pathLength, pathTime, planTime, turn; int legs, revisits;};

	std::vector<double> jsonNumbers(const std::string& text, const std::string& key){
		// the plan message is flat JSON from epic_mission.py: a number or a list of numbers per key
		std::vector<double> values;
		size_t at = text.find("\"" + key + "\":");
		if (at == std::string::npos) return values;
		at = text.find_first_of("-0123456789", at + key.size() + 3);
		if (at == std::string::npos) return values;
		size_t end = text[at - 1] == '[' ? text.find(']', at) : text.find_first_of(",}", at);
		std::string list = text.substr(at, end - at);
		std::replace(list.begin(), list.end(), ',', ' ');
		std::istringstream stream (list);
		double value;
		while (stream >> value) values.push_back(value);
		return values;
	}
}

class epicMonitor{
private:
	ros::NodeHandle nh_;
	std::shared_ptr<mapManager::occMap> map_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> CloudOdomPolicy;
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloudSub_;
	std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomFilterSub_;
	std::shared_ptr<message_filters::Synchronizer<CloudOdomPolicy>> cloudOdomSync_;
	ros::Publisher cloudPub_, statusPub_, planLogPub_, posePub_, unknownPub_;
	ros::Subscriber stateSub_, eventSub_, planSub_;
	ros::Timer statusTimer_, unknownTimer_, clearanceTimer_;

	Eigen::Matrix4d body2Lidar_;
	Eigen::Vector3d regionMin_, regionMax_, home_ = Eigen::Vector3d (6.0, 2.0, 0.2), position_ = Eigen::Vector3d::Zero();
	double yaw_ = 0.0, floorZ_ = std::numeric_limits<double>::quiet_NaN();
	double pit_ = 0.0, pitBlock_ = 0.0, floor_ = 0.0, floorNow_ = 0.0, surface_ = -1.0, volume_ = 0.0;
	double sampleClearance_ = std::numeric_limits<double>::infinity(), runClearance_ = std::numeric_limits<double>::infinity();
	std::vector<char> floorSeen_;
	std::string state_ = "IDLE", preset_, saveDir_;
	ros::Time startTime_, endTime_;
	std::vector<RunSample> samples_;
	std::vector<PlanRecord> plans_;
	std::vector<std::pair<double, std::string>> events_;
	bool saved_ = false;

	double elapsed(){
		if (this->startTime_.isZero()) return 0.0;
		return ((this->endTime_.isZero() ? ros::Time::now() : this->endTime_) - this->startTime_).toSec();
	}

	void cloudOdomCB(const sensor_msgs::PointCloud2ConstPtr& cloud, const nav_msgs::OdometryConstPtr& odom){
		// EPIC's simulator publishes the cloud in the world frame, the map expects the lidar frame
		const geometry_msgs::Pose& pose = odom->pose.pose;
		Eigen::Matrix4d world2Body = Eigen::Matrix4d::Identity();
		world2Body.block<3, 3>(0, 0) = Eigen::Quaterniond (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
		world2Body.block<3, 1>(0, 3) = Eigen::Vector3d (pose.position.x, pose.position.y, pose.position.z);
		Eigen::Matrix4d toLidar = (world2Body * this->body2Lidar_).inverse();
		sensor_msgs::PointCloud2 out = *cloud;
		out.header.frame_id = "lidar_link";
		sensor_msgs::PointCloud2Iterator<float> x (out, "x"), y (out, "y"), z (out, "z");
		for (; x != x.end(); ++x, ++y, ++z){
			Eigen::Vector4d p = toLidar * Eigen::Vector4d (*x, *y, *z, 1.0);
			*x = float(p(0));
			*y = float(p(1));
			*z = float(p(2));
		}
		this->cloudPub_.publish(out);
	}

	void odomCB(const nav_msgs::OdometryConstPtr& odom){
		const geometry_msgs::Pose& pose = odom->pose.pose;
		this->position_ = Eigen::Vector3d (pose.position.x, pose.position.y, pose.position.z);
		Eigen::Vector3d forward = Eigen::Quaterniond (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix().col(0);
		this->yaw_ = atan2(forward(1), forward(0));
		geometry_msgs::PoseStamped msg;
		msg.header.stamp = odom->header.stamp;
		msg.header.frame_id = "map";
		msg.pose = pose;
		this->posePub_.publish(msg);
	}

	void stateCB(const std_msgs::StringConstPtr& msg){
		if (this->state_ == "IDLE" and msg->data != "IDLE") this->startTime_ = ros::Time::now();
		this->state_ = msg->data;
		if (this->state_ == "LANDED" and not this->saved_){
			this->endTime_ = ros::Time::now();
			this->saved_ = true;
			this->saveRun();
		}
	}

	void eventCB(const std_msgs::StringConstPtr& msg){
		this->logEvent(msg->data);
	}

	void logEvent(const std::string& text){
		this->events_.push_back({this->elapsed(), text});
		cout << "[epic_monitor]: " << text << endl;
	}

	void planCB(const std_msgs::StringConstPtr& msg){
		std::vector<double> goal = jsonNumbers(msg->data, "goal"), from = jsonNumbers(msg->data, "from");
		std::vector<double> length = jsonNumbers(msg->data, "path_length"), legs = jsonNumbers(msg->data, "legs"), pathTime = jsonNumbers(msg->data, "path_time"), planTime = jsonNumbers(msg->data, "plan_time");
		if (goal.size() != 3 or from.size() != 3 or length.empty() or legs.empty() or pathTime.empty() or planTime.empty()) return;
		PlanRecord r;
		r.t = this->elapsed();
		r.from = Eigen::Vector3d (from[0], from[1], from[2]);
		r.goal = Eigen::Vector3d (goal[0], goal[1], goal[2]);
		r.pathLength = length[0];
		r.legs = int(legs[0]);
		r.pathTime = pathTime[0];
		r.planTime = planTime[0];
		r.turn = 0.0;
		if (not this->plans_.empty()){
			Eigen::Vector3d before = this->plans_.back().goal - this->plans_.back().from, now = r.goal - r.from;
			if (before.norm() > 1e-3 and now.norm() > 1e-3) r.turn = acos(std::max(-1.0, std::min(1.0, before.dot(now) / (before.norm() * now.norm())))) * 180.0 / M_PI;
		}
		r.revisits = 0;
		for (const PlanRecord& p : this->plans_){
			if ((p.goal - r.goal).norm() < 1.0) ++r.revisits;
		}
		this->plans_.push_back(r);

		std::ostringstream json;
		json << std::fixed << std::setprecision(2) << "[";
		for (size_t i=0; i<this->plans_.size(); ++i){
			const PlanRecord& p = this->plans_[i];
			json << (i == 0 ? "" : ",") << "{\"kind\":\"epic\",\"run\":1,\"t\":" << p.t
				<< ",\"from\":[" << p.from(0) << "," << p.from(1) << "," << p.from(2) << "]"
				<< ",\"goal\":[" << p.goal(0) << "," << p.goal(1) << "," << p.goal(2) << "]"
				<< ",\"path_length\":" << p.pathLength << ",\"legs\":" << p.legs << ",\"turn\":" << p.turn
				<< ",\"score\":0,\"gain\":0,\"path_time\":" << p.pathTime
				<< ",\"candidates\":0,\"learned\":-1,\"revisits\":" << p.revisits << ",\"nodes\":0,\"reachable\":0}";
		}
		json << "]";
		std_msgs::String out;
		out.data = json.str();
		this->planLogPub_.publish(out);
	}

	bool isFloorInspected(double x, double y){
		for (double z=this->floorZ_-0.3; z<=this->floorZ_+0.3; z+=this->map_->getRes()){
			if (this->map_->isInspectedHit(Eigen::Vector3d (x, y, z))) return true;
		}
		return false;
	}

	bool isFloorHit(double x, double y){
		for (double z=this->floorZ_-0.3; z<=this->floorZ_+0.3; z+=this->map_->getRes()){
			Eigen::Vector3d p (x, y, z);
			if (this->map_->isInMap(p) and this->map_->isOccupied(p)) return true;
		}
		return false;
	}

	void clearanceCB(const ros::TimerEvent&){
		if (this->startTime_.isZero() or this->state_ == "LANDED") return;
		double res = this->map_->getRes(), clearance = std::numeric_limits<double>::infinity();
		int range = int(std::ceil(1.5 / res));
		for (int i=-range; i<=range; ++i){
			for (int j=-range; j<=range; ++j){
				for (int k=-range; k<=range; ++k){
					Eigen::Vector3d offset (i * res, j * res, k * res);
					Eigen::Vector3d q = this->position_ + offset;
					if (not this->map_->isInMap(q) or not this->map_->isOccupied(q)) continue;
					Eigen::Vector3d gap = (offset.cwiseAbs().array() - res/2).max(0.0).matrix();
					clearance = std::min(clearance, gap.norm());
				}
			}
		}
		this->sampleClearance_ = std::min(this->sampleClearance_, clearance);
		if (this->state_ != "TAKEOFF" and this->state_ != "LANDING") this->runClearance_ = std::min(this->runClearance_, clearance);
	}

	void unknownCB(const ros::TimerEvent&){
		double res = this->map_->getRes(), block = 3.0 * res;
		std::vector<Eigen::Vector3d> blocks;
		long total = 0;
		for (double x=this->regionMin_(0)+block/2; x<this->regionMax_(0); x+=block){
			for (double y=this->regionMin_(1)+block/2; y<this->regionMax_(1); y+=block){
				for (double z=this->regionMin_(2)+block/2; z<this->regionMax_(2); z+=block){
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
		if (total != 0) this->pitBlock_ = 100.0 * double(total - long(blocks.size())) / double(total);

		if (this->map_->getInspectRange() > 0.0){
			long confirmed = 0, pendingVoxels = 0;
			for (double x=this->regionMin_(0); x<this->regionMax_(0); x+=res){
				for (double y=this->regionMin_(1); y<this->regionMax_(1); y+=res){
					for (double z=this->regionMin_(2)-block; z<this->regionMax_(2); z+=res){
						Eigen::Vector3d p (x, y, z);
						if (this->map_->isInspectedHit(p)) ++confirmed;
						else if (this->map_->isInspectionPending(p)) ++pendingVoxels;
					}
				}
			}
			if (confirmed + pendingVoxels != 0) this->surface_ = 100.0 * double(confirmed) / double(confirmed + pendingVoxels);
		}
		if (this->unknownPub_.getNumSubscribers() != 0){
			pcl::PointCloud<pcl::PointXYZ> cloud;
			for (const Eigen::Vector3d& p : blocks) cloud.push_back(pcl::PointXYZ (p(0), p(1), p(2)));
			sensor_msgs::PointCloud2 msg;
			pcl::toROSMsg(cloud, msg);
			msg.header.frame_id = "map";
			msg.header.stamp = ros::Time::now();
			this->unknownPub_.publish(msg);
		}
	}

	void statusCB(const ros::TimerEvent&){
		double res = this->map_->getRes();
		long known = 0, mapped = 0;
		for (double x=this->regionMin_(0)+res/2; x<this->regionMax_(0); x+=res){
			for (double y=this->regionMin_(1)+res/2; y<this->regionMax_(1); y+=res){
				for (double z=this->regionMin_(2)+res/2; z<this->regionMax_(2); z+=res){
					Eigen::Vector3d p (x, y, z);
					if (not this->map_->isInMap(p)) continue;
					++mapped;
					if (not this->map_->isUnknown(p)) ++known;
				}
			}
		}
		this->pit_ = mapped == 0 ? 0.0 : 100.0 * double(known) / double(mapped);
		this->volume_ = double(known) * pow(res, 3);

		Eigen::Vector3d size = this->regionMax_ - this->regionMin_;
		int cols = int(std::ceil(size(0)/res)), rows = int(std::ceil(size(1)/res));
		long seen = 0, now = 0;
		if (this->floorSeen_.empty()) this->floorSeen_.assign(cols * rows, 0);
		if (not std::isnan(this->floorZ_)){
			for (int i=0; i<cols; ++i){
				for (int j=0; j<rows; ++j){
					double cellX = this->regionMin_(0) + (i + 0.5) * res, cellY = this->regionMin_(1) + (j + 0.5) * res;
					if (this->isFloorHit(cellX, cellY)) ++now;
					if (this->isFloorInspected(cellX, cellY)) this->floorSeen_[i * rows + j] = 1;
					if (this->floorSeen_[i * rows + j]) ++seen;
				}
			}
			this->floor_ = 100.0 * double(seen) / double(cols * rows);
			this->floorNow_ = 100.0 * double(now) / double(cols * rows);
		}

		if (not this->startTime_.isZero() and this->state_ != "LANDED"){
			this->samples_.push_back({this->state_, this->elapsed(), this->position_(0), this->position_(1), this->position_(2), this->pit_, this->pitBlock_, this->floor_, this->floorNow_, this->sampleClearance_, this->surface_});
			this->sampleClearance_ = std::numeric_limits<double>::infinity();
		}

		std::ostringstream json;
		json << std::fixed << std::setprecision(2);
		json << "{\"state\":\"" << this->state_ << "\",\"paused_state\":\"\",\"elapsed\":" << this->elapsed() << ",\"run\":1,\"finished_runs\":[]";
		json << ",\"explored_pct\":" << this->pit_ << ",\"floor_pct\":" << (std::isnan(this->floorZ_) ? -1.0 : this->floor_) << ",\"explored_block_pct\":" << this->pitBlock_;
		json << ",\"z_target\":null,\"floor_now_pct\":" << (std::isnan(this->floorZ_) ? -1.0 : this->floorNow_) << ",\"inspect_range\":" << this->map_->getInspectRange();
		json << ",\"preset\":\"" << this->preset_ << "\",\"surface_pct\":" << this->surface_ << ",\"tour\":false,\"explored_m3\":" << this->volume_ << ",\"frontiers\":0,\"map_res\":" << res;
		json << ",\"position\":[" << this->position_(0) << "," << this->position_(1) << "," << this->position_(2) << "],\"yaw\":" << this->yaw_;
		json << ",\"home\":[" << this->home_(0) << "," << this->home_(1) << "," << this->home_(2) << "]";
		json << ",\"region_min\":[" << this->regionMin_(0) << "," << this->regionMin_(1) << "," << this->regionMin_(2) << "]";
		json << ",\"region_max\":[" << this->regionMax_(0) << "," << this->regionMax_(1) << "," << this->regionMax_(2) << "]";
		json << ",\"events\":[";
		for (size_t i=0; i<this->events_.size(); ++i){
			json << (i == 0 ? "" : ",") << "{\"t\":" << this->events_[i].first << ",\"text\":\"" << this->events_[i].second << "\"}";
		}
		json << "]}";
		std_msgs::String msg;
		msg.data = json.str();
		this->statusPub_.publish(msg);
	}

	void saveRun(){
		if (this->samples_.empty()) return;
		if (this->saveDir_.empty()){
			const char* home = getenv("HOME");
			this->saveDir_ = std::string(home == NULL ? "." : home) + "/mission_runs";
			mkdir(this->saveDir_.c_str(), 0755);
		}
		char stamp[32];
		std::time_t now = std::time(NULL);
		std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
		std::string base = this->saveDir_ + "/" + stamp + "_" + this->preset_ + "_run1";

		std::ofstream csv (base + ".csv");
		csv << std::fixed << std::setprecision(2);
		csv << "run,t_s,type,state,x_m,y_m,z_m,z_target_m,pit_pct,pit_block_pct,floor_pct,floor_now_pct,kind,n,goal_x,goal_y,goal_z,dist,path_length,legs,turn_deg,score,predicted_voxels,learned_voxels,path_time_s,candidates,revisits,roadmap_nodes,reachable_nodes,plan_time_s,flown,plan_end,tour_cells_left,tour_areas_given_up,min_clearance_m,surface_pct\n";
		std::vector<std::pair<double, std::string>> rows;
		for (const RunSample& r : this->samples_){
			std::ostringstream row;
			row << std::fixed << std::setprecision(2) << "1," << r.t << ",height," << r.state << "," << r.x << "," << r.y << "," << r.z << ",," << r.pit << "," << r.pitBlock << "," << r.floor << "," << r.floorNow << ",,,,,,,,,,,,,,,,,,,,,,,";
			if (not std::isinf(r.clearance)) row << r.clearance;
			row << ",";
			if (r.surface >= 0.0) row << r.surface;
			rows.push_back({r.t, row.str()});
		}
		std::vector<Eigen::Vector3d> goals;
		for (size_t i=0; i<this->plans_.size(); ++i){
			const PlanRecord& p = this->plans_[i];
			goals.push_back(p.goal);
			std::ostringstream row;
			// score, predicted voxels, candidates and roadmap size do not exist in EPIC: left empty. A goal counts as flown once chosen
			row << std::fixed << std::setprecision(2) << "1," << p.t << ",waypoint,,,,,,,,,,epic," << i + 1 << "," << p.goal(0) << "," << p.goal(1) << "," << p.goal(2) << "," << (p.goal - p.from).norm() << "," << p.pathLength << "," << p.legs << "," << p.turn << ",,,," << p.pathTime << ",," << p.revisits << ",,," << p.planTime << ",1,,,,,";
			rows.push_back({p.t, row.str()});
		}
		std::stable_sort(rows.begin(), rows.end(), [](const std::pair<double, std::string>& a, const std::pair<double, std::string>& b){return a.first < b.first;});
		for (const std::pair<double, std::string>& row : rows) csv << row.second << "\n";

		Eigen::Vector3d lo = this->regionMin_, hi = this->regionMax_;
		double lowest = std::numeric_limits<double>::infinity();
		for (const RunSample& r : this->samples_){
			lo = lo.cwiseMin(Eigen::Vector3d (r.x, r.y, r.z));
			hi = hi.cwiseMax(Eigen::Vector3d (r.x, r.y, r.z));
			lowest = std::min(lowest, r.z);
		}
		lo = lo.cwiseMin(this->home_) - Eigen::Vector3d (1, 1, 1);
		hi = hi.cwiseMax(this->home_) + Eigen::Vector3d (1, 1, 1);
		double scale = 40.0, margin = 50.0, topY = 90.0;
		double width = (hi(1) - lo(1)) * scale;
		double xyHeight = (hi(0) - lo(0)) * scale, xzHeight = (hi(2) - lo(2)) * scale;
		auto sx = [&](double y){return margin + (y - lo(1)) * scale;};
		auto syTop = [&](double x){return topY + (hi(0) - x) * scale;};
		double sideTop = topY + xyHeight + 70.0;
		auto sySide = [&](double z){return sideTop + (hi(2) - z) * scale;};
		std::string worldSdf;
		this->nh_.getParam("autonomous_flight/world_sdf", worldSdf);
		std::vector<Eigen::Vector3d> flown;
		for (const RunSample& r : this->samples_) flown.push_back(Eigen::Vector3d (r.x, r.y, r.z));
		double orthoTop = sideTop + xzHeight + 70.0, orthoHeight = 0.0;
		std::string ortho = orthoView(worldSdf, flown, goals, this->home_, margin, orthoTop, width, orthoHeight);
		std::ofstream svg (base + ".svg");
		svg << std::fixed << std::setprecision(1);
		svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width + 2 * margin << "' height='" << orthoTop + orthoHeight + 40 << "' font-family='sans-serif' font-size='14'>\n";
		svg << "<rect width='100%' height='100%' fill='white'/>\n";
		svg << "<text x='" << margin << "' y='28' font-size='18' font-weight='bold'>Run 1, " << stamp << ", " << this->preset_ << "</text>\n";
		svg << "<text x='" << margin << "' y='52'>" << this->samples_.back().t << " s, pit " << this->pitBlock_ << "% (voxels " << this->pit_ << "%), floor " << this->floor_ << "%, lowest z " << lowest << " m, closest obstacle ";
		if (std::isinf(this->runClearance_)) svg << "over 1.5"; else svg << std::setprecision(2) << this->runClearance_ << std::setprecision(1);
		svg << " m, " << goals.size() << " goals</text>\n";
		svg << "<text x='" << margin << "' y='" << topY - 8 << "' font-weight='bold'>From above (horizontal y, vertical x)</text>\n";
		svg << "<text x='" << margin << "' y='" << sideTop - 8 << "' font-weight='bold'>From the side (horizontal y, vertical z)</text>\n";
		svg << "<rect x='" << sx(this->regionMin_(1)) << "' y='" << syTop(this->regionMax_(0)) << "' width='" << (this->regionMax_(1) - this->regionMin_(1)) * scale << "' height='" << (this->regionMax_(0) - this->regionMin_(0)) * scale << "' fill='#E0E0E0' stroke='#6D6E71'/>\n";
		svg << "<rect x='" << sx(this->regionMin_(1)) << "' y='" << sySide(this->regionMax_(2)) << "' width='" << (this->regionMax_(1) - this->regionMin_(1)) * scale << "' height='" << (this->regionMax_(2) - this->regionMin_(2)) * scale << "' fill='#E0E0E0' stroke='#6D6E71'/>\n";
		svg << "<polyline fill='none' stroke='#C41230' stroke-width='1.5' points='";
		for (const RunSample& r : this->samples_) svg << sx(r.y) << "," << syTop(r.x) << " ";
		svg << "'/>\n<polyline fill='none' stroke='#C41230' stroke-width='1.5' points='";
		for (const RunSample& r : this->samples_) svg << sx(r.y) << "," << sySide(r.z) << " ";
		svg << "'/>\n";
		for (const Eigen::Vector3d& g : goals){
			svg << "<circle cx='" << sx(g(1)) << "' cy='" << syTop(g(0)) << "' r='3' fill='#000'/><circle cx='" << sx(g(1)) << "' cy='" << sySide(g(2)) << "' r='3' fill='#000'/>\n";
		}
		svg << "<circle cx='" << sx(this->home_(1)) << "' cy='" << syTop(this->home_(0)) << "' r='8' fill='none' stroke='#000' stroke-width='2'/><circle cx='" << sx(this->home_(1)) << "' cy='" << sySide(this->home_(2)) << "' r='8' fill='none' stroke='#000' stroke-width='2'/>\n";
		svg << "<text x='" << margin << "' y='" << orthoTop - 8 << "' font-weight='bold'>Orthographic view (from +x, -y, 30 deg above)</text>\n" << ortho;
		svg << "<text x='" << margin << "' y='" << orthoTop + orthoHeight + 24 << "' fill='#6D6E71'>Red: flown path. Black dots: chosen goals. Ring: home. Gray: pit interior, and pit walls, columns and beams in the orthographic view.</text>\n</svg>\n";
		this->logEvent("Saved this run: " + base + ".svg and .csv");
	}

public:
	// a run stopped before landing (Ctrl+C, a crash of the planner) still leaves its CSV and picture
	void saveIfUnsaved(){
		if (this->saved_ or this->samples_.empty()) return;
		this->saved_ = true;
		this->endTime_ = ros::Time::now();
		this->logEvent("Stopped before landing (" + this->state_ + "): saving what was logged");
		this->saveRun();
	}

	epicMonitor(const ros::NodeHandle& nh) : nh_(nh){
		std::vector<double> regionMin, regionMax, body2Lidar;
		this->nh_.getParam("autonomous_flight/progress_region_min", regionMin);
		this->nh_.getParam("autonomous_flight/progress_region_max", regionMax);
		this->nh_.getParam("occupancy_map/body_to_camera", body2Lidar);
		if (regionMin.size() != 3 or regionMax.size() != 3 or body2Lidar.size() != 16){
			ROS_FATAL("[epic_monitor]: progress region or body_to_camera missing");
			ros::shutdown();
			return;
		}
		this->regionMin_ = Eigen::Vector3d (regionMin[0], regionMin[1], regionMin[2]);
		this->regionMax_ = Eigen::Vector3d (regionMax[0], regionMax[1], regionMax[2]);
		for (int i=0; i<4; ++i){
			for (int j=0; j<4; ++j) this->body2Lidar_(i, j) = body2Lidar[i * 4 + j];
		}
		double floorZ;
		if (this->nh_.getParam("autonomous_flight/floor_z", floorZ)) this->floorZ_ = floorZ;
		this->nh_.param<std::string>("autonomous_flight/preset", this->preset_, "epic");
		this->nh_.param<std::string>("autonomous_flight/run_save_dir", this->saveDir_, "");
		std::vector<double> home;
		if (this->nh_.getParam("epic_mission/home", home) and home.size() == 3) this->home_ = Eigen::Vector3d (home[0], home[1], home[2]);

		this->cloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("/epic_monitor/cloud_lidar", 5);
		this->statusPub_ = this->nh_.advertise<std_msgs::String>("/exploration/status", 10);
		this->planLogPub_ = this->nh_.advertise<std_msgs::String>("/exploration/plan_log", 1, true);
		this->posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/CERLAB/quadcopter/pose", 10);
		this->unknownPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("/exploration/unknown_voxels", 1);
		this->map_.reset(new mapManager::occMap (this->nh_));

		this->cloudSub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(this->nh_, "/quad0_pcl_render_node/cloud", 5));
		this->odomFilterSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, "/quad_0/lidar_slam/odom", 25));
		this->cloudOdomSync_.reset(new message_filters::Synchronizer<CloudOdomPolicy>(CloudOdomPolicy(100), *this->cloudSub_, *this->odomFilterSub_));
		this->cloudOdomSync_->registerCallback(boost::bind(&epicMonitor::cloudOdomCB, this, _1, _2));
		this->odomFilterSub_->registerCallback(&epicMonitor::odomCB, this);

		this->stateSub_ = this->nh_.subscribe("/epic_mission/state", 10, &epicMonitor::stateCB, this);
		this->eventSub_ = this->nh_.subscribe("/epic_mission/event", 50, &epicMonitor::eventCB, this);
		this->planSub_ = this->nh_.subscribe("/epic_mission/plan", 50, &epicMonitor::planCB, this);
		this->unknownTimer_ = this->nh_.createTimer(ros::Duration(1.0), &epicMonitor::unknownCB, this);
		this->statusTimer_ = this->nh_.createTimer(ros::Duration(0.5), &epicMonitor::statusCB, this);
		this->clearanceTimer_ = this->nh_.createTimer(ros::Duration(0.1), &epicMonitor::clearanceCB, this);
	}
};

int main(int argc, char** argv){
	AutoFlight::terminalRelay relay;
	relay.capture();
	ros::init(argc, argv, "epic_monitor_node");
	ros::NodeHandle nh;
	relay.advertise(nh, "/exploration/terminal");
	epicMonitor monitor (nh);
	ros::spin();
	monitor.saveIfUnsaved();
	return 0;
}

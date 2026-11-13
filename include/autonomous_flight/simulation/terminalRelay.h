/*
	FILE: terminalRelay.h
	-----------------------------
	republish this process's stdout and stderr on a ROS topic, still printing them to the terminal
*/
#ifndef TERMINAL_RELAY
#define TERMINAL_RELAY
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace AutoFlight{
	class terminalRelay{
	private:
		static constexpr size_t maxHistory_ = 2000;
		std::mutex mutex_;
		std::deque<std::string> history_; // sent whole when a viewer connects
		std::string pending_; // new lines since the last publish
		ros::Publisher pub_;
		ros::Timer timer_;
		uint32_t lastSubscribers_ = 0;

		void readLoop(int readFd, int terminalFd){
			char buffer[4096];
			std::string partial;
			ssize_t n;
			while ((n = read(readFd, buffer, sizeof(buffer))) > 0){
				if (write(terminalFd, buffer, n) < 0){} // the terminal going away must not stop the node
				partial.append(buffer, n);
				size_t end;
				while ((end = partial.find('\n')) != std::string::npos){
					std::lock_guard<std::mutex> lock (this->mutex_);
					this->history_.push_back(partial.substr(0, end));
					while (this->history_.size() > maxHistory_) this->history_.pop_front();
					this->pending_ += partial.substr(0, end + 1);
					partial.erase(0, end + 1);
				}
			}
		}

		void publishCB(const ros::TimerEvent&){
			std_msgs::String msg;
			uint32_t subscribers = this->pub_.getNumSubscribers();
			{
				std::lock_guard<std::mutex> lock (this->mutex_);
				if (subscribers > this->lastSubscribers_){
					for (const std::string& line : this->history_) msg.data += line + "\n";
				}
				else{
					msg.data = this->pending_;
				}
				this->pending_.clear();
			}
			this->lastSubscribers_ = subscribers;
			if (subscribers != 0 and not msg.data.empty()) this->pub_.publish(msg);
		}

	public:
		void capture(){
			int fds[2];
			if (pipe(fds) != 0) return; // keep the normal terminal output
			int terminalFd = dup(STDOUT_FILENO);
			setvbuf(stdout, NULL, _IOLBF, 0); // a pipe would otherwise hold printf output back
			dup2(fds[1], STDOUT_FILENO);
			dup2(fds[1], STDERR_FILENO);
			close(fds[1]);
			std::thread(&terminalRelay::readLoop, this, fds[0], terminalFd).detach();
		}

		void advertise(ros::NodeHandle& nh, const std::string& topic){
			this->pub_ = nh.advertise<std_msgs::String>(topic, 10);
			this->timer_ = nh.createTimer(ros::Duration(0.2), &terminalRelay::publishCB, this);
		}
	};
}
#endif

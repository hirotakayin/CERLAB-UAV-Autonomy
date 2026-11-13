/*
	FILE: dynamic_exploration_node.cpp
	-----------------------------
	dynamic exploration ROS node
*/

#include <autonomous_flight/simulation/dynamicExploration.h>
#include <autonomous_flight/simulation/terminalRelay.h>

int main(int argc, char** argv){
	AutoFlight::terminalRelay relay;
	relay.capture(); // before anything prints, so the mission page gets the whole log
	ros::init(argc, argv, "dynamic_exploration_node");
	ros::NodeHandle nh;
	relay.advertise(nh, "/exploration/terminal");
	AutoFlight::dynamicExploration explorer (nh);
	explorer.run();

	ros::spin();

	return 0;
}
/*
	FILE: PRMAstar.h
	------------------------
	A* path search for PRM
*/

#ifndef PRM_ASTAR_H
#define PRM_ASTAR_H

#include <queue>
#include <functional>
#include <global_planner/PRMKDTree.h>

namespace PRM{
	bool inClose(std::shared_ptr<Node> n, const std::unordered_set<std::shared_ptr<Node>>& close){
		std::unordered_set<std::shared_ptr<Node>>::const_iterator got = close.find(n);
		return not (got == close.end());
	}

	// line check tolerating short unknown gaps: lidar rays near the edge of the FOV are sparse, so known-free
	// space has small holes that a strict check treats as walls. Never passes an occupied (inflated) voxel
	inline bool isClearLine(const std::shared_ptr<mapManager::occMap>& map, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double maxUnknownGap){
		if (maxUnknownGap <= 0.0) return map->isInflatedFreeLine(p1, p2);
		double dist = (p2 - p1).norm(), res = map->getRes(), unknownRun = 0.0;
		int stepNum = int(dist/res) + 1;
		for (int i=0; i<=stepNum; ++i){
			Eigen::Vector3d p = p1 + (p2 - p1) * double(i)/double(stepNum);
			if (map->isInflatedOccupied(p)) return false;
			unknownRun = map->isUnknown(p) ? unknownRun + dist/stepNum : 0.0;
			if (unknownRun > maxUnknownGap) return false;
		}
		return true;
	}

	std::vector<std::shared_ptr<Node>> AStar(const std::shared_ptr<KDTree>& roadmap,
										     const std::shared_ptr<Node>& start,
										     const std::shared_ptr<Node>& goal,
										     const std::shared_ptr<mapManager::occMap>& map,
										     double maxUnknownGap = 0.0,
										     const std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)>& edgeAllowed = nullptr){
		std::vector<std::shared_ptr<Node>> path;

		// open: priority queue
		std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> open;
		start->g = 0.0;
		open.push(start);

		// close: unordered set
		std::unordered_set<std::shared_ptr<Node>> close;

		// record all nodes involved in the path search (need to clear data afterwards)
		std::vector<std::shared_ptr<Node>> record;
		record.push_back(start);
		bool findPath = false;
		while (ros::ok()){
			if (inClose(goal, close)){
				findPath = true;
				break;
			}

			if (open.size() == 0){
				findPath = false;
				cout << "[Astar]: No valid path." << endl;
				break;
			}

			std::shared_ptr<Node> currNode = open.top();
			open.pop();

			// remove duplicates in open
			if (inClose(currNode, close)){
				continue;
			}

			// insert it into close
			close.insert(currNode);

			// iterate through all adjcent node:
			for (std::shared_ptr<Node> neighborNode : currNode->adjNodes){
				// Node must be not in close
				if (not inClose(neighborNode, close)){
					if ((not edgeAllowed or edgeAllowed(currNode->pos, neighborNode->pos)) and isClearLine(map, currNode->pos, neighborNode->pos, maxUnknownGap)){
						double cost = currNode->g + (currNode->pos - neighborNode->pos).norm();
						if (cost < neighborNode->g){
							neighborNode->g = cost;
							neighborNode->f = cost + (neighborNode->pos - goal->pos).norm();
							open.push(neighborNode);
							neighborNode->parent = currNode;
							record.push_back(neighborNode);
						}
					}
				}
			}
		}


		// backtracking
		std::shared_ptr<Node> ptr = goal;
		while (ptr != NULL and findPath){
			path.push_back(ptr);
			ptr = ptr->parent;
		}
		std::reverse(path.begin(), path.end());

		// clear search cache data
		for (std::shared_ptr<Node> n : record){
			n->g = std::numeric_limits<double>::infinity();
			n->f = std::numeric_limits<double>::infinity();
			n->parent = NULL;
		}
		return path;
	}
}

#endif
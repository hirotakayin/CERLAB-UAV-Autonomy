/*
	FILE: climbColumn.h
	-----------------------------
	find somewhere to climb straight up when there is no route home: beams and slabs can span the space above the drone
*/
#ifndef CLIMB_COLUMN
#define CLIMB_COLUMN
#include <Eigen/Eigen>
#include <cmath>
#include <deque>
#include <vector>

namespace AutoFlight{
	// Nearest point at the start height, inside the box, with a clear climb to topZ, and the way there at that height.
	// blocked(p) is true where the drone may not be. The climb keeps margin further from anything blocked. The way there is
	// searched on a grid, so it can go around walls; route gets its corners, without the start, ending at the column
	template <typename Blocked>
	bool findClimbRoute(const Eigen::Vector3d& start, double topZ, const Eigen::Vector2d& boxMin, const Eigen::Vector2d& boxMax,
	                    double margin, Blocked blocked, std::vector<Eigen::Vector3d>& route){
		const double step = 0.2, maxRadius = 20.0, lineStep = 0.1, leaveBlocked = 1.0;
		auto climbClear = [&](const Eigen::Vector3d& base){
			for (double z=base(2); z<=topZ; z+=lineStep){
				for (int i=0; i<5; ++i){
					Eigen::Vector3d p (base(0), base(1), z);
					if (i > 0) p(i <= 2 ? 0 : 1) += i % 2 == 0 ? margin : -margin;
					if (blocked(p)) return false;
				}
			}
			return not blocked(Eigen::Vector3d (base(0), base(1), topZ));
		};
		// the drone can start inside a safety margin: it may leave it, but not enter another one
		auto lineClear = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b, bool fromStart){
			double length = (b - a).norm();
			bool leaving = fromStart;
			for (double d=0.0; d<=length; d+=lineStep){
				Eigen::Vector3d p = a + (b - a) * (length > 0.0 ? d / length : 0.0);
				bool here = blocked(p);
				if (here and (not leaving or d > leaveBlocked)) return false;
				if (not here) leaving = false;
			}
			return not blocked(b);
		};
		int n = int(std::ceil(maxRadius / step));
		int size = 2 * n + 1;
		std::vector<int> parent (size * size, -2); // -2 unvisited, -1 the start
		auto cellPos = [&](int c){return Eigen::Vector3d (start(0) + (c % size - n) * step, start(1) + (c / size - n) * step, start(2));};
		std::deque<int> queue;
		int startCell = n * size + n;
		parent[startCell] = -1;
		queue.push_back(startCell);
		while (not queue.empty()){
			int c = queue.front();
			queue.pop_front();
			Eigen::Vector3d p = cellPos(c);
			bool inBox = p(0) >= boxMin(0) and p(0) <= boxMax(0) and p(1) >= boxMin(1) and p(1) <= boxMax(1);
			if (inBox and climbClear(p)){
				std::vector<Eigen::Vector3d> cells;
				for (int k=c; k!=-1; k=parent[k]) cells.push_back(cellPos(k));
				// keep only the corners: from each kept point, go as far along the cells as a straight line stays clear
				route.clear();
				size_t at = cells.size() - 1;
				Eigen::Vector3d from = start;
				while (at > 0){
					size_t next = at - 1;
					for (size_t k=0; k<at; ++k){
						if (lineClear(from, cells[k], (from - start).norm() < 1e-6)){ next = k; break; }
					}
					route.push_back(cells[next]);
					from = cells[next];
					at = next;
				}
				return true;
			}
			int cx = c % size, cy = c / size;
			for (int dy=-1; dy<=1; ++dy){
				for (int dx=-1; dx<=1; ++dx){
					int nx = cx + dx, ny = cy + dy;
					if ((dx == 0 and dy == 0) or nx < 0 or ny < 0 or nx >= size or ny >= size) continue;
					int k = ny * size + nx;
					if (parent[k] != -2) continue;
					Eigen::Vector3d q = cellPos(k);
					if ((q - start).norm() > maxRadius) continue;
					if (blocked(q) and (q - start).norm() > leaveBlocked) continue;
					parent[k] = c;
					queue.push_back(k);
				}
			}
		}
		return false;
	}
}
#endif

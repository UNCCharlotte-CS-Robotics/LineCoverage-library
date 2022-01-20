/**
 * This file is part of the LineCoverage-library.
 * The file contains Prim's algorithm for MST
 *
 * TODO:
 *
 * @author Saurav Agarwal
 * @contact sagarw10@uncc.edu
 * @contact agr.saurav1@gmail.com
 * Repository: https://github.com/UNCCharlotte-Robotics/LineCoverage-library
 *
 * Copyright (C) 2020--2022 University of North Carolina at Charlotte.
 * The LineCoverage-library is owned by the University of North Carolina at Charlotte and is protected by United States copyright laws and applicable international treaties and/or conventions.
 *
 * The LineCoverage-library is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * DISCLAIMER OF WARRANTIES: THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT WARRANTY OF ANY KIND INCLUDING ANY WARRANTIES OF PERFORMANCE OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR USE OR PURPOSE OR OF NON-INFRINGEMENT. YOU BEAR ALL RISK RELATING TO QUALITY AND PERFORMANCE OF THE SOFTWARE OR HARDWARE.
 *
 * SUPPORT AND MAINTENANCE: No support, installation, or training is provided.
 *
 * You should have received a copy of the GNU General Public License along with LineCoverage-library. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef LCLIBRARY_ALGORITHMS_MST_PRIM_H_
#define LCLIBRARY_ALGORITHMS_MST_PRIM_H_

#include <lclibrary/core/vertex.h>
#include <lclibrary/core/graph.h>
#include <limits>
#include <queue>
#include <memory>

namespace lclibrary {

	bool MST_Prim(const std::shared_ptr <const Graph> &g, std::vector<Edge> &mst_edges) {
		size_t n = g->GetN();
		size_t m = g->GetM();
		std::vector <std::vector <size_t>> adjacent_lists;
		adjacent_lists.resize(n);

		size_t t, h;
		for(size_t i = 0; i < m; ++i) {
			g->GetVerticesIndexOfEdge(i, t, h);
			adjacent_lists[t].push_back(i);
			adjacent_lists[h].push_back(i);
		}

		std::vector <size_t> adjacent_list_iterator;
		adjacent_list_iterator.resize(n, 0);

		std::vector <size_t> vertex_list(n, kNIL);
		for(size_t i = 0; i < n; ++i) {
			vertex_list[i] = i;
		}

		std::vector <bool> visited (n, false);
		std::vector <double> value (n, kDoubleMax);

		value[0] = 0;

		auto cmp = [&value](size_t lhs, size_t rhs) { return value[lhs] > value[rhs]; };
		std::priority_queue <size_t, std::vector<size_t>, decltype(cmp)> q(cmp);
		q.push(0);

		size_t u = kNIL;
		while(not q.empty()) {
			u = q.top();
			q.pop();
			if(visited[u] == true)
				continue;
			visited[u] = true;
			while(adjacent_list_iterator[u] < adjacent_lists[u].size()) {
				size_t v;
				size_t t, h;
				auto e = adjacent_lists[u][adjacent_list_iterator[u]];
				g->GetVerticesIndexOfEdge(e, t, h);
				++adjacent_list_iterator[u];
				if(u == t)
					v = h;
				else
					v = t;

				if(visited[v] == false and g->GetCost(e) < value[v] ) {

					g->GetVerticesIndexOfEdge(e, t, h);
					mst_edges.push_back(*(g->GetEdge(e)));
					value[v] = g->GetCost(e);
					q.push(v);
				}
			}
		}

		return 0;
	}

} // namespace lclibrary

#endif /* LCLIBRARY_ALGORITHMS_MST_PRIM_H_ */

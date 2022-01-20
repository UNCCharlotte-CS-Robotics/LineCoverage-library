/**
 * This file is part of the LineCoverage-library.
 * The file contains routine to check if graph is balanced
 *
 * TODO: Actually add functions to check Eulerian
 * Eulerian means balanced and connected
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

#ifndef LCLIBRARY_ALGORITHMS_ISBALANCED_H_
#define LCLIBRARY_ALGORITHMS_ISBALANCED_H_

#include <memory>
#include <lclibrary/core/graph.h>

namespace lclibrary {

	inline bool IsBalancedDigraph(const std::shared_ptr <const Graph> g) {
		auto n = g->GetN();
		auto m = g->GetM();
		auto m_nr = g->GetMnr();
		std::vector <int> vertex_imbalance_list(n, 0);
		size_t t_idx, h_idx;
		for(size_t i = 0; i < m; ++i) {
			g->GetVerticesIndexOfEdge(i, t_idx, h_idx, kIsRequired);
			++vertex_imbalance_list[t_idx];
			--vertex_imbalance_list[h_idx];
		}
		for(size_t i = 0; i < m_nr; ++i) {
			g->GetVerticesIndexOfEdge(i, t_idx, h_idx, kIsNotRequired);
			++vertex_imbalance_list[t_idx];
			--vertex_imbalance_list[h_idx];
		}
		for(auto &del:vertex_imbalance_list) {
			if(del != 0)
				return false;
		}
		return true;
	}

	inline void ComputeVertexDegree(const std::shared_ptr <const Graph> g, std::vector <int> &vertex_degree_list) {
		auto n = g->GetN();
		auto m = g->GetM();
		auto m_nr = g->GetMnr();
		vertex_degree_list.resize(n, 0);
		size_t t_idx, h_idx;
		for(size_t i = 0; i < m; ++i) {
			g->GetVerticesIndexOfEdge(i, t_idx, h_idx, kIsRequired);
			++vertex_degree_list[t_idx];
			++vertex_degree_list[h_idx];
		}
		for(size_t i = 0; i < m_nr; ++i) {
			g->GetVerticesIndexOfEdge(i, t_idx, h_idx, kIsNotRequired);
			++vertex_degree_list[t_idx];
			++vertex_degree_list[h_idx];
		}
	}

	inline bool IsBalancedGraph(const std::shared_ptr <const Graph> g) {
		std::vector <int> vertex_degree_list;
		ComputeVertexDegree(g, vertex_degree_list);
		for(const auto &del:vertex_degree_list) {
			if(del%2 != 0)
				return false;
		}
		return true;
	}

} // namespace lclibrary

#endif /* LCLIBRARY_ALGORITHMS_ISBALANCED_H_ */

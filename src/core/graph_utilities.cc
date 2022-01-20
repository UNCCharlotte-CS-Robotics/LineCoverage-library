/**
 * This file is part of the LineCoverage-library.
 * The file contains utilities functions for graphs
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

#include <lclibrary/core/graph_utilities.h>

namespace lclibrary {

	void AddCompleteNonRequiredEdges(std::shared_ptr <Graph> &G) {
		auto n = G->GetN();
		size_t m_nr = n * (n - 1)/2;
		std::vector <Edge> edge_list;
		edge_list.reserve(m_nr);
		for(size_t i = 0; i < n; ++i) {
			for (size_t j = i + 1; j < n; ++j) {
				Edge new_edge(G->GetVertexID(i), G->GetVertexID(j), false, 0, 0);
				edge_list.push_back(new_edge);
			}
		}
		G->AddEdge(edge_list, size_t(0), m_nr);
	}

	void AddReducedCompleteNonRequiredEdges(std::shared_ptr <Graph> &G) {
		auto n = G->GetN();
		auto m = G->GetM();
		size_t m_nr = n * (n - 1)/2 - m;
		std::vector <Edge> edge_list;
		edge_list.reserve(m_nr);
		for(size_t i = 0; i < n; ++i) {
			for (size_t j = i + 1; j < n; ++j) {
				bool existing = false;
				for(size_t k = 0; k < m; ++k) {
					size_t t, h;
					G->GetVerticesIndexOfEdge(k, t, h, kIsRequired);
					if((t == i and h == j) or (t == j and h == i)) {
						existing = true;
						break;
					}
				}
				if(existing == true) {
					continue;
				}
				Edge new_edge(G->GetVertexID(i), G->GetVertexID(j), false, 0, 0);
				edge_list.push_back(new_edge);
			}
		}
		G->AddEdge(edge_list, size_t(0), m_nr);
	}

	int ComputeAllEdgeCosts(std::shared_ptr <Graph> &G, EdgeCost &edge_cost_computer) {
		double cost, cost_rev;
		size_t m = G->GetM();
		for(size_t i = 0; i < m; ++i) {
			Edge e;
			G->GetEdgeData(i, e, true);
			if (edge_cost_computer.ComputeServiceCost(e, cost, cost_rev)) {
				std::cerr << "Error computing Edge Cost\n";
				return kFail;
			}
			G->SetServiceCost(i, cost, cost_rev);
			if (edge_cost_computer.ComputeDeadheadCost(e, cost, cost_rev)) {
				std::cerr << "Error computing Edge Cost\n";
				return kFail;
			}
			G->SetDeadheadCost(i, cost, cost_rev);
		}
		size_t m_nr = G->GetMnr();
		for(size_t i = 0; i < m_nr; ++i) {
			Edge e;
			G->GetEdgeData(i, e, false);
			if (edge_cost_computer.ComputeDeadheadCost(e, cost, cost_rev)) {
				std::cerr << "Error computing Edge Cost\n";
				return kFail;
			}
			G->SetDeadheadCost(i, cost, cost_rev, false);
		}
		return kSuccess;
	}

} /* lclibrary */

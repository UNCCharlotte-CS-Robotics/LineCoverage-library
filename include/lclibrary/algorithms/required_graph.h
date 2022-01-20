/**
 * This file is part of the LineCoverage-library.
 * The file contains helpful functions related to required subgraph
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

#ifndef LCLIBRARY_ALGORITHMS_REQUIRED_GRAPH_H_
#define LCLIBRARY_ALGORITHMS_REQUIRED_GRAPH_H_

#include <lclibrary/core/vertex.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/algorithms/connected_components.h>
#include <memory>

namespace lclibrary {

	inline bool GenerateRequiredGraph(const std::shared_ptr <const Graph> &g, std::shared_ptr <Graph> &g_r) {
		size_t m = g->GetM();
		size_t n = g->GetN();
		std::vector <bool> required_vertices_check (n, false);
		std::vector <Edge> edge_list;
		edge_list.reserve(m);
		std::vector <Vertex> vertex_list;
		vertex_list.reserve(n);
		for (size_t i = 0; i < m; ++i) {
			size_t t, h;
			g->GetVerticesIndexOfEdge(i, t, h, kIsRequired);
			if (required_vertices_check[t] == false) {
				Vertex tv;
				g->GetVertexData(t, tv);
				vertex_list.push_back(tv);
				required_vertices_check[t] = true;
			}
			if (required_vertices_check[h] == false) {
				Vertex hv;
				g->GetVertexData(h, hv);
				vertex_list.push_back(hv);
				required_vertices_check[h] = true;
			}
			Edge e;
			g->GetEdgeData(i, e, kIsRequired);
			edge_list.push_back(e);
		}
		g_r = std::make_shared <Graph> (vertex_list, edge_list, m, 0);
		return 0;
	}

	inline size_t GetNumCCRequiredGraph(std::shared_ptr <const Graph> g) {
		std::shared_ptr <Graph> g_r;
		GenerateRequiredGraph(g, g_r);
		g_r->AddReverseEdges();
		ConnectedComponents cc(g_r);
		cc.StronglyCCBalanced();
		size_t num_cc = cc.GetNumCC();
		return num_cc;
	}

} // namespace lclibrary

#endif /* LCLIBRARY_ALGORITHMS_REQUIRED_GRAPH_H_ */

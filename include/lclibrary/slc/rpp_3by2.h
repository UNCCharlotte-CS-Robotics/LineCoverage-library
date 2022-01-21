/**
 * This file is part of the LineCoverage-library.
 * The file contains Frederickson's 3/2-approximation algorithm for the Rural Postman Problem (RPP)
 *
 * TODO:	Add route improvement calls
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

#ifndef LCLIBRARY_SLC_RPP_H_
#define LCLIBRARY_SLC_RPP_H_

#include <list>
#include <utility>
#include <memory>
#include <lclibrary/core/core.h>
#include <lclibrary/utils/utils.h>
#include <lclibrary/algorithms/algorithms.h>
#include <lclibrary/slc/slc_base.h>
#include <lclibrary/slc/lp_glpk.h>
#include <lclibrary/slc/cpp.h>

namespace lclibrary {

	struct MSTEdge {
		double cost;
		size_t u;
		size_t v;

		MSTEdge() : cost{kDoubleMax}, u{kNIL}, v{kNIL} {}
	};

	class SLC_RPP : public SLC_Base{

		std::shared_ptr <Graph> mst_;
		std::shared_ptr <Graph> g_r_;
		std::shared_ptr <APSP_FloydWarshall> apsp_;
		std::shared_ptr <ConnectedComponents> cc_;
		size_t num_cc_;
		std::vector <int> vertex_degree_list_;
		std::vector<std::vector<MSTEdge>> edges_;

		void GenerateMSTGraph() {
			edges_.resize(num_cc_, std::vector<MSTEdge>(num_cc_));
			for(size_t i = 0; i < g_r_->GetN(); ++i) {
				auto u = cc_->GetVertexCC(i);
				size_t idx_i;
				g_->GetVertexIndex(g_r_->GetVertexID(i), idx_i);
				for(size_t j = 0; j < g_r_->GetN(); ++j) {
					auto v = cc_->GetVertexCC(j);
					size_t idx_j;
					g_->GetVertexIndex(g_r_->GetVertexID(j), idx_j);
					if(apsp_->GetCost(idx_i, idx_j) < edges_[u][v].cost) {
						edges_[u][v].cost = apsp_->GetCost(idx_i, idx_j);
						edges_[u][v].u = i;
						edges_[u][v].v = j;
					}
				}
			}
			std::vector <Vertex> vertex_list;
			std::vector <Edge> edge_list;
			vertex_list.reserve(num_cc_);
			for(size_t i = 0; i < num_cc_; ++i) {
				Vertex v(i);
				vertex_list.push_back(v);
				for(size_t j = i + 1; j < num_cc_; ++j) {
					Edge e(i, j);
					e.SetCost(edges_[i][j].cost);
					edge_list.push_back(e);
				}
			}
			mst_ = std::make_shared<Graph>(vertex_list, edge_list);
		}

		public:
		SLC_RPP(const std::shared_ptr <const Graph> &g_in) : SLC_Base (g_in) {
			apsp_ = std::make_shared <APSP_FloydWarshall>(g_);
			apsp_->APSP_Deadheading();
			std::vector <Vertex> vertex_list;
			for(size_t i = 0; i < g_->GetN(); ++i) {
				Vertex v;
				g_->GetVertexData(i, v);
				vertex_list.push_back(v);
			}
			std::vector <Edge> edge_list;
			for(size_t i = 0; i < g_->GetM(); ++i) {
				Edge e;
				g_->GetEdgeData(i, e, kIsRequired);
				edge_list.push_back(e);
			}
			sol_digraph_ = std::make_shared <Graph>(vertex_list, edge_list, edge_list.size(), 0);
		}

		int Solve() {
			GenerateRequiredGraph(g_, g_r_);
			g_r_->AddReverseEdges();

			cc_ = std::make_shared <ConnectedComponents>(g_r_);
			cc_->StronglyCCBalanced();
			num_cc_ = cc_->GetNumCC();
			std::vector <Edge> mst_edges;
			std::vector <Edge> mst_edge_list;
			if(num_cc_ != 1) {
				GenerateMSTGraph();
				MST_Prim(mst_, mst_edges);
				for(const auto &e:mst_edges) {
					size_t t, h;
					t = edges_[e.GetTailVertexID()][e.GetHeadVertexID()].u;
					h = edges_[e.GetTailVertexID()][e.GetHeadVertexID()].v;
					apsp_->GetPath(mst_edge_list, t, h);
				}
				sol_digraph_->AddEdge(mst_edge_list);
			}
			ComputeVertexDegree(sol_digraph_, vertex_degree_list_);
			size_t num_odd_vertices = std::count_if (vertex_degree_list_.begin(), vertex_degree_list_.end(), [](int i) {return ((i%2) == 1);});
			if(num_odd_vertices > 0) {
				std::vector <Edge> matching_edges;
				ComputeMatching(vertex_degree_list_, apsp_, matching_edges);
				sol_digraph_->AddEdge(matching_edges);
			}
			return GenerateTour();
		}

		bool GenerateTour() {
			route_ = EulerTourGeneration(sol_digraph_, kUndirectedGraph);
			route_.SetGraphAPSP(g_, apsp_);
			route_.RouteImprovement();
			if(route_.CheckRoute() == kFail)
				return kFail;
			sol_digraph_->ClearAllEdges();
			std::vector <Edge> edge_list;
			route_.GenerateEdgeList(edge_list);
			if(sol_digraph_->AddEdge(edge_list) == kFail)
				return kFail;
			return kSuccess;
		}

	};

}
#endif /* LCLIBRARY_SLC_RPP_H_ */

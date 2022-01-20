/**
 * This file is part of the LineCoverage-library.
 * The file contains functions to compute connected components for a balanced graph
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

#ifndef LCLIBRARY_ALGORITHMS_CONNECTEDCOMPONENTS_H_
#define LCLIBRARY_ALGORITHMS_CONNECTEDCOMPONENTS_H_

#include <lclibrary/core/vertex.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/core/route.h>
#include <lclibrary/algorithms/is_balanced.h>
#include <memory>

namespace lclibrary {
	enum Color {
		kWhite,
		kGray,
		kBlack,
	};

	class ConnectedComponents {
		std::shared_ptr <const Graph> g_;
		std::vector <const Vertex *> vertex_list_;
		std::vector <AdjacentEdgeList::const_iterator> adjacent_iterator_list_;
		std::vector <Color> vertex_color_list_;
		std::vector <size_t> vertex_cc_;
		std::vector <size_t> rep_vertices_;
		size_t n_;
		size_t cc_count_;

		void StronglyCCBalancedAux (size_t u) {
			vertex_cc_[u] = cc_count_;
			const Vertex *t_v, *h_v;
			vertex_color_list_[u] = kGray;
			size_t v;
			const Edge *e = nullptr;
			while (!vertex_list_[u]->IsAdjacencyEnd(adjacent_iterator_list_[u])) {
				vertex_list_[u]->GetAdjacentEdge(e, adjacent_iterator_list_[u]);
				++adjacent_iterator_list_[u];
				e->GetVertices(t_v, h_v);
				g_->GetVertexIndex(h_v->GetID(), v);
				if (vertex_color_list_[v] == kWhite) {
					StronglyCCBalancedAux(v);
				}
			}
			vertex_color_list_[u] = kBlack;

		}

		public:
		ConnectedComponents(const std::shared_ptr <const Graph> &g) : g_{g}, cc_count_{0} {
			n_ = g_->GetN();
			vertex_list_.reserve(n_);
			for(size_t i = 0; i < n_; ++i) {
				auto v = g_->GetVertex(i);
				vertex_list_.push_back(v);
				adjacent_iterator_list_.push_back(v->GetAdjacencyStart());
				vertex_cc_.push_back(0);
			}
			n_ = vertex_list_.size();
		}

		/* Compute connected components for a balanced graph */
		int StronglyCCBalanced() {
			if(!IsBalancedDigraph(g_)) {
				std::cerr << "Graph is not balanced: at each vertex, number of incoming edges should be equal to outgoing edges \n";
				return 1;
			}
			for(size_t i = 0; i < n_; ++i) {
				if (vertex_list_[i]->GetAdjListSize() > 0) {
					vertex_color_list_.push_back(kWhite);
				}
				else {
					vertex_color_list_.push_back(kBlack);
				}
			}
			for (size_t i = 0; i < n_; ++i) {
				if (vertex_color_list_[i] == kWhite) {
					rep_vertices_.push_back(i);
					StronglyCCBalancedAux(i);
					++cc_count_;
				}
			}
			return 0;
		}

		size_t GetNumCC() {
			return cc_count_;
		}

		size_t GetVertexCC(const size_t i) {
			return vertex_cc_[i];
		}

		size_t GetRepVertex(const size_t i) {
			return rep_vertices_[i];
		}

	};

} // namespace lclibrary

#endif /* LCLIBRARY_ALGORITHMS_CONNECTEDCOMPONENTS_H_ */

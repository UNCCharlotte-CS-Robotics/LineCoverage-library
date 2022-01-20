/**
 * This file is part of the LineCoverage-library.
 * The file contains functions to generate Eulerian Tour
 *
 * TODO: Why is the GraphType enum here?
 *			 Does not check if the graph is Eulerian!
 *			 Needs cleanup and improvement
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

#ifndef LCLIBRARY_ALGORITHMS_EULERTOURGENERATION_H_
#define LCLIBRARY_ALGORITHMS_EULERTOURGENERATION_H_

#include <lclibrary/core/vertex.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/core/route.h>
#include <lclibrary/algorithms/is_balanced.h>

namespace lclibrary {
enum GraphType {
	kDirectedGraph,
	kUndirectedGraph,
};

struct EdgeType {
	size_t e;
	bool type;
	bool is_traversed;
	EdgeType (): e{0}, type{kIsRequired}, is_traversed{false} {};
	EdgeType (size_t e_in, bool type_in, bool traversed): e{e_in}, type{type_in}, is_traversed{traversed} {};
};

inline Route EulerTourGeneration(const std::shared_ptr <const Graph> g, GraphType graph_type = kDirectedGraph) {
	Route route;
	if(graph_type == kDirectedGraph) {
		if(!IsBalancedDigraph(g)){
			std::cerr << "Graph is not an Eulerian digraph\n";
			return route;
		}
	}
	else if(!IsBalancedGraph(g)){
		std::cerr << "Graph is not balanced\n";
		return route;
	}

	auto n = g->GetN();
	auto m = g->GetM();
	auto m_nr = g->GetMnr();

	std::vector <std::vector <size_t>> adjacent_lists;
	adjacent_lists.resize(n);
	std::vector <EdgeType> edge_list;

	size_t t, h;
	for(size_t i = 0; i < m; ++i) {
		g->GetVerticesIndexOfEdge(i, t, h, kIsRequired);
		EdgeType e(i, kIsRequired, false);
		edge_list.push_back(e);
		adjacent_lists[t].push_back(edge_list.size() - 1);
		if (graph_type == kUndirectedGraph) {
			adjacent_lists[h].push_back(edge_list.size() - 1);
		}
	}
	for(size_t i = 0; i < m_nr; ++i) {
		g->GetVerticesIndexOfEdge(i, t, h, kIsNotRequired);
		EdgeType e(i, kIsNotRequired, false);
		edge_list.push_back(e);
		adjacent_lists[t].push_back(edge_list.size() - 1);
		if (graph_type == kUndirectedGraph) {
			adjacent_lists[h].push_back(edge_list.size() - 1);
		}
	}

	std::vector <size_t> adjacent_list_iterator;
	adjacent_list_iterator.resize(n, 0);

	const Vertex *t_v, *h_v;
	g->GetVerticesIndexOfEdge(0, t, h, kIsRequired);
	auto v = t;

	auto route_it = route.GetRouteStart();
	std::list <RouteEdges::const_iterator> insert_route;
	std::list <size_t> multi_vertex_list;
	const Edge *e = nullptr;
	while(adjacent_list_iterator[v] < adjacent_lists[v].size()) {
		auto e_idx = adjacent_lists[v][adjacent_list_iterator[v]];
		auto e_t = edge_list[e_idx];

		e = g->GetEdge(e_t.e, e_t.type);
		++adjacent_list_iterator[v];

		if (e_t.is_traversed == false) {

			Edge e_new;
			e_new = (*e);
			if(e_new.GetTailVertexID() != g->GetVertexID(v)) {
				e_new.Reverse();
				if(e_new.GetTailVertexID() != g->GetVertexID(v)) {
					return route;
				}
			}
			edge_list[e_idx].is_traversed = true;
			auto route_it1 = route.AddEdge(e_new, route_it);

			if(adjacent_list_iterator[v] < adjacent_lists[v].size()) {
				multi_vertex_list.push_back(v);
				insert_route.push_back(route_it1);
			}

			e_new.GetVertices(t_v, h_v);
			g->GetVertexIndex(h_v->GetID(), v);
		}

		while(adjacent_list_iterator[v] >= adjacent_lists[v].size() and multi_vertex_list.size() > 0) {
			v = multi_vertex_list.front();
			route_it = insert_route.front();
			multi_vertex_list.pop_front();
			insert_route.pop_front();
			while(adjacent_list_iterator[v] < adjacent_lists[v].size()) {
				auto idx = adjacent_lists[v][adjacent_list_iterator[v]];
				if (edge_list[idx].is_traversed == true) {
					++adjacent_list_iterator[v];
					continue;
				}
				else {
					break;
				}
			}
		}
	}
	return route;

}

} // namespace lclibrary

#endif /* LCLIBRARY_ALGORITHMS_EULERTOURGENERATION_H_ */

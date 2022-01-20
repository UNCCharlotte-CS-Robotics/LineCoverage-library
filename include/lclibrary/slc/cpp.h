/**
 * This file is part of the LineCoverage-library.
 * The file contains algorithm for the Chinese Postman Problem (CPP)
 *
 * TODO:	Add route improvement functions
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

#ifndef LCLIBRARY_SLC_CPP_H_
#define LCLIBRARY_SLC_CPP_H_

#include <lclibrary/core/core.h>
#include <lclibrary/utils/utils.h>
#include <lclibrary/algorithms/algorithms.h>
#include <lclibrary/slc/slc_base.h>
#include <list>
#include <utility>

namespace lclibrary {

	class SLC_CPP : public SLC_Base{
		std::shared_ptr <APSP_FloydWarshall> apsp_;
		std::vector <int> vertex_degree_list_;

		public:
		SLC_CPP(std::shared_ptr <const Graph> g_in) : SLC_Base (g_in){
			apsp_ = std::make_shared<APSP_FloydWarshall>(g_);
			apsp_->APSP_Deadheading();
			sol_digraph_ = std::make_shared <Graph> (*g_);
		}

		int Solve() {
			ComputeVertexDegree(g_, vertex_degree_list_);
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
			route_.RouteImprovement();
			if(route_.CheckRoute() == kFail)
				return kFail;
			sol_digraph_->ClearAllEdges();
			std::vector <Edge> edge_list;
			route_.GenerateEdgeList(edge_list);
			std::cout << "Route size: " << edge_list.size() << std::endl;
			if(sol_digraph_->AddEdge(edge_list) == kFail)
				return kFail;
			std::cout << "Route cost: " << route_.GetCost() << std::endl;
			return kSuccess;
		}

	};

}
#endif /* LCLIBRARY_SLC_CPP_HPP_ */

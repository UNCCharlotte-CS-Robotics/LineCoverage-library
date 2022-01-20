/**
 * This file is part of the LineCoverage-library.
 * The file contains routine to compute matching of odd vertices in a graph
 *
 * The file uses code from https://github.com/dilsonpereira/Minimum-Cost-Perfect-Matching under MIT license
 * Modifications to the program are released in the following repository:
 * https://github.com/AgarwalSaurav/mcpm
 *
 * TODO:	Consider writing own matching functions. I did not test the mcpm library.
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

#ifndef LCLIBRARY_ALGORITHMS_MATCHING_H_
#define LCLIBRARY_ALGORITHMS_MATCHING_H_

#include <vector>
#include <list>
#include <utility>
#include <memory>
#include <lclibrary/core/core.h>
#include <lclibrary/utils/utils.h>
#include <mcpm/graph.h>
#include <mcpm/matching.h>

namespace lclibrary{

	void ComputeMatching(std::vector <int> vertex_degree_list, const std::shared_ptr <const APSP_FloydWarshall> &apsp, std::vector <Edge> &matching_edges) {
		size_t num_odd_vertices = std::count_if (vertex_degree_list.begin(), vertex_degree_list.end(), [](int i) {return ((i%2) == 1);});
		std::vector <size_t> odd_vertices;
		odd_vertices.reserve(num_odd_vertices);
		std::list< std::pair<int, int> > edges;
		for(size_t i = 0; i < vertex_degree_list.size(); ++i) {
			if(vertex_degree_list[i]%2 == 1) {
				odd_vertices.push_back(i);
			}
		}
		std::vector <double> costs;
		costs.reserve((num_odd_vertices * num_odd_vertices - num_odd_vertices)/2);
		std::vector < std::vector <double> > d;
		d.resize(num_odd_vertices, std::vector <double>(num_odd_vertices));
		for(size_t i = 0; i < num_odd_vertices; ++i) {
			for(size_t j = i + 1; j < num_odd_vertices; ++j){
				costs.push_back(apsp->GetCost(odd_vertices[i], odd_vertices[j]));
				std::pair<int, int> edge (i, j);
				edges.push_back(edge);
			}
		}

		mcpm::Graph g_mcpm (num_odd_vertices, edges);
		mcpm::Matching m_mcpm(g_mcpm);
		std::pair< std::list<int>, double > solution = m_mcpm.SolveMinimumCostPerfectMatching(costs);
		auto matching = solution.first;
		for(std::list<int>::iterator it = matching.begin(); it != matching.end(); it++) {
			std::pair<int, int> e = g_mcpm.GetEdge( *it );
			apsp->GetPath(matching_edges, odd_vertices[e.first], odd_vertices[e.second]);
		}
	}
}

#endif /* LCLIBRARY_ALGORITHMS_MATCHING_H_ */

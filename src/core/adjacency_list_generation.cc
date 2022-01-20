/**
 * This file is part of the LineCoverage-library.
 * The file contains functions to generate adjacency list of graph
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

#include <lclibrary/core/graph.h>

namespace lclibrary {

	int Graph::AdjacencyListGeneration() {
		for (auto &v:vertex_list_) {
			v->ClearAdjacentList();
		}
		for(auto const &e:required_edge_list_) {
			Vertex *v1, *v2;
			if(GetVertex(e->GetTailVertexID(), v1) or GetVertex(e->GetHeadVertexID(), v2)) {
				std::cerr << "Edge list error" << std::endl;
				return kFail;
			}
			v1->AddEdge(e);
		}
		for(auto const &e:non_required_edge_list_) {
			Vertex *v1, *v2;
			if(GetVertex(e->GetTailVertexID(), v1) or GetVertex(e->GetHeadVertexID(), v2)) {
				std::cerr << "Edge list error" << std::endl;
				return kFail;
			}
			v1->AddEdge(e);
		}
		return kSuccess;
	}

} // namespace lclibrary

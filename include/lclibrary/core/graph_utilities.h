/**
 * This file is part of the LineCoverage-library.
 * The file contains utilities functions for graphs
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

#ifndef LCLIBRARY_CORE_GRAPH_UTILITIES_H_
#define LCLIBRARY_CORE_GRAPH_UTILITIES_H_

#include <lclibrary/core/graph.h>
#include <lclibrary/core/edge_cost_base.h>
#include <memory>

namespace lclibrary {

	/* Add a non-required edge for every pair of vertices */
	void AddCompleteNonRequiredEdges(std::shared_ptr <Graph> &);

	/* Add a non-required edge for every pair of vertices, except when it is same as a required edge */
	void AddReducedCompleteNonRequiredEdges(std::shared_ptr <Graph> &);

	/* Compute edge costs for all the edges using an EdgeCost based object */
	int ComputeAllEdgeCosts(std::shared_ptr <Graph> &, EdgeCost &);

}

#endif /* GRAPH_UTILITIES_UTILITIES_H_ */

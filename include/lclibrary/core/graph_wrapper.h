/**
 * This file is part of the LineCoverage-library.
 * The file contains wrapper function for graphs, uses Config object to create graphs
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

#ifndef LCLIBRARY_CORE_GRAPH_WRAPPER_H_
#define LCLIBRARY_CORE_GRAPH_WRAPPER_H_

#include <lclibrary/core/constants.h>
#include <lclibrary/core/config.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/core/graph_io.h>
#include <lclibrary/core/graph_utilities.h>
#include <lclibrary/core/edge_cost_base.h>
#include <lclibrary/utils/edge_cost_travel_time.h>
#include <lclibrary/utils/edge_cost_with_circular_turns.h>

#include <memory>

namespace lclibrary {

	inline int GraphCreate(const Config &config, std::shared_ptr <Graph> &g) {
		if(CreateGraph(g, config.database.dir + config.filenames.nodes_data, config.database.dir + config.filenames.req_edges, config.input_graph.lla, config.input_graph.costs) == kFail) {
			std::cerr << "Graph creation failed\n";
			return kFail;
		}

		if(config.add_pairwise_nonreq_edges) {
			/* AddCompleteNonRequiredEdges(g); */
			AddReducedCompleteNonRequiredEdges(g);
		}

		if(config.depot_mode == Config::DepotMode::mean) {
			if(g->SetMeanDepot() == kFail) {
				std::cerr << "Depot could not be set\n";
				return kFail;
			}
		}

		if(config.depot_mode == Config::DepotMode::custom) {
			if(g->SetDepot(config.depot_ID) == kFail) {
				std::cerr << "Depot could not be set\n";
				return kFail;
			}
		}

		if(config.problem == "mlc") {
			if(g->CheckDepotRequiredVertex() == kFail) {
				if(g->AddDepotAsRequiredEdge() == kFail) {
					return kFail;
				}
			}
		}

		return kSuccess;
	}

	inline int GraphCreateWithCostFn(const Config &config, std::shared_ptr <Graph> &g) {

		if(GraphCreate(config, g) == kFail) {
			return kFail;
		}
		
		if(config.cost_function == "euclidean") {
			g->SetDefaultEdgeCosts();
		}

		if(config.cost_function == "ramp") {
			g->SetRampEdgeCosts(config.ramp.acceleration, config.ramp.speed);
		}

		if(config.cost_function == "travel_time") {
			std::shared_ptr <EdgeCost> edge_cost_fn;
			auto params = config.travel_time;
			edge_cost_fn = std::make_shared <EdgeCost_TravelTime>(params.service_speed, params.deadhead_speed, params.wind_speed, params.wind_dir);
			ComputeAllEdgeCosts(g, *edge_cost_fn);
		}

		if(config.cost_function == "travel_time_circturns") {
			std::shared_ptr <EdgeCost_CircularTurns> edge_cost_fn;
			auto params = config.travel_time_circ_turns;
			edge_cost_fn = std::make_shared <EdgeCost_CircularTurns>(params.service_speed, params.deadhead_speed, params.wind_speed, params.wind_dir, params.angular_vel, params.acc, params.delta);
			ComputeAllEdgeCosts(g, *edge_cost_fn);
			g->SetTurnsCostFunction(edge_cost_fn);
		}

		if(config.problem == "mlc" or config.problem == "mlc_md") {
			g->SetCapacity(config.capacity);
			g->SetDemandsToCosts();
		}

		return kSuccess;
	}


}

#endif /* LCLIBRARY_CORE_GRAPH_WRAPPER_H_ */

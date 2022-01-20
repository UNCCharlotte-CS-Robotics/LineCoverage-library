/**
 * This file is part of the LineCoverage-library.
 * The file contains functions to convert OSM JSON to lclibrary format for vertices and edges
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

#ifndef LCLIBRARY_UTILS_CONVERT_OSM_JSON_GRAPH_H_
#define LCLIBRARY_UTILS_CONVERT_OSM_JSON_GRAPH_H_

#include <lclibrary/core/constants.h>
#include <lclibrary/core/config.h>
#include <lclibrary/core/transform_lla_xy.h>
#include <lclibrary/core/graph_io.h>
#include <lclibrary/core/graph_utilities.h>
#include <exception>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <json/json.hpp>

namespace lclibrary {

	using json = nlohmann::json;
	inline void OSMjsonGraph(const std::string &osm_filename, const std::string &node_filename, const std::string &edge_filename) {
		std::ifstream osm_file(osm_filename);
		std::ofstream node_file(node_filename);
		std::ofstream edge_file(edge_filename);
		json osm_data;
		osm_file >> osm_data;
		std::cout << osm_data["osm3s"]["copyright"] << std::endl;
		json elements = osm_data["elements"];
		for(auto &e:elements) {
			if(e["type"] == "node") {
				node_file << e["id"] << " " << e["lat"] << " " << e["lon"] << std::endl;
			}
			else if(e["type"] == "way") {
				json nodes = e["nodes"];
				size_t node_prev = nodes[0];
				bool start_flag = false;
				for(auto &n:nodes) {
					if(start_flag == false) {
						start_flag = true;
						continue;
					}
					edge_file << node_prev << " " << n << std::endl;
					node_prev = n;
				}
			}
			else {
				std::cout << "Unhandled element type: " << e["type"] << std::endl;
			}
		}

		osm_file.close();
		node_file.close();
		edge_file.close();
	}
	inline int OSMjsonGraph(const Config &config) {
		auto fn = config.filenames;
		auto dir = config.database.path + "/" + config.database.data_dir + "/";
		if(not std::filesystem::exists(dir + fn.map_json)) {
			std::cerr << "JSON file " << dir + fn.map_json << " does not exist\n";
			return 1;
		}
		OSMjsonGraph(dir + fn.map_json, dir + fn.nodes_ll, dir + fn.req_edges);
		LLAtoXY llaToXY(dir + fn.nodes_ll, dir + fn.nodes_data);
		std::shared_ptr <lclibrary::Graph> G;
		lclibrary::CreateGraph(G, dir + fn.nodes_data, dir + fn.req_edges, lclibrary::kIsWithLLA, not lclibrary::kIsWithCost, true);
		G->ShiftOrigin();
		WriteNodes(G, dir + fn.nodes_data, lclibrary::kIsWithLLA);
		if(config.add_pairwise_nonreq_edges) {
			/* AddCompleteNonRequiredEdges(G); */
			AddReducedCompleteNonRequiredEdges(G);
			WriteNonRequiredEdges(G, dir + fn.nonreq_edges);
		}

		std::string info_filename = dir + "/" + config.database.data_dir + ".info";
		WriteGraphInfo(G, info_filename);
		G->PrintNM();
		return 0;
	}
}

#endif /* LCLIBRARY_UTILS_CONVERT_OSM_JSON_GRAPH_H_*/

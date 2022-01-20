/**
 * This file is part of the LineCoverage-library.
 * The file contains function for writing GeoJSON file for graphs
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


#ifndef LCLIBRARY_UTILS_WRITEGEOJSON_H_
#define LCLIBRARY_UTILS_WRITEGEOJSON_H_

#include <fstream>
#include <lclibrary/core/graph.h>
#include <lclibrary/core/graph_io.h>
#include <lclibrary/core/graph_utilities.h>
#include <lclibrary/core/graph_wrapper.h>

namespace lclibrary {

	inline void WriteGeoJSON_Req(
			const std::shared_ptr <const Graph> &g,
			const std::string filename,
			const std::string varName){
		std::ofstream out_file (filename);
		out_file.precision(16);
		auto m = g->GetM();
		out_file << "var "<< varName <<" = [";
		for (size_t i = 0; i < m; ++i){
			double tLLA[3], hLLA[3];
			g->GetVertexLLAofEdge(i, tLLA, hLLA, kIsRequired);
			if (i == 0)
				out_file << "{\n";
			else
				out_file << ", {\n";
			out_file << "\t\"type\": \"Feature\",\n";
			out_file << "\t\"req\": \"true\",\n";
			out_file << "\t\"geometry\": {\n";
			out_file << "\t\"type\": \"LineString\",\n";
			out_file << "\t\"coordinates\": [["<< tLLA[1] <<", "<<tLLA[0]<<"], [" <<hLLA[1]<<", "<<hLLA[0]<<"]]\n}\n}";
		}
		out_file << "];";
		out_file.close();
	}

	inline void WriteGeoJSON_All(
			const std::shared_ptr <const Graph> &g,
			const std::string filename,
			const std::string varName){
		std::ofstream out_file (filename);
		out_file.precision(16);
		auto m = g->GetM(); auto m_nr = g->GetMnr();
		bool init = false;
		out_file << "var "<< varName <<" = [";
		for (size_t i = 0; i < m; ++i){
			double tLLA[3], hLLA[3];
			g->GetVertexLLAofEdge(i, tLLA, hLLA, kIsRequired);
			if (init == false) {
				out_file << "{\n";
				init = true;
			}
			else {
				out_file << ", {\n";
			}
			out_file << "\t\"type\": \"Feature\",\n";
			out_file << "\t\"req\": \"true\",\n";
			out_file << "\t\"geometry\": {\n";
			out_file << "\t\"type\": \"LineString\",\n";
			out_file << "\t\"coordinates\": [["<< tLLA[1] <<", "<<tLLA[0]<<"], [" <<hLLA[1]<<", "<<hLLA[0]<<"]]\n}\n}";
		}
		for (size_t i = 0; i < m_nr; ++i){
			double tLLA[3], hLLA[3];
			g->GetVertexLLAofEdge(i, tLLA, hLLA, kIsNotRequired);
			if (init == false) {
				out_file << "{\n";
				init = true;
			}
			else {
				out_file << ", {\n";
			}
			out_file << "\t\"type\": \"Feature\",\n";
			out_file << "\t\"req\": \"false\",\n";
			out_file << "\t\"geometry\": {\n";
			out_file << "\t\"type\": \"LineString\",\n";
			out_file << "\t\"coordinates\": [["<< tLLA[1] <<", "<<tLLA[0]<<"], [" <<hLLA[1]<<", "<<hLLA[0]<<"]]\n}\n}";
		}
		out_file << "];";
		out_file.close();
	}

	inline int WriteGeoJSON (const Config &config, std::shared_ptr <const Graph> g) {
		if(config.writeGeoJSON.write == false) {
			std::cerr << "writeGeoJSON is off in config\n";
			return kFail;
		}

		if(config.writeGeoJSON.non_req_edges) {
			WriteGeoJSON_All(g, config.database.dir + config.writeGeoJSON.filename, config.writeGeoJSON.var_name);
		} else {
			WriteGeoJSON_Req(g, config.database.dir + config.writeGeoJSON.filename, config.writeGeoJSON.var_name);
		}

		return kSuccess;
	}

	inline int WriteGeoJSON(const Config &config) {

		if(config.writeGeoJSON.write == false) {
			std::cerr << "writeGeoJSON is off in config\n";
			return kFail;
		}

		std::shared_ptr <Graph> g;
		if(GraphCreate (config, g) == kFail) {
			std::cerr << "Graph creation failed\n";
			return kFail;
		}

		return WriteGeoJSON(config, g);
	}

} /* lclibrary */
#endif /*  LCLIBRARY_UTILS_WRITEGEOJSON_H_*/

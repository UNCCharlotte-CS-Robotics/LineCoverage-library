/**
 * This file is part of the LineCoverage-library.
 * The file contains functions for Graph IO
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

#include <lclibrary/core/graph_io.h>

namespace lclibrary {

	/*! Create graph with only required edges */
	int CreateGraph(
			std::shared_ptr <Graph> &g,
			const std::string &vertex_list_file_name,
			const std::string &edge_list_file_name,
			const bool is_with_lla,
			const bool is_with_cost,
			const bool filter_vertices){

		std::ifstream vertex_list_infile (vertex_list_file_name);
		std::ifstream edge_list_infile (edge_list_file_name);

		if( !vertex_list_infile ) {
			std::cerr << "Cannot open " << vertex_list_file_name << std::endl;
			return kFail;
		}
		if( !edge_list_infile ) {
			std::cerr << "Cannot open " << edge_list_file_name << std::endl;
			return kFail;
		}

		FileParser(g, vertex_list_infile, edge_list_infile, is_with_lla, is_with_cost, filter_vertices);

		vertex_list_infile.close();
		edge_list_infile.close();
		return kSuccess;
	}

	/*! Create graph with required and non-required edges */
	int CreateGraph(
			std::shared_ptr <Graph> &g,
			const std::string &vertex_list_file_name,
			const std::string &req_edge_list_file_name,
			const std::string &non_req_edge_list_file_name,
			const bool is_with_lla,
			const bool is_with_cost){

		std::ifstream vertex_list_infile (vertex_list_file_name);
		std::ifstream req_edge_list_infile (req_edge_list_file_name);
		std::ifstream non_req_edge_list_infile (non_req_edge_list_file_name);

		if( !vertex_list_infile ) {
			std::cerr << "Cannot open " << vertex_list_file_name << std::endl;
			return kFail;
		}
		if( !req_edge_list_infile ) {
			std::cerr << "Cannot open " << req_edge_list_file_name << std::endl;
			return kFail;
		}

		if( !non_req_edge_list_infile ) {
			std::cerr << "Cannot open " << non_req_edge_list_file_name << std::endl;
			return kFail;
		}
		FileParser(g, vertex_list_infile, req_edge_list_infile, non_req_edge_list_infile, is_with_lla, is_with_cost);

		vertex_list_infile.close();
		req_edge_list_infile.close();
		non_req_edge_list_infile.close();
		return kSuccess;
	}

	/*! Write graph info */
	void WriteGraphInfo (std::shared_ptr <const Graph> G,
			const std::string &filename) {
		std::ofstream info_file(filename);
		info_file << "No. of nodes: " << G->GetN() << std::endl;
		info_file << "No. of required edges: " << G->GetM() << std::endl;
		info_file << "No. of non-required edges: " << G->GetMnr() << std::endl;
		info_file << "Length of the network (m): " << G->GetLength() << std::endl;
		info_file << "No. of connected components in required graph: " << GetNumCCRequiredGraph(G);
		info_file.close();
	}

	/*! Write node data to file */
	void WriteNodes(
			std::shared_ptr <const Graph> g,
			const std::string filename,
			const bool is_with_lla) {

		std::ofstream out_file (filename);
		out_file.precision(16);
		double lla[3];
		Vec2d xy;
		auto n = g->GetN();
		for(size_t i = 0; i < n; ++i){
			g->GetVertexXY(i, xy);
			if(is_with_lla) {
				g->GetVertexLLA(i, lla);
				out_file << g->GetVertexID(i) <<" "<< xy.x <<" " << xy.y <<" " << lla[0] <<" " << lla[1] <<" " << lla[2]<<"\n";
			}
			else {
				out_file << g->GetVertexID(i) <<" "<< xy.x <<" " << xy.y <<"\n";
			}
		}
		out_file.close();
	}

	/*! Write edge data to file */
	void WriteRequiredEdges(std::shared_ptr <const Graph> g, const std::string filename) {

		std::ofstream out_file (filename);
		out_file.precision(16);
		size_t m = g->GetM();
		size_t t_ID, h_ID;
		for (size_t i = 0; i < m; ++i) {
			g->GetVerticesIDOfEdge(i, t_ID, h_ID);
			out_file << t_ID << " " << h_ID << " " << g->GetServiceCost(i)<< " " << g->GetReverseServiceCost(i) << " " << g->GetDeadheadCost(i) << " " << g->GetReverseDeadheadCost(i);
			out_file<<"\n";
		}
		out_file.close();
	}

	void WriteNonRequiredEdges(std::shared_ptr <const Graph> g, const std::string filename) {
		std::ofstream out_file (filename);
		out_file.precision(16);
		size_t m_nr = g->GetMnr();
		size_t t_ID, h_ID;
		for (size_t i = 0; i < m_nr; ++i) {
			g->GetVerticesIDOfEdge(i, t_ID, h_ID, kIsNotRequired);
			out_file << t_ID << " " << h_ID << " " << g->GetDeadheadCost(i,kIsNotRequired)<< " " << g->GetReverseDeadheadCost(i,kIsNotRequired);
			out_file<<"\n";
		}
		out_file.close();
	}

	void DepotListParser(const std::string &depot_filename, std::vector <size_t> &depot_ids) {
		std::ifstream in_file (depot_filename);
		size_t depot;
		while(in_file >> depot) {
			depot_ids.push_back(depot);
		}
		in_file.close();
	}

}

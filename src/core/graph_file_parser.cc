/**
 * This file is part of the LineCoverage-library.
 * The file contains file parsers to create graphs
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

	void VertexParser (std::vector <Vertex> &vertex_list, std::ifstream &vertex_list_infile, const bool is_with_lla) {
		size_t vertex_ID;
		double lat, lng, alt, x, y;
		if (is_with_lla) {
			while(vertex_list_infile >> vertex_ID >> x >> y >> lat >> lng >> alt) {
				Vertex new_vertex(vertex_ID);
				new_vertex.SetLLA(lat, lng, alt);
				new_vertex.SetXY(x, y);
				vertex_list.push_back(new_vertex);
			}
		}
		else {
			while(vertex_list_infile >> vertex_ID >> x >> y) {
				Vertex new_vertex(vertex_ID);
				new_vertex.SetXY(x, y);
				vertex_list.push_back(new_vertex);
			}
		}
	}

	void FileParser (std::shared_ptr <Graph> &g, std::ifstream &vertex_list_infile, std::ifstream &edge_list_infile, const bool is_with_lla, const bool is_with_cost, bool filter_vertices) {
		std::vector <Vertex> vertex_list;
		VertexParser (vertex_list, vertex_list_infile, is_with_lla);
		std::vector <Edge> edge_list;
		size_t m = 0, m_nr = 0;
		EdgeParser (edge_list, edge_list_infile, is_with_cost, m);
		if(filter_vertices == true) {
			std::vector <Vertex> filtered_vertex_list;
			for(const auto &v:vertex_list) {
				bool found = false;
				for(const auto &e:edge_list) {
					if(v.GetID() == e.GetTailVertexID() or v.GetID() == e.GetHeadVertexID()) {
						found = true;
						break;
					}
				}
				if(found == true) {
					filtered_vertex_list.push_back(v);
				}
			}
			vertex_list = filtered_vertex_list;
		}
		g = std::make_shared <Graph>(vertex_list, edge_list, m, m_nr);
	}

	void FileParser (std::shared_ptr <Graph> &g, std::ifstream &vertex_list_infile, std::ifstream &req_edge_list_infile, std::ifstream &non_req_edge_list_infile, const bool is_with_lla, const bool is_with_cost, const bool filter_vertices) {
		std::vector <Vertex> vertex_list;
		VertexParser (vertex_list, vertex_list_infile, is_with_lla);
		std::vector <Edge> edge_list;
		size_t m = 0, m_nr = 0;
		EdgeParser (edge_list, req_edge_list_infile, non_req_edge_list_infile, is_with_cost, m, m_nr);
		g = std::make_shared <Graph>(vertex_list, edge_list, m, m_nr);
	}

	void EdgeParser (std::vector <Edge> &edge_list, std::ifstream &edge_list_infile, const bool is_with_cost, size_t &m) {
		size_t tail_v_ID, head_v_ID;
		if (is_with_cost) {
			double service_cost, service_cost_rev, deadhead_cost, deadhead_cost_rev;
			while(edge_list_infile >> tail_v_ID >> head_v_ID >> service_cost >> service_cost_rev >> deadhead_cost >> deadhead_cost_rev){
				edge_list.push_back(Edge(tail_v_ID, head_v_ID, true, service_cost, service_cost_rev, deadhead_cost, deadhead_cost_rev));
				++m;
			}
		}
		else {
			while(edge_list_infile >> tail_v_ID >> head_v_ID){
				edge_list.push_back(Edge(tail_v_ID, head_v_ID, true));
				++m;
			}
		}
	}

	void EdgeParser (std::vector <Edge> &edge_list, std::ifstream &req_edge_list_infile, std::ifstream &non_req_edge_list_infile, const bool is_with_cost, size_t &m, size_t &m_nr) {
		size_t tail_v_ID, head_v_ID;
		EdgeParser (edge_list, req_edge_list_infile, is_with_cost, m);
		if (is_with_cost) {
			double deadhead_cost, deadhead_cost_rev;
			while(non_req_edge_list_infile >> tail_v_ID >> head_v_ID >> deadhead_cost >> deadhead_cost_rev){
				edge_list.push_back(Edge(tail_v_ID, head_v_ID, false, deadhead_cost, deadhead_cost_rev));
				++m_nr;
			}
		}
		else {
			while(non_req_edge_list_infile >> tail_v_ID >> head_v_ID){
				edge_list.push_back(Edge(tail_v_ID, head_v_ID, false));
				++m_nr;
			}
		}
	}
}

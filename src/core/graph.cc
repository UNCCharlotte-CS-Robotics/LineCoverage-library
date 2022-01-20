/**
 * This file is part of the LineCoverage-library.
 * The file contains the definition of the class Graph
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

	Graph::Graph (const std::vector <Vertex> &vertex_list_i, const std::vector <Edge> &edge_list_i, const size_t m_i, const size_t m_nr_i) {
		size_t n_i = vertex_list_i.size();
		size_t repeated_vertex_count = 0;
		if (n_i > 0) {
			vertex_list_.reserve(n_i);
			for (size_t i = 0; i < n_i; ++i) {
				if (IsVertexInGraph(vertex_list_i[i].GetID()) ==  kSuccess) {
					std::cout << "Repeated vertex igonored: " << vertex_list_i[i].GetID() << std::endl;
					++repeated_vertex_count;
					continue;
				}
				Vertex *new_vertex;
				new_vertex = new Vertex(vertex_list_i[i]);
				vertex_list_.push_back(new_vertex);
				vertex_map_[new_vertex->GetID()] = vertex_list_.size() - 1;
			}
		}

		n_ = vertex_list_.size();
		if (n_ != n_i - repeated_vertex_count) {
			std::cerr << "Graph generation failed: vertex list error" << std::endl;
			return;
		}

		if(edge_list_i.size() != (m_i + m_nr_i)) {
			AddEdge(edge_list_i);
		}
		else {
			if(AddEdge(edge_list_i, m_i, m_nr_i)) {
				std::cerr << "Graph generation failed: edge list error" << std::endl;
			}
		}
		AdjacencyListGeneration();
	}

	Graph::~Graph() {
		for(auto &v:vertex_list_) {
			delete v;
		}
		vertex_list_.clear();
		for(auto &e:required_edge_list_) {
			delete e;
		}
		for(auto &e:non_required_edge_list_) {
			delete e;
		}
		required_edge_list_.clear();
		non_required_edge_list_.clear();
	}

	bool Graph::CopyDataFromGraph(const Graph &g) {
		n_ = g.GetN();
		vertex_list_.clear();
		vertex_list_.reserve(n_);
		m_ = g.GetM();
		required_edge_list_.clear();
		required_edge_list_.reserve(m_);
		m_nr_ = g.GetMnr();
		non_required_edge_list_.clear();
		non_required_edge_list_.reserve(m_nr_);

		for (size_t i = 0; i < n_; ++i) {
			if (IsVertexInGraph(g.GetVertexID(i)) ==  kSuccess)
				continue;
			Vertex *new_vertex;
			new_vertex = new Vertex(*(g.GetVertex(i)));
			vertex_list_.push_back(new_vertex);
			vertex_map_[new_vertex->GetID()] = vertex_list_.size() - 1;
		}

		for (size_t i = 0; i < m_; ++i) {
			auto e = *(g.GetEdge(i, kIsRequired));
			Vertex *v1, *v2;
			if(GetVertex(e.GetTailVertexID(), v1) or GetVertex(e.GetHeadVertexID(), v2)) {
				std::cerr << "Edge list error" << std::endl;
				return 1;
			}
			Edge *new_edge;
			new_edge = new Edge(e);
			new_edge->SetVertices(v1, v2);
			required_edge_list_.push_back(new_edge);
		}
		for (size_t i = 0; i < m_nr_; ++i) {
			auto e = *(g.GetEdge(i, kIsNotRequired));
			Vertex *v1, *v2;
			if(GetVertex(e.GetTailVertexID(), v1) or GetVertex(e.GetHeadVertexID(), v2)) {
				std::cerr << "Edge list error" << std::endl;
				return kFail;
			}
			Edge *new_edge;
			new_edge = new Edge(e);
			new_edge->SetVertices(v1, v2);
			non_required_edge_list_.push_back(new_edge);
		}
		UpdateEdgeCounts();
		AdjacencyListGeneration();
		capacity_ = g.GetCapacity();
		if(g.IsDepotSet()) {
			depot_ = g.GetDepot();
			is_depot_set_ = true;
			GetVertexXY(depot_, depot_xy_);
		}
		if(g.IsMultipleDepotSet()) {
			std::vector <size_t> depots;
			g.GetDepotsIDs(depots);
			AddDepots(depots);
		}

		return kSuccess;
	}

	int Graph::GetVertex (size_t const ID, Vertex* &v) const {
		size_t idx;
		if(GetVertexIndex(ID, idx)) {
			v = nullptr;
			return kFail;
		}
		v = vertex_list_[idx];
		return kSuccess;
	}

	int Graph::GetVertexIndex (const size_t ID, size_t &idx) const {
		auto search_vertex = vertex_map_.find(ID);
		if(search_vertex == vertex_map_.end()){
			std::cerr<<"Couldn't find vertex corresponding to ID\n";
			std::cerr<< ID <<"\n";
			idx = kNIL;
			return kFail;
		}
		idx = search_vertex->second;
		return kSuccess;
	}

	/*! Adds given Vertex */
	void Graph::AddVertex(Vertex *v) {
		vertex_list_.push_back(v);
		n_ = vertex_list_.size();
		vertex_map_[v->GetID()] = n_ - 1;
	}

	/*! Adds new Vertex by first creating a copy */
	const Vertex* Graph::AddNewVertex(Vertex const &v){
		Vertex *new_vertex;
		new_vertex = new Vertex(v);
		AddVertex(new_vertex);
		return new_vertex;
	}

	/*! Get index of vertices of Edge e */
	int Graph::GetVerticesIndexOfEdge(const size_t i, size_t &tIdx, size_t &hIdx, const bool is_req) const {
		Edge* e;
		if (is_req)
			e = required_edge_list_[i];
		else
			e = non_required_edge_list_[i];
		size_t t_ID = e->GetTailVertexID();
		size_t h_ID = e->GetHeadVertexID();
		if(GetVertexIndex(t_ID, tIdx))
			return kFail;
		if(GetVertexIndex(h_ID, hIdx))
			return kFail;
		return kSuccess;
	}

	void Graph::GetVerticesIDOfEdge(size_t i, size_t &t_ID, size_t &h_ID, bool is_req) const {
		Edge* e;
		if (is_req) {
			e = required_edge_list_[i];
		}
		else {
			e = non_required_edge_list_[i];
		}
		t_ID = e->GetTailVertexID();
		h_ID = e->GetHeadVertexID();
	}

	/*! Get coordinates of the Vertices of Edge e */
	void Graph::GetVertexCoordinateofEdge(size_t i, Vec2d &t_xy, Vec2d &h_xy, bool is_req) const {
		size_t t, h;
		if (is_req)
			GetVerticesIndexOfEdge(i, t, h);
		else
			GetVerticesIndexOfEdge(i, t, h, kIsNotRequired);
		t_xy = vertex_list_[t]->GetXY();
		h_xy = vertex_list_[h]->GetXY();
	}

	void Graph::GetVertexLLAofEdge(size_t i, double t_LLA[3], double h_LLA[3], bool is_req) const {
		size_t t, h;
		if (is_req)
			GetVerticesIndexOfEdge(i, t, h);
		else
			GetVerticesIndexOfEdge(i, t, h, kIsNotRequired);
		vertex_list_[t]->GetLLA(t_LLA);
		vertex_list_[h]->GetLLA(h_LLA);
	}

	/*! Add edge given IDs of the corresponding vertices */
	int Graph::AddEdge(const size_t v1_ID, const size_t v2_ID, const bool req){
		Vertex *v1, *v2;
		if (GetVertex(v1_ID, v1))
			return kFail;
		if (GetVertex(v2_ID, v2))
			return kFail;

		Edge* new_edge = new Edge(v1, v2, req);
		if (req) {
			required_edge_list_.push_back(new_edge);
		}
		else {
			non_required_edge_list_.push_back(new_edge);
		}
		UpdateEdgeCounts();
		if(AdjacencyListGeneration() == kFail) {
			std::cerr << "Adjacency list generation failed after adding edge\n";
			return kFail;
		}
		return kSuccess;
	}

	int Graph::AddEdge(const std::vector <Edge> &edge_list_i) {
		size_t req_count = 0, non_req_count = 0;
		for (size_t i = 0; i < edge_list_i.size(); ++i) {
			auto e = edge_list_i[i];
			Vertex *v1, *v2;
			if(GetVertex(e.GetTailVertexID(), v1) or GetVertex(e.GetHeadVertexID(), v2)) {
				std::cerr << "Edge list error" << std::endl;
				return kFail;
			}
			Edge *new_edge;
			new_edge = new Edge(e);
			new_edge->SetVertices(v1, v2);
			if(new_edge->GetReq()) {
				required_edge_list_.push_back(new_edge);
				++req_count;
			}
			else {
				non_required_edge_list_.push_back(new_edge);
				++non_req_count;
			}
		}
		UpdateEdgeCounts();
		AdjacencyListGeneration();
		return kSuccess;
	}

	int Graph::AddReverseEdges() {
		for (size_t i = 0; i < m_; ++i) {
			auto e = required_edge_list_[i];
			Edge *new_edge;
			new_edge = new Edge(*e);
			new_edge->Reverse();
			required_edge_list_.push_back(new_edge);
		}
		for (size_t i = 0; i < m_nr_; ++i) {
			auto e = non_required_edge_list_[i];
			Edge *new_edge;
			new_edge = new Edge(*e);
			new_edge->Reverse();
			non_required_edge_list_.push_back(new_edge);
		}
		UpdateEdgeCounts();
		AdjacencyListGeneration();
		return kSuccess;
	}

	int Graph::AddEdge(const std::vector <Edge> &edge_list_i, const size_t m_i, const size_t m_nr_i) {
		required_edge_list_.reserve(m_i);
		non_required_edge_list_.reserve(m_nr_i);
		return AddEdge(edge_list_i);
	}

	double Graph::ComputeArea() const {
		double minX, minY, maxX, maxY;
		minX = DBL_MAX; minY = DBL_MAX; maxX = -DBL_MAX; maxY = -DBL_MAX;
		for (auto v:vertex_list_) {
			auto xy = v->GetXY();
			if (xy.x < minX)
				minX = xy.x;
			if (xy.x > maxX)
				maxX = xy.x;
			if (xy.y < minY)
				minY = xy.y;
			if (xy.y > maxY)
				maxY = xy.y;
		}
		return (maxX - minX ) * (maxY - minY);
	}

	void Graph::GetLimits(double &minX, double &maxX, double &minY, double &maxY) const {
		minX = DBL_MAX; minY = DBL_MAX; maxX = -DBL_MAX; maxY = -DBL_MAX;
		for (auto v:vertex_list_) {
			auto xy = v->GetXY();
			if (xy.x < minX)
				minX = xy.x;
			if (xy.x > maxX)
				maxX = xy.x;
			if (xy.y < minY)
				minY = xy.y;
			if (xy.y > maxY)
				maxY = xy.y;
		}
	}

	void Graph::ShiftOrigin() {
		double minX, maxX, minY, maxY;
		GetLimits(minX, maxX, minY, maxY);
		for(auto &v:vertex_list_) {
			auto xy = v->GetXY();
			v->SetXY(xy.x - minX, xy.y - minY);
		}
	}


} /* lclibrary */

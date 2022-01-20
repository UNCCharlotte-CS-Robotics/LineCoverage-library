/**
 * This file is part of the LineCoverage-library.
 * The file contains the description of the class Graph
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

/**
 * TODO: Make the class templated and header only so that other vertex and edge objects can be used
 */

#ifndef LCLIBRARY_CORE_GRAPH_H_
#define LCLIBRARY_CORE_GRAPH_H_

#include <lclibrary/core/typedefs.h>
#include <lclibrary/core/vec2d.h>
#include <lclibrary/core/vertex.h>
#include <lclibrary/core/edge.h>
#include <lclibrary/core/edge_cost_base.h>
#include <lclibrary/utils/edge_cost_with_circular_turns.h>

#include <vector>
#include <array>
#include <unordered_map>
#include <cfloat>
#include <fstream>
#include <memory>

namespace lclibrary {

	struct GraphEdge {
		size_t edge_index_;
		bool req_ = true;
		bool rev_ = false;
		bool serv_ = true;
		GraphEdge(size_t i, bool req, bool rev, bool serv) : edge_index_(i), req_(req), rev_(rev), serv_(serv) {}
	};

	typedef std::vector <GraphEdge> GraphEdgeList;

	class Graph
	{

		private:

			VertexList vertex_list_; /*! Stores the vertices of the graph */
			EdgeList required_edge_list_; /*! Stores the required edges of the graph */
			EdgeList non_required_edge_list_; /*! Stores the required edges of the graph */
			std::vector <size_t> depot_list_;

			std::unordered_map <size_t, size_t> vertex_map_; /*! Stores a map of the index of vertices to actual ID of the node <ID, index>*/

			size_t n_ = 0; /*! No. of Vertices */
			size_t m_ = 0; /*! No. of required Edges */
			size_t m_nr_ = 0; /*! No. of non-required Edges */
			size_t depot_ = 0, depot_id_ = 0;
			bool is_depot_set_ = false;
			bool has_multiple_depots_ = false;
			Vec2d depot_xy_;
			double capacity_;
			std::shared_ptr <EdgeCost_CircularTurns> edge_cost_fn_ = nullptr;

		protected:
			int GetVertex (size_t const, Vertex* &) const;

			void UpdateEdgeCounts(){
				m_ = required_edge_list_.size();
				m_nr_ = non_required_edge_list_.size();
			}

			int AdjacencyListGeneration();

		public:

			Graph() : n_{0}, m_{0}, m_nr_{0} {}
			Graph (const std::vector <Vertex> &, const std::vector <Edge> &, const size_t m_i = 0, const size_t m_nr_i = 0);
			Graph(const Graph &g){ CopyDataFromGraph(g); }
			inline void operator = (const Graph &g ) { CopyDataFromGraph(g); }
			bool CopyDataFromGraph(const Graph &g);

			~Graph();

			void SetTurnsCostFunction(const std::shared_ptr <EdgeCost_CircularTurns> &cost_fn) {
				edge_cost_fn_ = cost_fn;
			}

			std::shared_ptr <const EdgeCost_CircularTurns> GetTurnsCostFunction() const {
				return edge_cost_fn_;
			}

			double GetTurnCost(const size_t edge1, const size_t edge2, const bool req1, const bool req2, const bool serv1, const bool serv2, const bool rev1, const bool rev2, CircularTurn &circ_turn) const {
				double cost = kDoubleNaN;
				edge_cost_fn_->ComputeTurnCost(GetEdge(edge1, req1), GetEdge(edge1, req2), serv1, serv2, rev1, rev2, cost, circ_turn);
				return cost;
			}

			bool SetDepot(size_t vertex_id) {
				size_t idx;
				if(GetVertexIndex(vertex_id, idx)) {
					std::cerr << "Depot vertex not found\n";
					return kFail;
				}
				depot_ = idx;
				depot_id_ = vertex_id;
				is_depot_set_ = true;
				GetVertexXY(depot_, depot_xy_);
				return kSuccess;
			}

			bool SetDepotIndex(const size_t vertex_idx) {
				depot_ = vertex_idx;
				depot_id_ = vertex_list_[depot_]->GetID();
				is_depot_set_ = true;
				GetVertexXY(depot_, depot_xy_);
				return kSuccess;
			}

			int SetMeanDepot() {
				Vec2d mean;
				for(size_t i = 0; i < n_; ++i) {
					Vec2d v_xy;
					GetVertexXY(i, v_xy);
					mean.Add(v_xy);
				}
				mean.x = mean.x/n_; mean.y = mean.y/n_;
				double min_dist_sqr = kDoubleMax;
				size_t depot_id = GetVertexID(0);
				for(size_t i = 0; i < m_; ++i) {
					size_t u, v;
					Vec2d u_xy, v_xy;
					GetVerticesIDOfEdge(i, u, v, kIsRequired);
					GetVertexCoordinateofEdge(i, u_xy, v_xy, kIsRequired);
					if(mean.DistSqr(u_xy) < min_dist_sqr) {
						depot_id = u;
						min_dist_sqr = mean.DistSqr(u_xy);
					}
					if(mean.DistSqr(v_xy) < min_dist_sqr) {
						depot_id = v;
						min_dist_sqr = mean.DistSqr(v_xy);
					}
				}
				is_depot_set_ = true;
				return SetDepot(depot_id);
			}

			size_t GetDepot() const {
				if(is_depot_set_ == false) {
					size_t t, h;
					GetVerticesIndexOfEdge(0, t, h, kIsRequired);
					return t;
				}
				return depot_;
			}

			size_t GetDepotID() const {
				return depot_id_;
			}

			void GetDepotXY(double &x, double &y) const {
				x = depot_xy_.x; y = depot_xy_.y;
			}

			inline bool IsDepotSet() const { return is_depot_set_; }

			int AddDepots(const std::vector <size_t> &depot_ids) {
				for(const auto &d:depot_ids) {
					if(IsVertexInGraph(d) == kFail) {
						std::cerr << "Depot not found\n";
						return kFail;
					}
				}
				depot_list_ = depot_ids;
				has_multiple_depots_ = true;
				return kSuccess;
			}

			int GetDepotsXY(std::vector <Vec2d> &depots) const {
				for(const auto depot:depot_list_) {
					size_t v;
					Vec2d xy;
					if(GetVertexIndex(depot, v)) {
						std::cerr << "Depot not found\n";
						return kFail;
					}
					GetVertexXY(v, xy);
					depots.push_back(xy);
				}
				return kSuccess;
			}

			inline bool IsMultipleDepotSet() const{
				return has_multiple_depots_;
			}

			void GetDepotIDs(std::vector <size_t> &depot_ids) const {
				depot_ids = depot_list_;
			}

			void GetDepotIndices(std::vector <size_t> &depot_indices) const {
				for(auto &v:depot_list_) {
					size_t v_idx;
					GetVertexIndex(v, v_idx);
					depot_indices.push_back(v_idx);
				}
			}

			void AddAllDepots() {
				depot_list_.clear();
				for(auto &v:vertex_list_) {
					depot_list_.push_back(v->GetID());
				}
			}

			inline bool CheckDepotRequiredVertex() {
				return IsRequiredVertex(depot_id_);
			}

			inline bool IsRequiredVertex(const size_t &v_ID) {
				for(const auto &e:required_edge_list_) {
					auto t_ID = e->GetTailVertexID();
					auto h_ID = e->GetHeadVertexID();
					if(v_ID == h_ID or v_ID == t_ID) {
						return kSuccess;
					}
				}
				return kFail;
			}

			bool AddDepotAsRequiredEdge() {
				return AddEdge(depot_id_, depot_id_, kIsRequired);
			}

			void GetDepotsIDs(std::vector <size_t> &depot_ids) const {
				depot_ids = depot_list_;
			}

			void GetDepotsIndices(std::vector <size_t> &depot_indices) const {
				size_t id;
				for(const auto &v:depot_list_) {
					GetVertexIndex(v, id);
					depot_indices.push_back(id);
				}
			}

			/* ************************* */
			/* Vertex related functions */
			inline const Vertex * GetVertex(size_t i) const {
				if (i >= n_)
					return nullptr;
				return vertex_list_[i];
			}

			int GetVertexIndex (const size_t , size_t &) const;
			inline size_t GetVertexID (const size_t v) const {
				return vertex_list_[v]->GetID();
			}
			inline void GetVertexData (const size_t i, Vertex &v) const {
				v = *vertex_list_[i];
			}

			bool IsVertexInGraph (const size_t ID) const {
				auto search_vertex = vertex_map_.find(ID);
				if(search_vertex == vertex_map_.end())
					return kFail;
				else
					return kSuccess;
			}

			void AddVertex(Vertex *);
			const Vertex* AddNewVertex(const Vertex &);

			inline void GetVertexXY(const size_t v, Vec2d &xy) const {
				xy = vertex_list_[v]->GetXY();
			}

			inline void GetVertexLLA(const size_t v, double * lla) const {
				vertex_list_[v]->GetLLA(lla);
			}
			int GetVerticesIndexOfEdge(const size_t, size_t &, size_t &, const bool is_req = kIsRequired) const ;
			void GetVerticesIDOfEdge(const size_t, size_t &, size_t &, const bool is_req = kIsRequired) const ;
			void GetVerticesIDOfEdge(const GraphEdge &edge, size_t &t, size_t &h) const {
				GetVerticesIDOfEdge(edge.edge_index_, t, h, edge.req_);
				if(edge.rev_) {
					std::swap(t, h);
				}
			}
			void GetVertexCoordinateofEdge(const size_t, Vec2d &, Vec2d &, const bool is_req = kIsRequired) const ;
			void GetVertexCoordinateofEdge(const GraphEdge &edge, Vec2d &t_xy, Vec2d &h_xy) const {
				GetVertexCoordinateofEdge(edge.edge_index_, t_xy, h_xy, edge.req_);
				if(edge.rev_) {
					std::swap(t_xy, h_xy);
				}
			}

			void GetVertexLLAofEdge(const size_t, double t_LLA[3], double h_LLA[3], const bool is_req = kIsRequired) const ;

			/* ************************* */

			/* ************************* */
			/* Edge related functions */
			inline const Edge * GetEdge(const size_t i, const bool is_req = kIsRequired) const {
				if (is_req)
					return required_edge_list_[i];
				else
					return non_required_edge_list_[i];
			}

			inline void GetEdgeData (const size_t i, Edge &e, const bool is_req = kIsRequired) const {
				if (is_req)
					e = (*required_edge_list_[i]);
				else
					e = (*non_required_edge_list_[i]);
			}

			int AddEdge(const size_t, const size_t, const bool req = kIsRequired);
			int AddEdge(const std::vector <Edge> &);
			int AddEdge(const std::vector <Edge> &, const size_t, const size_t);
			int AddReverseEdges();

			void ClearAllEdges() {
				for(auto &e:required_edge_list_) {
					delete e;
				}
				for(auto &e:non_required_edge_list_) {
					delete e;
				}
				required_edge_list_.clear();
				non_required_edge_list_.clear();
			}

			/* ************************* */

			inline double GetCost(const size_t i) const  {
				return required_edge_list_[i]->GetCost();
			}

			inline double GetServiceCost(const size_t i, const bool is_rev) const  {
				if(is_rev == false) {
					return required_edge_list_[i]->GetServiceCost();
				} else {
					return required_edge_list_[i]->GetReverseServiceCost();
				}
			}

			inline double GetServiceCost(const size_t i) const  {
				return required_edge_list_[i]->GetServiceCost();
			}

			inline double GetReverseServiceCost(const size_t i) const  {
				return required_edge_list_[i]->GetReverseServiceCost();
			}

			inline double GetCost(const GraphEdge &edge) const {
				if(edge.serv_ == true) {
					return GetServiceCost(edge.edge_index_, edge.rev_);
				} else {
					return GetDeadheadCost(edge.edge_index_, edge.req_, edge.rev_);
				}
			}

			inline double GetDeadheadCost(const size_t i, const bool is_req, const bool is_rev) const {
				if(is_req) {
					if(is_rev) {
						return required_edge_list_[i]->GetReverseDeadheadCost();
					} else {
						return required_edge_list_[i]->GetDeadheadCost();
					}
				} else {
					if(is_rev) {
						return non_required_edge_list_[i]->GetReverseDeadheadCost();
					} else {
						return non_required_edge_list_[i]->GetDeadheadCost();
					}
				}
			}

			inline double GetDeadheadCost(const size_t i, const bool is_req = kIsRequired) const  {
				if (is_req)
					return required_edge_list_[i]->GetDeadheadCost();
				else
					return non_required_edge_list_[i]->GetDeadheadCost();
			}

			inline double GetReverseDeadheadCost(const size_t i, const bool is_req = kIsRequired) const  {
				if (is_req)
					return required_edge_list_[i]->GetReverseDeadheadCost();
				else
					return non_required_edge_list_[i]->GetReverseDeadheadCost();
			}

			inline void SetCost(const size_t i, const double c) { required_edge_list_[i]->SetCost(c); }
			inline void SetCosts(const size_t i, const double c, bool is_req = kIsRequired) {
				if(is_req == kIsRequired)
					required_edge_list_[i]->SetCosts(c);
				else
					non_required_edge_list_[i]->SetCosts(c);
			}
			inline void SetServiceCost(const size_t i, const double c_s) { required_edge_list_[i]->SetServiceCost(c_s); }
			inline void SetServiceCost(const size_t i, const double c_s, double c_srev) { required_edge_list_[i]->SetServiceCost(c_s, c_srev); }

			inline void SetDeadheadCost(const size_t i, const double c_d, const bool is_req = kIsRequired) {
				if (is_req == kIsRequired)
					required_edge_list_[i]->SetDeadheadCost(c_d);
				else
					non_required_edge_list_[i]->SetDeadheadCost(c_d);
			}

			inline void SetDeadheadCost(const size_t i, const double c_d, const double c_drev, const bool is_req = kIsRequired) {
				if (is_req == kIsRequired)
					required_edge_list_[i]->SetDeadheadCost(c_d, c_drev);
				else
					non_required_edge_list_[i]->SetDeadheadCost(c_d, c_drev);
			}

			inline double GetDemand(const size_t i) const  {
				return required_edge_list_[i]->GetDemand();
			}

			inline double GetServiceDemand(const size_t i) const  {
				return required_edge_list_[i]->GetServiceDemand();
			}

			inline double GetReverseServiceDemand(const size_t i) const  {
				return required_edge_list_[i]->GetReverseServiceDemand();
			}

			inline double GetDeadheadDemand(const size_t i, const bool is_req = kIsRequired) const  {
				if (is_req == kIsRequired)
					return required_edge_list_[i]->GetDeadheadDemand();
				else
					return non_required_edge_list_[i]->GetDeadheadDemand();
			}

			inline double GetReverseDeadheadDemand(const size_t i, const bool is_req = kIsRequired) const  {
				if (is_req == kIsRequired)
					return required_edge_list_[i]->GetReverseDeadheadDemand();
				else
					return non_required_edge_list_[i]->GetReverseDeadheadDemand();
			}

			inline void SetDemand(const size_t i, const double q) { required_edge_list_[i]->SetDemand(q); }
			inline void SetServiceDemand(const size_t i, const double q_s) { required_edge_list_[i]->SetServiceDemands(q_s, q_s); }
			inline void SetServiceDemand(const size_t i, const double q_s, double q_srev) { required_edge_list_[i]->SetServiceDemands(q_s, q_srev); }

			inline void SetDeadheadDemand(const size_t i, const double q_d, const bool is_req = kIsRequired) {
				if (is_req)
					required_edge_list_[i]->SetDeadheadDemands(q_d, q_d);
				else
					non_required_edge_list_[i]->SetDeadheadDemands(q_d, q_d);
			}

			inline void SetDeadheadDemand(const size_t i, const double q_d, const double q_drev, const bool is_req = kIsRequired) {
				if (is_req) {
					required_edge_list_[i]->SetDeadheadDemands(q_d, q_drev);
				}
				else {
					non_required_edge_list_[i]->SetDeadheadDemands(q_d, q_drev);
				}
			}

			inline void SetCapacity(double q) { capacity_ = q; }
			inline double GetCapacity() const { return capacity_; }
			/* ************************* */

			double ComputeArea() const;
			void GetLimits(double &, double &, double &, double &) const;
			void ShiftOrigin();

			inline size_t GetN() const { return n_; }
			inline size_t GetM() const { return m_; }
			inline size_t GetMnr() const { return m_nr_; }

			double GetCost () const {
				double cost = 0;
				for (auto &e : required_edge_list_) {
					cost += e->GetCost();
				}
				for (auto &e : non_required_edge_list_) {
					cost += e->GetCost();
				}
				return cost;
			}

			/*! Display the size of Graph */
			inline void PrintNM() const {
				std::cout << "Number of nodes = " << n_;
				std::cout << "\nNumber of required edges = " << m_;
				std::cout << "\nNumber of non-required edges = " << m_nr_ << std::endl;
			}

			void SetDefaultEdgeCosts() {
				for(auto e:required_edge_list_) {
					e->ComputeCost();
				}
				for(auto e:non_required_edge_list_) {
					e->ComputeCost();
				}
			}

			void SetRampEdgeCosts(const double acc, const double vel) {
				for(auto e:required_edge_list_) {
					e->ComputeTravelTimeRamp(acc, vel);
				}
				for(auto e:non_required_edge_list_) {
					e->ComputeTravelTimeRamp(acc, vel);
				}
			}

			void SetDemandsToCosts() {
				for(auto e:required_edge_list_) {
					e->SetDemandsToCosts();
				}
				for(auto e:non_required_edge_list_) {
					e->SetDemandsToCosts();
				}
			}

			double GetLength() const{
				Vec2d u, v;
				double length = 0;
				for(size_t i = 0; i < m_; ++i) {
					GetVertexCoordinateofEdge(i, u, v, kIsRequired);
					length += u.Dist(v);
				}
				return length;
			}
	};
} // namespace lclibrary
#endif /*  LCLIBRARY_CORE_GRAPH_H_*/

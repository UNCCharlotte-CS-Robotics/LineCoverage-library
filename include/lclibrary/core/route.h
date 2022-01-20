/**
 * This file is part of the LineCoverage-library.
 * The file contains description of the class Route
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
 * TODO:
 * Use smart pointers
 * Needs cleanup
 * Move 2opt and route improvements to separate files?
 * Modify 2opt to handle depot: currently a fake required edge is created for the depot elsewhere in the library.
 * The class does not handle servicing separately. It sets edges to kIsNotRequired for deadheading a required arc. Might want to rewrite so that Graph object is used directly.
 */

#ifndef LCLIBRARY_CORE_ROUTE_H_
#define LCLIBRARY_CORE_ROUTE_H_

#include <lclibrary/core/math_utils.h>
#include <lclibrary/core/edge.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/algorithms/apsp_base.h>
#include <lclibrary/core/edge_cost_base.h>
#include <fstream>
#include <memory>
#include <algorithm>

namespace lclibrary {
	typedef std::list <Edge> RouteEdges;

	class Route {

		RouteEdges route_;
		std::vector <double> cummulative_costs_;
		std::vector <double> rev_cummulative_costs_;
		std::vector <const Edge *> route_vector_;
		size_t m_;
		size_t local_moves_count_ = 0;
		std::shared_ptr <const Graph> g_;
		std::shared_ptr <const APSP> apsp_;

		public:

		void AddEdge(Edge e) {
			route_.push_back(e);
		}

		RouteEdges::const_iterator AddEdge(Edge e, RouteEdges::const_iterator it) {
			return route_.insert(it, e);
		}

		RouteEdges::const_iterator GetRouteStart() const {
			return route_.cbegin();
		}

		RouteEdges::const_iterator GetRouteEnd() const {
			return route_.cend();
		}

		void SetGraph(const std::shared_ptr <const Graph> g) {
			g_ = g;
		}

		void SetAPSP(const std::shared_ptr <const APSP> apsp) {
			apsp_ = apsp;
		}

		void SetGraphAPSP(const std::shared_ptr <const Graph> g, const std::shared_ptr <const APSP> apsp) {
			g_ = g;
			apsp_ = apsp;
		}

		/* Checks is the arcs are connected */
		bool CheckRoute() const {
			size_t prev_head_ID = (route_.cbegin())->GetTailVertexID();
			for(auto &e:route_) {
				if (prev_head_ID != e.GetTailVertexID()) {
					std::cerr << "The route is disconnected\n";
					std::cerr << prev_head_ID << " " << e.GetTailVertexID() << std::endl;
					return kFail;
				}
				prev_head_ID = e.GetHeadVertexID();
			}
			return kSuccess;
		}

		size_t GetNumLocalMoves() const {
			return local_moves_count_;
		}

		void PrintRoute() const {
			double cost = 0;
			double edge_cost = 0, rev_edge_cost = 0;
			size_t i = 0;
			for(auto &e:route_) {
				cost += e.GetCost();
				GetEdgeCosts(e, edge_cost, rev_edge_cost);
				std::cout << i << " " << e.GetTailVertexID() << " " << e.GetHeadVertexID() << " " << e.GetReq() << " "<< cost << " " << edge_cost << " " << rev_edge_cost << std::endl;
				++i;
			}
			std::cout << std::endl;
		}

		size_t GetRouteLength() const {
			return route_.size();
		}

		void GetTailHeadReq(RouteEdges::const_iterator &it, size_t &tail, size_t &head, bool &req) const {
			tail = it->GetTailVertexID();
			head = it->GetHeadVertexID();
			req = it->GetReq();
		}

		void GetEdgeCosts(const Edge &e, double &cost, double &rev_cost) const {
			if(e.GetReq() == kIsRequired) {
				cost = e.GetServiceCost();
				rev_cost = e.GetReverseServiceCost();
			} else {
				cost = e.GetDeadheadCost();
				rev_cost = e.GetReverseDeadheadCost();
			}
		}

		double GetCost() const {
			double cost = 0;
			for(auto &e:route_) {
				cost += e.GetCost();
			}
			return cost;
		}

		double GetEdgeCost (RouteEdges::const_iterator it) const {
			/* auto e = *it; */
			return it->GetCost();
		}

		int GetXY(RouteEdges::const_iterator it, Vec2d &t_xy, Vec2d &h_xy) const {
			auto e = *it;
			const Vertex *t, *h;
			e.GetVertices(t, h);
			if(t!=nullptr and h!=nullptr) {
				t_xy = t->GetXY();
				h_xy = h->GetXY();
				return 0;
			}
			return 1;
		}

		bool IsReqEdge(RouteEdges::const_iterator it) const {
			return (*it).GetReq();
		}

		void ComputeCummulativeCosts() {
			/* std::cout << "Computing commulative costs\n"; */
			/* PrintRoute(); */
			double cost = 0, rev_cost = 0;
			size_t ii = 0;
			for(const auto &e:route_) {
				GetEdgeCosts(e, cost, rev_cost);
				cummulative_costs_[ii] = cost;
				rev_cummulative_costs_[ii] = rev_cost;
				++ii;
			}
			for(size_t i = 1; i < route_.size(); ++i) {
				cummulative_costs_[i] += cummulative_costs_[i - 1];
			}
			for(int i = route_.size() - 2; i >= 0; --i) {
				rev_cummulative_costs_[i] += rev_cummulative_costs_[i + 1];
			}
			rev_cummulative_costs_.push_back(0);
		}


		double TwoOptSwap(size_t i, size_t k, bool has_depot = false) {
			/* std::cout << i << " " << k << std::endl; */
			double cost = 0, partb_cost = 0, partc_cost = 0;
			size_t link_ab_u, link_ab_v, link_bc_u, link_bc_v, start_vertex, end_vertex;

			if( i == k and route_vector_[k]->GetReq() != kIsRequired ) {
				return cummulative_costs_[m_ - 1];
			}
			if(i == 0 and k == m_ - 1) {
				return rev_cummulative_costs_[0];
			}
			if(has_depot == false and i == 1 and route_vector_[i - 1]->GetReq() != kIsRequired and k == m_ - 1) {
				g_->GetVertexIndex(route_vector_[0]->GetHeadVertexID(), link_ab_u);
				g_->GetVertexIndex(route_vector_[0]->GetTailVertexID(), link_ab_v);
				return rev_cummulative_costs_[i] + apsp_->GetCost(link_ab_u, link_ab_v);
			}

			if(i != 0) {
				if(route_vector_[i - 1]->GetReq() == kIsRequired) {
					cost = cummulative_costs_[i - 1];
					g_->GetVertexIndex(route_vector_[i - 1]->GetHeadVertexID(), link_ab_u);
				} else {
					if(i > 1) {
						cost = cummulative_costs_[i - 2];
						g_->GetVertexIndex(route_vector_[i - 2]->GetHeadVertexID(), link_ab_u);
					}
				}
			}
			if(i == 0 and has_depot) {
				g_->GetVertexIndex(route_vector_[0]->GetTailVertexID(), link_ab_u);
			}

			g_->GetVertexIndex(route_vector_[0]->GetTailVertexID(), start_vertex);
			/* std::cout << "parta: " << cost << std::endl; */

			partb_cost = rev_cummulative_costs_[i] - rev_cummulative_costs_[k + 1];
			/* std::cout << "partb: " << partb_cost << std::endl; */
			if(route_vector_[k]->GetReq() != kIsRequired) {
				partb_cost -= route_vector_[k]->GetReverseDeadheadCost();
				g_->GetVertexIndex(route_vector_[k]->GetTailVertexID(), link_ab_v);
			} else {
				g_->GetVertexIndex(route_vector_[k]->GetHeadVertexID(), link_ab_v);
			}

			/* std::cout << "partb1: " << partb_cost << std::endl; */
			if((i == 0 or (i == 1 and route_vector_[i - 1]->GetReq() != kIsRequired)) and has_depot == false) {
				start_vertex = link_ab_v;
				cost = partb_cost;
			} else {
				/* std::cout << "links: " << link_ab_u << " " << link_ab_v << std::endl; */
				cost += partb_cost + apsp_->GetCost(link_ab_u, link_ab_v);
			}

			/* std::cout << "partb2: " << cost << std::endl; */

			if(route_vector_[i]->GetReq() == kIsRequired or (i <= 1 and has_depot)) {
				g_->GetVertexIndex(route_vector_[i]->GetTailVertexID(), link_bc_u);
			} else {
				cost -= route_vector_[i]->GetReverseDeadheadCost();
				g_->GetVertexIndex(route_vector_[i]->GetHeadVertexID(), link_bc_u);
			}
			/* std::cout << "partb3: " << cost << std::endl; */

			if(k == m_ - 1 or (k == m_ - 2 and route_vector_[k + 1]->GetReq() != kIsRequired)) {
				cost += apsp_->GetCost(link_bc_u, start_vertex);
				return cost;
			}

			partc_cost = cummulative_costs_[m_ - 1] - cummulative_costs_[k];
			/* std::cout << "partc: " << partc_cost << std::endl; */
			if(route_vector_[k + 1]->GetReq() != kIsRequired) {
				partc_cost -= route_vector_[k + 1]->GetDeadheadCost();
				g_->GetVertexIndex(route_vector_[k + 2]->GetTailVertexID(), link_bc_v);
			} else {
				g_->GetVertexIndex(route_vector_[k + 1]->GetTailVertexID(), link_bc_v);
			}
			cost += partc_cost + apsp_->GetCost(link_bc_u, link_bc_v);
			/* std::cout << "partc + dd: " << cost << " " << link_bc_u << " " << link_bc_v<< std::endl; */

			if(route_vector_[m_ - 1]->GetReq() != kIsRequired) {
				cost -= route_vector_[m_ - 1]->GetDeadheadCost();
				g_->GetVertexIndex(route_vector_[m_ - 1]->GetTailVertexID(), end_vertex);
			} else {
				g_->GetVertexIndex(route_vector_[m_ - 1]->GetHeadVertexID(), end_vertex);
			}
			cost += apsp_->GetCost(end_vertex, start_vertex);
			/* std::cout << "final: " << cost << " " << end_vertex << " " << start_vertex<< std::endl; */
			return cost;
		}

		void TwoOpt(bool has_depot = false) {
			m_ = route_.size();
			bool is_improved = true;
			double best_cost = GetCost();
			local_moves_count_ = 0;
			size_t n = g_->GetN();
			size_t max_moves = n * n * n;
			std::cout << "Route size: " << m_ << std::endl;

			cummulative_costs_.clear();
			rev_cummulative_costs_.clear();
			cummulative_costs_.resize(m_, 0);
			rev_cummulative_costs_.resize(m_, 0);

			while(is_improved == true and local_moves_count_ <= max_moves) {
				m_ = route_.size();
				cummulative_costs_.clear();
				rev_cummulative_costs_.clear();
				cummulative_costs_.resize(m_, 0);
				rev_cummulative_costs_.resize(m_, 0);

				ComputeCummulativeCosts();
				is_improved = false;
				route_vector_.clear();
				route_vector_.reserve(m_);
				for(const auto &edge:route_){
					route_vector_.push_back(&edge);
				}
				for(size_t i = 0; i < (m_ - 1) and is_improved == false; ++i) {
					for(size_t k = i; k < m_ and is_improved == false; ++k) {
						/* std::cout << i << " " << k << std::endl; */
						++local_moves_count_;
						auto new_cost = TwoOptSwap(i, k, has_depot);
						if(new_cost < best_cost) {
							best_cost = new_cost;
							is_improved = true;
							TwoOptAux(i, k);
							RouteImprovement();
						}
					}
				}
			} while(is_improved == true);
			std::cout << "No. of local moves: " << local_moves_count_ << std::endl;
		}

		void TwoOptAux(const size_t ii, const size_t kk) {
			size_t j = 0;
			auto it = GetRouteStart();
			/* std::cout << it->GetTailVertexID() << " " << it->GetHeadVertexID() << ii << " " << kk<< std::endl; */
			RouteEdges new_route;
			while (j <= ii - 1 and ii != 0) {
				/* std::cout << j << " " << ii<< std::endl; */
				new_route.push_back(*it);
				++it; ++j;
				/* std::cout << it->GetTailVertexID() << " " << it->GetHeadVertexID() << " 1 " << ii << " " << kk<< std::endl; */
			}

			auto new_route_it = new_route.cend();
			for(j = ii; j <= kk; ++j) {
				Edge new_edge = *it;
				new_edge.Reverse();
				if(new_edge.GetReq() == kIsRequired) {
					new_edge.SetCost(new_edge.GetServiceCost());
				} else {
					new_edge.SetCost(new_edge.GetDeadheadCost());
				}
				new_route_it = new_route.insert(new_route_it, new_edge);
				++it;
				/* std::cout << it->GetTailVertexID() << " " << it->GetHeadVertexID() << " 3 " << ii << " " << kk<< std::endl; */
			}
			for(j = kk + 1; j < m_; ++j) {
				new_route.push_back(*it);
				++it;
				/* std::cout << it->GetTailVertexID() << " " << it->GetHeadVertexID() << " 3 " << ii << " " << kk<< std::endl; */
			}
			route_ = std::move(new_route);
			ConnectRoute();
		}

		void ConnectRoute() {
			auto it = GetRouteStart();
			if(it == GetRouteEnd()) {
				return;
			}
			size_t u, v;
			size_t u_id, v_id;
			if(route_.size() == 1) {
				Edge e = route_.front();
				u = e.GetTailVertexID(); v = e.GetHeadVertexID();
				g_->GetVertexIndex(u, u_id);
				g_->GetVertexIndex(v, v_id);
				std::vector <Edge> edge_list;
				if(g_->IsDepotSet()) {
				}
				apsp_->GetPath(edge_list, u_id, v_id);
				for(const auto &e:edge_list) {
					AddEdge(e);
				}
			}
			auto next_it = std::next(it);
			while (next_it != GetRouteEnd()) {
				u = it->GetHeadVertexID();
				v = next_it->GetTailVertexID();
				if(u != v) {
					g_->GetVertexIndex(u, u_id);
					g_->GetVertexIndex(v, v_id);
					std::vector <Edge> edge_list;
					apsp_->GetPath(edge_list, u_id, v_id);
					for(const auto &e:edge_list) {
						AddEdge(e, next_it);
					}
				}
				it = next_it;
				next_it = std::next(it);
			}
			Edge e_front = route_.front();
			Edge e_back = route_.back();
			u = e_back.GetHeadVertexID(); v = e_front.GetTailVertexID();
			if(u != v) {
				g_->GetVertexIndex(u, u_id);
				g_->GetVertexIndex(v, v_id);
				std::vector <Edge> edge_list;
				apsp_->GetPath(edge_list, u_id, v_id);
				for(const auto &e:edge_list) {
					AddEdge(e);
				}
			}
		}

		void Shortcut(RouteEdges::const_iterator start_it, RouteEdges::const_iterator end_it, RouteEdges::const_iterator it,  double cost) {
			size_t t, h;
			g_->GetVertexIndex(start_it->GetTailVertexID(), t);
			g_->GetVertexIndex(end_it->GetHeadVertexID(), h);
			auto dd_cost = apsp_->GetCost(t, h);
			RouteEdges::const_iterator insert_it;
			if(dd_cost < cost) {
				if(it != route_.cend())
					insert_it = route_.erase(start_it, it);
				else {
					insert_it = route_.erase(start_it, route_.cend());
				}
				std::vector <Edge> edge_list;
				apsp_->GetPath(edge_list, t, h);
				for (auto &new_edge:edge_list) {
					AddEdge(new_edge, insert_it);
				}
			}
		}

		void RouteImprovement() {
			auto start_it = route_.cbegin();
			auto end_it = route_.cbegin();
			bool start_flag = false;
			double cost = 0;
			bool at_end = false;
			for(auto it = route_.cbegin(); it != route_.cend(); ++it) {
				if(at_end)
					break;
				auto next_it = it; ++next_it;
				auto e = *it;
				if (e.GetReq()) {
					if(start_flag) {
						Shortcut(start_it, end_it, it, cost);
					}
					start_flag = false;
					cost = 0;
				}
				else {
					if(!start_flag) {
						start_flag = true;
						start_it = it;
						end_it = it;
						cost += e.GetCost();
					}
					else {
						end_it = it;
						cost += e.GetCost();
					}
					if(next_it == route_.cend()) {
						at_end = true;
						Shortcut(start_it, end_it, next_it, cost);
					}
				}
			}
			if(route_.front().GetReq() != kIsRequired and route_.back().GetReq() != kIsRequired and route_.size() > 1) {
				auto front_index = route_.cbegin();
				for(auto it = route_.cbegin(); it != route_.cend(); ++it) {
					auto e = *it;
					front_index = it;
					if(e.GetReq()) {
						break;
					}
				}
				auto back_index = route_.cbegin();
				for(auto it = std::prev(route_.cend()); it != route_.cbegin(); --it) {
					auto e = *it;
					if(e.GetReq()) {
						back_index = std::next(it);
						break;
					}
				}
				size_t t , h;
				/* std::cout << "-------------\n"; */
				/* PrintRoute(); */
				g_->GetVertexIndex((*std::prev(back_index)).GetHeadVertexID(), t);
				g_->GetVertexIndex((*front_index).GetTailVertexID(), h);
				route_.erase(route_.begin(), front_index);
				route_.erase(back_index, route_.end());
				/* PrintRoute(); */
				std::vector <Edge> edge_list;
				apsp_->GetPath(edge_list, t, h);
				/* std::cout << t << " ... " << h << std::endl; */
				for (auto &new_edge:edge_list) {
					AddEdge(new_edge);
				}
				/* PrintRoute(); */
				/* std::cout << "-------------\n"; */
			}
		}

		void GenerateEdgeList(std::vector <Edge> &edge_list) {
			for (auto &e:route_) {
				edge_list.push_back(e);
			}
		}

		void RotateToDepot(size_t depot_id) {
			auto n_begin = route_.begin();
			for(auto it = route_.begin(); it != route_.end(); ++it) {
				auto e = *it;
				if(e.GetTailVertexID() == depot_id) {
					n_begin = it;
					break;
				}
			}
			std::rotate(route_.begin(), n_begin, route_.end());
		}

		/*! Write edge data to file */
		void WriteRouteData(std::string filename) const {
			std::ofstream out_file (filename);
			out_file.precision(16);
			double cost = 0;
			for (auto &e:route_) {
				cost += e.GetCost();
				out_file << e.GetTailVertexID() << " " << e.GetHeadVertexID() << " " << e.GetReq()<< " " << cost;
				out_file<<"\n";

			}
			out_file.close();
		}

		/*! Write edge data to file */
		void WriteRouteEdgeData(std::string filename) const {
			std::ofstream out_file (filename);
			out_file.precision(16);
			double cost = 0;
			for (auto &e:route_) {
				cost += e.GetCost();
				double x, y;
				e.GetTailVertexXY(x, y);
				out_file << x << " " << y << " ";
				e.GetHeadVertexXY(x, y);
				out_file << x << " " << y << " " << e.GetReq() << " " << cost;
				out_file<<"\n";

			}
			out_file.close();
		}

		/*! Append waypoints to file */
		void WriteWayPoints(std::string filename) const {
			std::ofstream out_file (filename);
			out_file.precision(16);
			double cost = 0;
			double x, y;
			for (const auto &e:route_) {
				cost += e.GetCost();
				e.GetTailVertexXY(x, y);
				out_file << x << " " << y << " " << e.GetReq();
				out_file<<"\n";

			}
			auto e = route_.back();
			e.GetHeadVertexXY(x, y);
			out_file << x << " " << y << " " << e.GetReq();
			out_file.close();
		}

		void WritePlacemarkKML(std::ofstream &out_file, const size_t count, const Vertex *v) const {
			double lla[3];
			v->GetLLA(lla);
			out_file << "<Placemark>\n";
			out_file << "<name>" << count << "</name>\n";
			out_file << "<description>" << count<< "</description>\n";
			out_file << "<Point>\n";
			out_file << "<coordinates>" << lla[1] << "," << lla[0]  <<"</coordinates>\n";
			out_file << "</Point>\n";
			out_file << "</Placemark>\n";
		}

		void WriteKML(std::string file_name) const {
			std::ofstream out_file (file_name);
			out_file.precision(16);
			std::string init_string = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n <kml xmlns=\"http://www.opengis.net/kml/2.2\">\n <Document>\n";
			out_file << init_string;
			size_t count = 1;
			auto e = route_.front();
			const Vertex *t, *h;
			e.GetVertices(t, h);
			WritePlacemarkKML(out_file, count, t);
			++count;
			for (auto it = route_.begin(); it != route_.end(); ++it){
				auto e = *it;
				e.GetVertices(t, h);
				if(t!=nullptr and h!=nullptr) {
					WritePlacemarkKML(out_file, count, h);
					++count;
				}
			}
			out_file << "</Document>\n </kml>";
			out_file.close();
		}
		void WriteKMLReverse(std::string file_name) const {
			std::ofstream out_file (file_name);
			out_file.precision(16);
			std::string init_string = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n <kml xmlns=\"http://www.opengis.net/kml/2.2\">\n <Document>\n";
			out_file << init_string;
			size_t count = 1;
			auto e = route_.front();
			const Vertex *t, *h;
			e.GetVertices(t, h);
			WritePlacemarkKML(out_file, count, t);
			++count;
			for (auto it = route_.rbegin(); it != route_.rend(); ++it){
				auto e = *it;
				e.GetVertices(t, h);
				if(t!=nullptr and h!=nullptr) {
					WritePlacemarkKML(out_file, count, t);
					++count;
				}
			}
			out_file << "</Document>\n </kml>";
			out_file.close();
		}

		double CostCompare(const EdgeCost &edge_cost_computer) {
			double cost = 0, cost_rev = 0;
			double total_cost = 0;
			for(const auto &e:route_){
				if(e.GetReq() == kIsRequired) {
					edge_cost_computer.ComputeServiceCost(e, cost, cost_rev);
					total_cost += cost;
				} else {
					edge_cost_computer.ComputeDeadheadCost(e, cost, cost_rev);
					total_cost += cost;
				}
			}
			return total_cost;
		}

		size_t GetNumTurns() const{
			return (route_.size() - 1);
		}

	};
} // namespace lclibrary

#endif /* LCLIBRARY_CORE_ROUTE_H_ */

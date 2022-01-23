/**
 * This file is part of the LineCoverage-library.
 * The file contains MEM algorithm for SLC
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

#ifndef LCLIBRARY_SLC_MEM_H_
#define LCLIBRARY_SLC_MEM_H_

#include <lclibrary/core/core.h>
#include <lclibrary/core/mem_base.h>
#include <lclibrary/utils/utils.h>
#include <lclibrary/algorithms/algorithms.h>
#include <lclibrary/slc/slc_base.h>
#include <vector>
#include <algorithm>
#include <memory>
#include <initializer_list>

namespace lclibrary {

	class SLC_MEM : public MEM_Base, public SLC_Base  {
		size_t n_, m_;
		size_t v0;
		std::vector <MEM_Route*> route_list_;
		std::shared_ptr <APSP_FloydWarshall> apsp_;

		public:
		SLC_MEM(std::shared_ptr <const Graph> g_in) : MEM_Base(), SLC_Base(g_in) {
			apsp_ = std::make_shared <APSP_FloydWarshall>(g_);
			apsp_->APSP_Deadheading();
		}

		~SLC_MEM() {
			route_list_.clear();
		}

		int Solve() {
			n_ = g_->GetN();
			m_ = g_->GetM();
			if(InitializeRoutes()) {
				std::cerr << "MEM initialization failed\n";
				return kFail;
			}
			MEM();
			GenerateRoute();
			route_ = EulerTourGeneration(sol_digraph_);
			route_.SetGraphAPSP(g_, apsp_);
			route_.CheckRoute();
			std::cout << "Route cost: " << route_.GetCost() << std::endl;
			route_.RouteImprovement();
			std::cout << "Route cost after improvement: " << route_.GetCost() << std::endl;
			if(use_2opt_ == true) {
				route_.TwoOpt();
			}
			std::cout << "Route cost after 2opt: " << route_.GetCost() << std::endl;
			if(g_->IsDepotSet()) {
				std::cout << "MEM: rotating to depot\n";
				route_.RotateToDepot(g_->GetDepot());
			}
			route_.CheckRoute();
			sol_digraph_->ClearAllEdges();
			std::vector <Edge> edge_list;
			route_.GenerateEdgeList(edge_list);
			sol_digraph_->AddEdge(edge_list);
			return 0;
		}
		bool InitializeRoutes() {
			v0 = g_->GetDepot();
			size_t t, h;

			for(size_t i = 0; i < m_; ++i) {
				auto e = g_->GetEdge(i, kIsRequired);
				g_->GetVerticesIndexOfEdge(i, t, h, kIsRequired);
				double c_s = g_->GetServiceCost(i);
				double c_s_rev = g_->GetReverseServiceCost(i);
				double cost1 = apsp_->GetCost(v0, t) + c_s + apsp_->GetCost(h, v0);
				double cost2 = apsp_->GetCost(v0, h) + c_s_rev + apsp_->GetCost(t, v0);
				MEM_Route *route = new MEM_Route();
				route->reversed_ = false;
				if(cost1 < cost2) {
					route->cost_ = cost1;
					route->req_cost_ = c_s; route->req_cost_rev_ = c_s_rev;
					route->edge_ = MakeEdgeTuple(e, false);
					route->vertex_idx_start_ = t; route->vertex_idx_end_ = h;
				}
				else {
					route->cost_ = cost2;
					route->req_cost_ = c_s_rev; route->req_cost_rev_ = c_s;
					route->edge_ = MakeEdgeTuple(e, true);
					route->vertex_idx_start_ = h; route->vertex_idx_end_ = t;
				}
				if(route->cost_ > 1e+300) {
					std::cout << route->cost_ << " MEM error\n";
					std::cout << cost1 << " " << cost2 << std::endl;
					std::cout << v0 << " " << t << " " << h << std::endl;
					return kFail;
				}
				route_list_.push_back(route);
			}
			return kSuccess;
		}

		bool ComputeSavings(RouteSavings &rs) {
			size_t p = rs.p_; size_t q = rs.q_;
			double savings = 0;
			const MEM_Route* r_p = route_list_[p]; const MEM_Route* r_q = route_list_[q];
			size_t i, j; size_t l, m;
			r_p->GetVertices(i, j); r_q->GetVertices(l, m);
			double cost_pq = r_p->cost_ + r_q->cost_;
			double savings_list[8];
			double ij = r_p->req_cost_; double ji = r_p->req_cost_rev_;
			double lm = r_q->req_cost_; double ml = r_q->req_cost_rev_;
			savings_list[0] = apsp_->GetCost(v0, i) + ij + apsp_->GetCost(j, m) + ml + apsp_->GetCost(l, v0);
			savings_list[1] = apsp_->GetCost(v0, i) + ij + apsp_->GetCost(j, l) + lm + apsp_->GetCost(m, v0);
			savings_list[2] = apsp_->GetCost(v0, j) + ji + apsp_->GetCost(i, l) + lm + apsp_->GetCost(m, v0);
			savings_list[3] = apsp_->GetCost(v0, j) + ji + apsp_->GetCost(i, m) + ml + apsp_->GetCost(l, v0);

			savings_list[4] = apsp_->GetCost(v0, l) + lm + apsp_->GetCost(m, j) + ji + apsp_->GetCost(i, v0);
			savings_list[5] = apsp_->GetCost(v0, m) + ml + apsp_->GetCost(l, j) + ji + apsp_->GetCost(i, v0);
			savings_list[6] = apsp_->GetCost(v0, m) + ml + apsp_->GetCost(l, i) + ij + apsp_->GetCost(j, v0);
			savings_list[7] = apsp_->GetCost(v0, l) + lm + apsp_->GetCost(m, i) + ij + apsp_->GetCost(j, v0);

			savings = cost_pq - savings_list[0];
			size_t savings_perm = 0;
			for(size_t perm = 1; perm < 8; ++perm) {
				savings_list[perm] = cost_pq - savings_list[perm];
				if(savings_list[perm] > savings) {
					savings = savings_list[perm];
					savings_perm = perm;
				}
			}
			rs.savings_ = savings;
			rs.savings_perm_ = savings_perm;
			return kSuccess;
		}

		bool IsTourEmpty(const size_t p) {
			if(route_list_[p] == nullptr)
				return true;
			else
				return false;
		}

		size_t NumOfRoutes() {
			return route_list_.size();
		}

		void Merge(const RouteSavings &rs) {
			size_t p = rs.p_; size_t q = rs.q_;
			size_t savings_perm = rs.savings_perm_;
			double savings = rs.savings_;
			MEM_Route *r = new MEM_Route();
			size_t i, j; size_t l, m;
			auto r_p = route_list_[p]; auto r_q = route_list_[q];
			r_p->GetVertices(i, j);
			r_q->GetVertices(l, m);

			double ij = r_p->req_cost_; double ji = r_p->req_cost_rev_;
			double lm = r_q->req_cost_; double ml = r_q->req_cost_rev_;
			if(savings_perm == 0) {
				r_p->reversed_ = false; r_q->reversed_ = true;
				r->route1_ = r_p; r->route2_ = r_q;
				r->vertex_idx_start_ = r_p->vertex_idx_start_;
				r->vertex_idx_end_ = r_q->vertex_idx_start_;
				r->req_cost_ = ij + apsp_->GetCost(j, m) + ml;
				r->req_cost_rev_ = lm + apsp_->GetCost(m, j) + ji;
				r->cost_ = apsp_->GetCost(v0, i) + r->req_cost_ + apsp_->GetCost(l, v0);
			}
			else if(savings_perm == 1) {
				r_p->reversed_ = false; r_q->reversed_ = false;
				r->route1_ = r_p; r->route2_ = r_q;
				r->vertex_idx_start_ = r_p->vertex_idx_start_;
				r->vertex_idx_end_ = r_q->vertex_idx_end_;
				r->req_cost_ = ij + apsp_->GetCost(j, l) + lm;
				r->req_cost_rev_ = ml + apsp_->GetCost(l, j) + ji;
				r->cost_ = apsp_->GetCost(v0, i) + r->req_cost_ + apsp_->GetCost(m, v0);
			}
			else if(savings_perm == 2) {
				r_p->reversed_ = true; r_q->reversed_ = false;
				r->route1_ = r_p; r->route2_ = r_q;
				r->vertex_idx_start_ = r_p->vertex_idx_end_;
				r->vertex_idx_end_ = r_q->vertex_idx_end_;
				r->req_cost_ = ji + apsp_->GetCost(i, l) + lm;
				r->req_cost_rev_ = ml + apsp_->GetCost(l, i) + ij;
				r->cost_ = apsp_->GetCost(v0, j) + r->req_cost_ + apsp_->GetCost(m, v0);
			}
			else if(savings_perm == 3) {
				r_p->reversed_ = true; r_q->reversed_ = true;
				r->route1_ = r_p; r->route2_ = r_q;
				r->vertex_idx_start_ = r_p->vertex_idx_end_;
				r->vertex_idx_end_ = r_q->vertex_idx_start_;
				r->req_cost_ = ji + apsp_->GetCost(i, m) + ml;
				r->req_cost_rev_ = lm + apsp_->GetCost(m, i) + ij;
				r->cost_ = apsp_->GetCost(v0, j) + r->req_cost_ + apsp_->GetCost(l, v0);
			}
			else if(savings_perm == 4) {
				r_p->reversed_ = true; r_q->reversed_ = false;
				r->route1_ = r_q; r->route2_ = r_p;
				r->vertex_idx_start_ = r_q->vertex_idx_start_;
				r->vertex_idx_end_ = r_p->vertex_idx_start_;
				r->req_cost_ = lm + apsp_->GetCost(m, j) + ji;
				r->req_cost_rev_ = ij + apsp_->GetCost(j, m) + ml;
				r->cost_ = apsp_->GetCost(v0, l) + r->req_cost_ + apsp_->GetCost(i, v0);
			}
			else if(savings_perm == 5) {
				r_p->reversed_ = true; r_q->reversed_ = true;
				r->route1_ = r_q; r->route2_ = r_p;
				r->vertex_idx_start_ = r_q->vertex_idx_end_;
				r->vertex_idx_end_ = r_p->vertex_idx_start_;
				r->req_cost_ = ml + apsp_->GetCost(l, j) + ji;
				r->req_cost_rev_ = ij + apsp_->GetCost(j, l) + lm;
				r->cost_ = apsp_->GetCost(v0, m) + r->req_cost_ + apsp_->GetCost(i, v0);
			}
			else if(savings_perm == 6) {
				r_p->reversed_ = false; r_q->reversed_ = true;
				r->route1_ = r_q; r->route2_ = r_p;
				r->vertex_idx_start_ = r_q->vertex_idx_end_;
				r->vertex_idx_end_ = r_p->vertex_idx_end_;
				r->req_cost_ = ml + apsp_->GetCost(l, i) + ij;
				r->req_cost_rev_ = ji + apsp_->GetCost(i, l) + lm;
				r->cost_ = apsp_->GetCost(v0, m) + r->req_cost_ + apsp_->GetCost(j, v0);
			}
			else if(savings_perm == 7) {
				r_p->reversed_ = false; r_q->reversed_ = false;
				r->route1_ = r_q; r->route2_ = r_p;
				r->vertex_idx_start_ = r_q->vertex_idx_start_;
				r->vertex_idx_end_ = r_p->vertex_idx_end_;
				r->req_cost_ = lm + apsp_->GetCost(m, i) + ij;
				r->req_cost_rev_ = ji + apsp_->GetCost(i, m) + ml;
				r->cost_ = apsp_->GetCost(v0, l) + r->req_cost_ + apsp_->GetCost(j, v0);
			}
			if(abs(savings - (r_p->cost_ + r_q->cost_ - r->cost_)) > 1e-10) {
				std::cerr << "Mismatch savings and merge: " << savings << " " << r_p->cost_ + r_q->cost_ - r->cost_ << " " <<savings_perm <<"\n";
				std::cerr << r_p->cost_ << " " << r_q->cost_ << " "  << r->cost_ << std::endl;

			}
			r->reversed_ = false;
			route_list_.push_back(r);
			route_list_[p] = nullptr; route_list_[q] = nullptr;
		}

		void GenerateRoute() {
			std::vector <Vertex> vertex_list;
			for(size_t i = 0; i < n_; ++i) {
				Vertex v;
				g_->GetVertexData(i, v);
				vertex_list.push_back(v);
			}
			std::vector <Edge> edge_list;
			auto r = route_list_.back();
			size_t t, h;
			r->GetVertices(t, h);

			apsp_->GetPath(edge_list, v0, t);
			GetPath(edge_list, r);
			apsp_->GetPath(edge_list, h, v0);
			sol_digraph_ = std::make_shared <Graph>(vertex_list, edge_list);
		}

		void GetPath(std::vector <Edge> &edge_list, MEM_Route* r) {
			auto r1 = r->route1_;
			auto r2 = r->route2_;
			if(r1 == nullptr and r2 == nullptr) {
				Edge e = *(std::get<0>(r->edge_));
				bool rev = std::get<1>(r->edge_);
				rev = rev xor r->reversed_;
				if(rev) {
					e.SetCost(e.GetReverseServiceCost());
					e.Reverse();
				}
				else {
					e.SetCost(e.GetServiceCost());
				}
				edge_list.push_back(e);
			}
			else {
				size_t r1_t, r1_h; size_t r2_t, r2_h;
				r1->GetVertices(r1_t, r1_h);
				r2->GetVertices(r2_t, r2_h);

				if(r->reversed_) {
					r1->XOR(r->reversed_);
					r2->XOR(r->reversed_);
					GetPath(edge_list, r2);
					apsp_->GetPath(edge_list, r2_t, r1_h);
					GetPath(edge_list, r1);
				}
				else {
					GetPath(edge_list, r1);
					apsp_->GetPath(edge_list, r1_h, r2_t);
					GetPath(edge_list, r2);
				}
			}
			delete r;
		}

	};

}
#endif /* LCLIBRARY_SLC_MEM_H_ */

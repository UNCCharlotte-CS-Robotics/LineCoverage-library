/**
 * This file is part of the LineCoverage-library.
 * The file contains tour splitting algrorithm for mulit depot MLC
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

#ifndef LCLIBRARY_MLC_TOUR_SPLITTING_MD_H_
#define LCLIBRARY_MLC_TOUR_SPLITTING_MD_H_

#include <lclibrary/core/core.h>
#include <lclibrary/utils/utils.h>
#include <lclibrary/algorithms/algorithms.h>
#include <lclibrary/slc/slc_beta2_atsp.h>
#include <lclibrary/mlc/mlc_base.h>

#include <iostream>

namespace lclibrary {

	struct Sequence {
		double cost_, rcost_;
		double demand_, rdemand_;
		size_t t_, h_;
	};
	struct Tour {
		double cost_;
		double demand_;
		size_t depot_;
		size_t l_, perm_, r_;
		bool og_ = true;
	};
	class MLC_TS_MD : public MLC_Base {
		private:
			size_t n_, m_, K_;
			APSP_FloydWarshall *apsp_;
			double capacity_ = 0;
			std::vector <size_t> depots_;
			std::vector <Edge> req_edges_;
			std::vector < std::vector <Sequence>> sequence_;
			std::vector < std::vector <std::vector<Tour>>> tours_;
			Route route_;
			std::vector <Vertex> vertex_list_;

		public:
			MLC_TS_MD(const std::shared_ptr <const Graph> g_in) : MLC_Base(g_in) {
				std::cout << "Start MLC TS MD\n";
				apsp_ = new APSP_FloydWarshall(g_, true);
				apsp_->APSP_Deadheading();
				n_ = g_->GetN();
				m_ = g_->GetM();
				capacity_ = g_->GetCapacity();
				g_->GetDepotIDs(depots_);
				K_ = depots_.size();

				sequence_.resize(m_, std::vector <Sequence> (m_));
				for(size_t i = 0; i < m_; ++i) {
					std::vector <std::vector <Tour>> tours;
					tours.resize(m_, std::vector <Tour> (K_));
					tours_.push_back(tours);
				}
				std::cout << "Done MLC TS MD\n";
			}

			~MLC_TS_MD() {
				delete apsp_;
			}

			int Solve() {
				for(size_t i = 0; i < n_; ++i) {
					Vertex v;
					g_->GetVertexData(i, v);
					vertex_list_.push_back(v);
				}
				std::cout << "Solve MLC TS MD\n";
				SLC_Beta2ATSP slc_beta2_atsp(g_);
				bool use_2opt = true;
				slc_beta2_atsp.Use2Opt(use_2opt);
				std::cout << std::boolalpha;
				slc_beta2_atsp.Solve();
				std::cout << "beta2_atsp Solution check: " << slc_beta2_atsp.CheckSolution() << std::endl;
				std::vector <Edge> edge_list;
				slc_beta2_atsp.GetRoute(route_);
				route_.GenerateEdgeList(edge_list);
				for(auto &e:edge_list) {
					if(e.GetReq()){
						req_edges_.push_back(e);
					}
				}
				Initialize();
				DP();
				std::cout << "Initialization done\n";
				GetTourEdges(0, m_ - 1);
				for(auto &sol_digraph:sol_digraph_list_) {
					sol_digraph->PrintNM();
					auto route = EulerTourGeneration(sol_digraph);
					route.CheckRoute();
					if(sol_digraph->IsDepotSet()) {
						route.RotateToDepot(sol_digraph->GetDepot());
					}
					std::cout << "Route cost: " << route.GetCost() << std::endl;
					/* route_.RouteImprovement(g_, apsp_); */
					/* std::cout << "Route cost after improvement: " << route_.GetCost() << std::endl; */
					/* route = route.TwoOpt(g_, apsp_, g_->IsDepotSet()); */
					std::cout << "Route cost after two opt: " << route.GetCost() << std::endl;
					/* new_route.RouteImprovement(g_, apsp_); */
					/* route = new_route; */
					route.CheckRoute();
					sol_digraph->ClearAllEdges();
					std::vector <Edge> edge_list;
					route.GenerateEdgeList(edge_list);
					sol_digraph->AddEdge(edge_list);
					sol_digraph->PrintNM();
					route_list_.push_back(route);
				}
				std::cout << "Solve end MLC TS MD\n";
				return 0;
			}

			void Initialize() {
				for(int i = (int)(m_ - 1); i >= 0 ; --i) {
					sequence_[i][i].cost_ = req_edges_[i].GetServiceCost();
					sequence_[i][i].rcost_ = req_edges_[i].GetReverseServiceCost();
					sequence_[i][i].demand_ = req_edges_[i].GetServiceDemand();
					sequence_[i][i].rdemand_ = req_edges_[i].GetReverseServiceDemand();
					sequence_[i][i].t_ = req_edges_[i].GetTailVertexID();
					sequence_[i][i].h_ = req_edges_[i].GetHeadVertexID();
					for(size_t j = i + 1; j < m_; ++j) {
						sequence_[i][j].cost_ = sequence_[i][i].cost_ + GetSPCost(sequence_[i][i].h_, sequence_[i + 1][j].t_) + sequence_[i + 1][j].cost_;
						sequence_[i][j].rcost_ = sequence_[i + 1][j].rcost_ + GetSPCost(sequence_[i + 1][j].t_, sequence_[i][i].h_) + sequence_[i][i].rcost_;
						sequence_[i][j].demand_ = sequence_[i][i].demand_ + GetSPDemand(sequence_[i][i].h_, sequence_[i + 1][j].t_) + sequence_[i + 1][j].demand_;
						sequence_[i][j].rdemand_ = sequence_[i + 1][j].rdemand_ + GetSPDemand(sequence_[i + 1][j].t_, sequence_[i][i].h_) + sequence_[i][i].rdemand_;
						sequence_[i][j].t_ = sequence_[i][i].t_;
						sequence_[i][j].h_ = sequence_[i + 1][j].h_;
						/* std::cout << "Sequence: " << sequence_[i][j].cost_ << std::endl; */
					}
				}
				for(int i = (int)(m_ - 1); i >= 0; --i) {
					for(size_t j = i; j < m_; ++j) {
						for(size_t k = 0; k < K_; ++k) {
							tours_[i][j][k].demand_ = kDoubleMax;
							tours_[i][j][k].cost_ = kDoubleMax;
							tours_[i][j][k].depot_ = depots_[k];
						}
					}
				}
			}

			void DP() {
				for(size_t k = 0; k < K_; ++k) {
					for(int i = (int)(m_ - 1); i >= 0 ; --i) {
						tours_[i][i][k].cost_ = kDoubleMax;
						tours_[i][i][k].demand_ = kDoubleMax;
						double c = GetSPCost(depots_[k], sequence_[i][i].t_) +  sequence_[i][i].cost_ + GetSPCost(sequence_[i][i].h_, depots_[k]);
						double crev = GetSPCost(depots_[k], sequence_[i][i].h_) +  sequence_[i][i].rcost_ + GetSPCost(sequence_[i][i].t_, depots_[k]);
						double d = GetSPDemand(depots_[k], sequence_[i][i].t_) +  sequence_[i][i].rdemand_ + GetSPDemand(sequence_[i][i].h_, depots_[k]);
						double drev = GetSPDemand(depots_[k], sequence_[i][i].h_) +  sequence_[i][i].rdemand_ + GetSPDemand(sequence_[i][i].t_, depots_[k]);
						if((c <= crev or drev > capacity_) and d <= capacity_) {
							tours_[i][i][k].cost_ = c; tours_[i][i][k].demand_ = d;
							tours_[i][i][k].l_ = 0;
						} else {
							tours_[i][i][k].cost_ = crev; tours_[i][i][k].demand_ = drev;
							tours_[i][i][k].l_ = 1;
						}
						for(size_t j = i + 1; j < m_; ++j) {
							auto depot = depots_[k];
							size_t perm; double cost, demand;
							size_t count = 0;
							cost = GetSPCost(depot, sequence_[i][j].t_) + sequence_[i][j].cost_ + GetSPCost(sequence_[i][j].h_, depot);
							demand = GetSPDemand(depot, sequence_[i][j].t_) + sequence_[i][j].demand_ + GetSPDemand(sequence_[i][j].h_, depot);
							if(demand <= capacity_) {
								tours_[i][j][k].cost_ = cost; tours_[i][j][k].demand_ = demand;
								tours_[i][j][k].l_ = 0;
							}
							++count;
							for(size_t l = i; l < j; ++l) {
								if(GetOptimalPerm(depot, sequence_[i][l], sequence_[l + 1][j], cost, demand, perm) == kSuccess) {
									if(demand <= capacity_ and cost < tours_[i][j][k].cost_) {
										tours_[i][j][k].cost_ = cost; tours_[i][j][k].demand_ = demand;
										tours_[i][j][k].l_ = count; tours_[i][j][k].perm_ = perm;
									}
								}
								++count;
							}
							/* std::cout << "count: " << i << " " << j << " " << count << std::endl; */
							cost = GetSPCost(depot, sequence_[i][j].h_) + sequence_[i][j].rcost_ + GetSPCost(sequence_[i][j].t_, depot);
							demand = GetSPDemand(depot, sequence_[i][j].h_) + sequence_[i][j].rdemand_ + GetSPDemand(sequence_[i][j].t_, depot);
							if(demand <= capacity_ and cost < tours_[i][j][k].cost_) {
								tours_[i][j][k].cost_ = cost; tours_[i][j][k].demand_ = demand;
								tours_[i][j][k].l_ = count;
							}
							/* std::cout << "Tours: " << i << " " << j << " " << k << " " << tours_[i][j][k].cost_ << " " << tours_[i][j][k].demand_ << " " << tours_[i][j][k].l_ << std::endl; */
						}
					}
				}
				/* std::cout << "T: " << m_ - 1 << " " << m_ - 1 << " " << tours_[m_ - 1][m_ - 1][0].cost_ << " " << tours_[m_ - 1][m_ - 1][0].demand_ << std::endl; */

				/* std::cout << "DP, init done\n"; */

				for(int i = (int)(m_ - 1); i >= 0; --i) {
					for(size_t j = i; j < m_; ++j) {
						/* std::cout << i << " " << j << std::endl; */
						for(size_t k = 1; k < K_; ++k) {
							if(tours_[i][j][k].cost_ < tours_[i][j][0].cost_ and tours_[i][j][k].demand_ <= capacity_) {
								tours_[i][j][0] = tours_[i][j][k];
								tours_[i][j][0].depot_ = depots_[k];
							}
						}
						/* std::cout << "T: " << i << " " << j << " " << tours_[i][j][0].cost_ << " " << tours_[i][j][0].demand_ << std::endl; */
						if(tours_[i][j][0].demand_ > capacity_){
							/* std::cout << "Split: " << i << " " << j << "\n"; */
							tours_[i][j][0].og_ = false;
							for(size_t r = i; r < j; ++r) {
								if(tours_[i][r][0].cost_ + tours_[r + 1][j][0].cost_ < tours_[i][j][0].cost_) {
									tours_[i][j][0].cost_ = tours_[i][r][0].cost_ + tours_[r + 1][j][0].cost_;
									tours_[i][j][0].demand_ = tours_[i][r][0].demand_ + tours_[r + 1][j][0].demand_;
									tours_[i][j][0].r_ = r;
								}
							}
							/* std::cout << "Tsplit: " << i << " " << j << " " << tours_[i][j][0].cost_ << " " << tours_[i][j][0].demand_ << std::endl; */
						}
					}
				}
				/* std::cout << "DP cost: " << tours_[0][m_ - 1][0].cost_ << std::endl; */
			}

			void GetTourEdges(size_t i, size_t j) {
				Tour t = tours_[i][j][0];
				if(t.og_ == true) {
					CreateRoute(i, j);
				} else {
					GetTourEdges(i, t.r_);
					GetTourEdges(t.r_ + 1, j);
				}
			}

			void CreateRoute(size_t i, size_t j) {
				std::cout << "Inside create route\n";
				Tour t = tours_[i][j][0];
				std::vector <Edge> edge_list;
				std::cout << i << " " << j << " " << t.depot_ << " " << t.l_ << " " << t.perm_<< std::endl;
				if(i == j) {
					if(t.l_ == 0) {
						GetSPPath(t.depot_, sequence_[i][i].t_, edge_list);
						edge_list.push_back(req_edges_[i]);
						GetSPPath(sequence_[i][i].h_, t.depot_, edge_list);
					} else {
						GetSPPath(t.depot_, sequence_[i][i].h_, edge_list);
						auto e = req_edges_[i]; e.Reverse();
						edge_list.push_back(e);
						GetSPPath(sequence_[i][i].t_, t.depot_, edge_list);
					}
				} else {
					if(t.l_ == 0) {
						auto u = t.depot_;
						for(size_t p = i; p <= j; ++p) {
							GetSPPath(u, req_edges_[p].GetTailVertexID(), edge_list);
							edge_list.push_back(req_edges_[p]);
							u = req_edges_[p].GetHeadVertexID();
						}
						GetSPPath(u, t.depot_, edge_list);
					}
					else if(t.l_ == (j - i + 1)) {
						auto u = t.depot_;
						for(int p = j; p >= (int)i; --p) {
							GetSPPath(u, req_edges_[p].GetHeadVertexID(), edge_list);
							auto new_edge = req_edges_[p];
							new_edge.Reverse();
							edge_list.push_back(new_edge);
							u = req_edges_[p].GetTailVertexID();
						}
						GetSPPath(u, t.depot_, edge_list);
					}
					else {
						AddOptimalPerm(t.depot_, i, j, i + t.l_ - 1, t.perm_, edge_list);
					}
				}
				std::cout << "Edge list size: " << edge_list.size() << std::endl;
				std::vector <Edge> final_edges;
				final_edges.push_back(edge_list[0]);
				auto curr_e = edge_list[0];
				for(size_t idx = 1; idx < edge_list.size(); ++idx) {
					auto e = edge_list[idx];
					if(curr_e.GetHeadVertexID() != e.GetTailVertexID()) {
						GetSPPath(curr_e.GetHeadVertexID(), e.GetTailVertexID(), final_edges);
					}
					final_edges.push_back(e);
					curr_e = e;
				}
				auto sol_digraph = std::make_shared <Graph> (vertex_list_, final_edges);
				sol_digraph->SetDepot(t.depot_);
				sol_digraph->PrintNM();
				sol_digraph_list_.push_back(sol_digraph);
				std::cout << "Route size: " << final_edges.size() << std::endl;
				std::cout << "End create route\n";
			}

			void AddOptimalPerm(const size_t v0, const size_t seqi, const size_t seqj, const size_t seql, const size_t perm, std::vector <Edge> &edge_list) {
				Sequence s1 = sequence_[seqi][seqj]; Sequence s2 = sequence_[seql + 1][seqj];
				size_t i, j; size_t l, m;
				i = s1.t_; j = s1.h_; l = s2.t_; m = s2.h_;
				size_t si = seqi, sj = seql, sl = seql + 1, sm = seqj;
				/* std::cout << "Inside AddOptimalPerm: " << i << " " << j << " " << l << " " << m << std::endl; */
				if(perm == 0) {
					GetSPPath(v0, i, edge_list);
					AddSeqEdges(si, sj, edge_list);
					GetSPPath(j, m, edge_list);
					AddSeqEdges(sm, sl, edge_list);
					GetSPPath(l, v0, edge_list);
				}
				if(perm == 1) {
					GetSPPath(v0, j, edge_list);
					AddSeqEdges(sj, si, edge_list);
					GetSPPath(i, l, edge_list);
					AddSeqEdges(sl, sm, edge_list);
					GetSPPath(m, v0, edge_list);
				}
				if(perm == 2) {
					GetSPPath(v0, j, edge_list);
					AddSeqEdges(sj, si, edge_list);
					GetSPPath(i, m, edge_list);
					AddSeqEdges(sm, sl, edge_list);
					GetSPPath(l, v0, edge_list);
				}
				if(perm == 3) {
					GetSPPath(v0, l, edge_list);
					AddSeqEdges(sl, sm, edge_list);
					GetSPPath(m, j, edge_list);
					AddSeqEdges(sj, si, edge_list);
					GetSPPath(i, v0, edge_list);
				}
				if(perm == 4) {
					GetSPPath(v0, m, edge_list);
					AddSeqEdges(sm, sl, edge_list);
					GetSPPath(l, i, edge_list);
					AddSeqEdges(si, sj, edge_list);
					GetSPPath(j, v0, edge_list);
				}
				if(perm == 5) {
					GetSPPath(v0, l, edge_list);
					AddSeqEdges(sl, sm, edge_list);
					GetSPPath(m, i, edge_list);
					AddSeqEdges(si, sj, edge_list);
					GetSPPath(j, v0, edge_list);
				}
			}

			bool AddSeqEdges(const size_t i, const size_t j, std::vector <Edge> &edge_list) {
				if(i <= j) {
					for(size_t p = i; p <= j; ++p) {
						edge_list.push_back(req_edges_[p]);
					}
				} else {
					for(int p = j; p >= (int)i; --p) {
						auto new_edge = req_edges_[p];
						new_edge.Reverse();
						edge_list.push_back(new_edge);
					}
				}
				return 0;
			}

			bool GetOptimalPerm(const size_t v0, const Sequence &s1, const Sequence &s2, double &cost, double &demand, size_t &optimal_perm) {
				size_t i, j; size_t l, m;
				i = s1.t_; j = s1.h_; l = s2.t_; m = s2.h_;
				double cost_list[6]; double demand_list[6];

				double ij = s1.cost_; double ji = s1.rcost_;
				double lm = s2.cost_; double ml = s2.rcost_;

				double d_ij = s1.demand_; double d_ji = s1.rdemand_;
				double d_lm = s2.demand_; double d_ml = s2.rdemand_;

				cost_list[0] = GetSPCost(v0, i) + ij + GetSPCost(j, m) + ml + GetSPCost(l, v0);
				cost_list[1] = GetSPCost(v0, j) + ji + GetSPCost(i, l) + lm + GetSPCost(m, v0);
				cost_list[2] = GetSPCost(v0, j) + ji + GetSPCost(i, m) + ml + GetSPCost(l, v0);

				cost_list[3] = GetSPCost(v0, l) + lm + GetSPCost(m, j) + ji + GetSPCost(i, v0);
				cost_list[4] = GetSPCost(v0, m) + ml + GetSPCost(l, i) + ij + GetSPCost(j, v0);
				cost_list[5] = GetSPCost(v0, l) + lm + GetSPCost(m, i) + ij + GetSPCost(j, v0);

				demand_list[0] = GetSPDemand(v0, i) + d_ij + GetSPDemand(j, m) + d_ml + GetSPDemand(l, v0);
				demand_list[1] = GetSPDemand(v0, j) + d_ji + GetSPDemand(i, l) + d_lm + GetSPDemand(m, v0);
				demand_list[2] = GetSPDemand(v0, j) + d_ji + GetSPDemand(i, m) + d_ml + GetSPDemand(l, v0);

				demand_list[3] = GetSPDemand(v0, l) + d_lm + GetSPDemand(m, j) + d_ji + GetSPDemand(i, v0);
				demand_list[4] = GetSPDemand(v0, m) + d_ml + GetSPDemand(l, i) + d_ij + GetSPDemand(j, v0);
				demand_list[5] = GetSPDemand(v0, l) + d_lm + GetSPDemand(m, i) + d_ij + GetSPDemand(j, v0);

				bool has_valid_merge = kFail;
				cost = kDoubleMax; demand = kDoubleMax;
				for(size_t perm = 0; perm < 6; ++perm) {
					if(demand_list[perm] <= capacity_) {
						if(has_valid_merge == kFail) {
							has_valid_merge = kSuccess;
							cost = cost_list[perm];
							demand = demand_list[perm];
							optimal_perm = perm;
						} else {
							if(cost_list[perm] < cost) {
								cost = cost_list[perm];
								demand = demand_list[perm];
								optimal_perm = perm;
							}
						}
					}
				}
				return has_valid_merge;
			}

			double GetSPCost(size_t tID, size_t hID) {
				size_t tindex, hindex;
				g_->GetVertexIndex(tID, tindex);
				g_->GetVertexIndex(hID, hindex);
				return apsp_->GetCost(tindex, hindex);
			}

			double GetSPDemand(size_t tID, size_t hID) {
				size_t tindex, hindex;
				g_->GetVertexIndex(tID, tindex);
				g_->GetVertexIndex(hID, hindex);
				return apsp_->GetDemand(tindex, hindex);
			}

			void GetSPPath(size_t tID, size_t hID, std::vector <Edge> &edge_list) {
				size_t tindex, hindex;
				g_->GetVertexIndex(tID, tindex);
				g_->GetVertexIndex(hID, hindex);
				apsp_->GetPath(edge_list, tindex, hindex);
			}
	};
}

#endif /* LCLIBRARY_MLC_TOUR_SPLITTING_MD_H_ */

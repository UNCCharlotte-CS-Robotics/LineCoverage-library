/**
 * This file is part of the LineCoverage-library.
 * The file contains beta=2 with atsp algorithm for SLC
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

#ifndef LCLIBRARY_SLC_SLC_BETA2_ATSP_H_
#define LCLIBRARY_SLC_SLC_BETA2_ATSP_H_

#include <lclibrary_config.h>
#include <chrono>
#include <memory>
#include <lclibrary/core/core.h>
#include <lclibrary/utils/utils.h>
#include <lclibrary/algorithms/algorithms.h>
#include <lclibrary/slc/slc_base.h>
#include <lclibrary/algorithms/atsp_base.h>
#include <lclibrary/algorithms/atsp_lkh.h>
#ifdef LCLIBRARY_USE_GUROBI
#include <lclibrary/slc/lp_gurobi.h>
#else
#include <lclibrary/slc/lp_glpk.h>
#endif /* LCLIBRARY_USE_GUROBI */

namespace lclibrary {

	class SLC_Beta2ATSP : public SLC_Base{

		std::shared_ptr <Graph> undirected_graph_;
		std::unique_ptr <SLC_LP> lp_;
		std::shared_ptr <APSP_FloydWarshall> apsp_;
		std::unique_ptr <ConnectedComponents> cc_;
		std::vector <size_t> atsp_tour_;
		double time_lp_, time_beta2_, time_atsp_, time_2opt_;
		double cost_beta2_atsp_, cost_;

		public:
		SLC_Beta2ATSP(const std::shared_ptr <const Graph> &g_in) : SLC_Base (g_in){
		}

		int Solve() {

			auto t_start_lp = std::chrono::high_resolution_clock::now();
#ifdef LCLIBRARY_USE_GUROBI
			std::cout << "Using Gurobi for LP\n";
			lp_ = std::make_unique <SLC_LP_Gurobi> (g_);
#else
			lp_ = std::make_unique <SLC_LP_glpk> (g_);
#endif
			apsp_ = std::make_shared <APSP_FloydWarshall>(g_);
			lp_->Solve();
			lp_->GenerateSolutionGraph(sol_digraph_, undirected_graph_);
			auto t_end_lp = std::chrono::high_resolution_clock::now();
			time_lp_ = std::chrono::duration<double, std::milli>(t_end_lp-t_start_lp).count();

			auto t_start_beta2 = std::chrono::high_resolution_clock::now();
			apsp_->APSP_Deadheading();
			SLC_Connected_TwoApprox();
			auto t_end_beta2 = std::chrono::high_resolution_clock::now();
			time_beta2_ = std::chrono::duration<double, std::milli>(t_end_beta2-t_start_beta2).count();

			auto t_start_atsp = std::chrono::high_resolution_clock::now();
			cc_ = std::make_unique <ConnectedComponents> (sol_digraph_);
			cc_->StronglyCCBalanced();
			std::cout << "Number of connected components: " << cc_->GetNumCC() << std::endl;
			if(cc_->GetNumCC() > 1) {
				SolveATSP();
			}

			route_ = EulerTourGeneration(sol_digraph_);
			std::cout << "Initial tour cost: " << route_.GetCost() << std::endl;
			route_.SetGraphAPSP(g_, apsp_);
			route_.RouteImprovement();
			std::cout << "Tour cost after route improvement: " << route_.GetCost() << std::endl;
			auto t_end_atsp = std::chrono::high_resolution_clock::now();
			time_atsp_ = std::chrono::duration<double, std::milli>(t_end_atsp-t_start_atsp).count();
			cost_beta2_atsp_ = route_.GetCost();

			auto t_start_2opt = std::chrono::high_resolution_clock::now();
			if(use_2opt_ == true) {
				route_.TwoOpt();
			}
			auto t_end_2opt = std::chrono::high_resolution_clock::now();
			time_2opt_ = std::chrono::duration<double, std::milli>(t_end_2opt-t_start_2opt).count();

			if(g_->IsDepotSet()) {
				route_.RotateToDepot(g_->GetVertexID(g_->GetDepot()));
			}

			route_.CheckRoute();
			sol_digraph_->ClearAllEdges();
			std::vector <Edge> edge_list;
			route_.GenerateEdgeList(edge_list);
			std::cout << "Route size: " << edge_list.size() << std::endl;
			sol_digraph_->AddEdge(edge_list);
			if(g_->IsDepotSet()) {
				sol_digraph_->SetDepot(g_->GetDepotID());
			}
			sol_digraph_->PrintNM();
			cost_ = route_.GetCost();
			std::cout << "Route cost: " << cost_ << std::endl;

			return kSuccess;
		}

		void SolveATSP() {
			auto num_cc = cc_->GetNumCC();
			std::vector <Edge> edge_list;

			if(num_cc == 2) {
				auto i = cc_->GetRepVertex(0);
				auto j = cc_->GetRepVertex(1);
				apsp_->GetPath(edge_list, i, j);
				apsp_->GetPath(edge_list, j, i);
			}
			else {
				std::vector < std::vector <double> > d;
				d.resize(num_cc, std::vector <double>(num_cc));
				for(size_t i = 0; i < num_cc; ++i) {
					for(size_t j = 0; j < num_cc; ++j){
						d[i][j] = apsp_->GetCost(cc_->GetRepVertex(i), cc_->GetRepVertex(j));
					}
				}

				std::unique_ptr <ATSP> atsp_solver;
#ifdef LCLIBRARY_USE_LKH
				if(num_cc <= 20) {
					atsp_solver = std::make_unique <ATSP_DP_BHK>(num_cc, d);
				} else {
					atsp_solver = std::make_unique <ATSP_LKH>(num_cc, d);
				}
#else
				atsp_solver = std::make_unique <ATSP_DP_BHK>(num_cc, d);
				if(num_cc > 20) {
					std::cerr << "ATSP size too large. Please use LKH\n";
					return;
				}
#endif /* LCLIBRARY_USE_LKH */
				atsp_solver->GetPath(atsp_tour_);
				for(size_t i = 0; i < atsp_tour_.size() - 1; ++i) {
					apsp_->GetPath(edge_list, cc_->GetRepVertex(atsp_tour_[i]), cc_->GetRepVertex(atsp_tour_[i + 1]));
				}
			}
			sol_digraph_->AddEdge(edge_list);
		}

		void GetComputationTimes(std::vector <double> &comp_t) {
			comp_t.clear();
			comp_t.push_back(time_lp_);
			comp_t.push_back(time_beta2_);
			comp_t.push_back(time_atsp_);
			comp_t.push_back(time_2opt_);
		}

		void GetComputationTimes(double &time_lp, double &time_beta2, double &time_atsp, double &time_2opt) {
			time_lp = time_lp_;
			time_beta2 = time_beta2_;
			time_atsp = time_atsp_;
			time_2opt = time_2opt_;
		}

		void GetCosts(std::vector <double> &costs) {
			costs.push_back(cost_beta2_atsp_);
			costs.push_back(cost_);
		}

		void GetCosts(double &cost_beta2_atsp, double &cost) {
			cost_beta2_atsp = cost_beta2_atsp_;
			cost = cost_;
		}

		private:
		void SLC_Connected_TwoApprox () {
			auto m_u = undirected_graph_->GetM();
			std::vector <Edge> edge_list;
			for (size_t i = 0; i < m_u; ++i) {
				size_t t, h;
				undirected_graph_->GetVerticesIndexOfEdge(i, t, h, kIsRequired);
				if(undirected_graph_->GetServiceCost(i) + apsp_->GetCost(h, t) <= undirected_graph_->GetReverseServiceCost(i) + apsp_->GetCost(t, h) ) {
					auto e = undirected_graph_->GetEdge(i, kIsRequired);
					Edge new_edge = *e;
					new_edge.SetReq(kIsRequired);
					/* Edge new_edge(e->GetTailVertexID(), e->GetHeadVertexID(), kIsRequired); */
					new_edge.SetCost(e->GetServiceCost());
					edge_list.push_back(new_edge);
					apsp_->GetPath(edge_list, h, t);
				}
				else {
					auto e = undirected_graph_->GetEdge(i, kIsRequired);
					Edge new_edge = *e;
					new_edge.SetReq(kIsRequired);
					/* Edge new_edge(e->GetHeadVertexID(), e->GetTailVertexID(), kIsRequired); */
					new_edge.SetCost(e->GetReverseServiceCost());
					new_edge.Reverse();
					edge_list.push_back(new_edge);
					apsp_->GetPath(edge_list, t, h);
				}
			}
			sol_digraph_->AddEdge(edge_list);
		}
	};

}
#endif /* LCLIBRARY_SLC_SLC_BETA2_ATSP_H_ */

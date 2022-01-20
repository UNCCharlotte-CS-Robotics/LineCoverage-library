/**
 * This file is part of the LineCoverage-library.
 * The file contains SLC beta3 heuristic solver using GLPK
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

#ifndef LCLIBRARY_SLC_LP_BETA3_GUROBI_H_
#define LCLIBRARY_SLC_LP_BETA3_GUROBI_H_

#include <gurobi_c++.h>
#include <cmath>
#include <string>
#include <memory>
#include <lclibrary/core/constants.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/slc/slc_lp.h>

namespace lclibrary {

	class SLC_LP_beta3_Gurobi : public SLC_LP {
		size_t n_, m_, m_nr_;
		size_t depot_index_;
		double obj_val_;
		std::shared_ptr <const Graph> g_;

		std::vector <GRBVar> req_sa_;
		std::vector <GRBVar> req_sa_rev_;
		std::vector <GRBVar> req_da_;
		std::vector <GRBVar> req_da_rev_;
		std::vector <GRBVar> nonreq_da_;
		std::vector <GRBVar> nonreq_da_rev_;

		GRBEnv env;
		std::unique_ptr <GRBModel> model;

		public:
		SLC_LP_beta3_Gurobi(const std::shared_ptr <const Graph> &g_in) : g_{g_in} {
			model = std::make_unique<GRBModel>(env);
		}

		~SLC_LP_beta3_Gurobi() { }

		int Solve() {
			Initialize();
			AddVars();
			SymmetryConstraints();
			TraversingConstraints();
			model->optimize();
			obj_val_ = model->get(GRB_DoubleAttr_ObjVal);
			return 0;
		}

		private:

		void Initialize() {
			n_ = g_->GetN();
			m_ = g_->GetM();
			m_nr_ = g_->GetMnr();
			req_sa_.resize(m_);
			req_sa_rev_.resize(m_);
			req_da_.resize(m_);
			req_da_rev_.resize(m_);
			nonreq_da_.resize(m_nr_);
			nonreq_da_rev_.resize(m_nr_);

			size_t tail_index, head_index;
			g_->GetVerticesIndexOfEdge(0, tail_index, head_index, kIsRequired);
			depot_index_ = tail_index;
		}

		void AddVars() {

			for(size_t i = 0; i < m_; ++i) {
				if(g_->GetServiceCost(i) < g_->GetReverseServiceCost(i)) {
					req_sa_[i] = model->addVar(1.0, 1.0, g_->GetServiceCost(i), GRB_CONTINUOUS, "s_" + std::to_string(i));
					req_sa_rev_[i] = model->addVar(0.0, 0.0, g_->GetReverseServiceCost(i), GRB_CONTINUOUS, "s_" + std::to_string(i) + "_rev");
				} else {
					req_sa_[i] = model->addVar(0.0, 0.0, g_->GetServiceCost(i), GRB_CONTINUOUS, "s_" + std::to_string(i));
					req_sa_rev_[i] = model->addVar(1.0, 1.0, g_->GetReverseServiceCost(i), GRB_CONTINUOUS, "s_" + std::to_string(i) + "_rev");
				}

				req_da_[i] = model->addVar(0.0, GRB_INFINITY, g_->GetDeadheadCost(i, kIsRequired), GRB_CONTINUOUS, "d_req_" + std::to_string(i));
				req_da_rev_[i] = model->addVar(0.0, GRB_INFINITY, g_->GetReverseDeadheadCost(i, kIsRequired), GRB_CONTINUOUS, "d_req_" + std::to_string(i) + "_rev");
			}

			for(size_t i = 0; i < m_nr_; ++i) {
				nonreq_da_[i] = model->addVar(0.0, GRB_INFINITY, g_->GetDeadheadCost(i, kIsNotRequired), GRB_CONTINUOUS, "d_nreq_" + std::to_string(i));
				nonreq_da_rev_[i] = model->addVar(0.0, GRB_INFINITY, g_->GetReverseDeadheadCost(i, kIsNotRequired), GRB_CONTINUOUS, "d_nreq_" + std::to_string(i) + "_rev");
			}
		}

		void SymmetryConstraints() {
			std::vector <GRBLinExpr> symmetry_constraints(n_);
			for(size_t i = 0; i < n_; ++i) {
				symmetry_constraints[i] = 0.0;
			}
			size_t tail_index, head_index;
			for(size_t i = 0; i < m_; ++i) {
				g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsRequired);
				symmetry_constraints[head_index] += (req_sa_[i] + req_da_[i]);
				symmetry_constraints[tail_index] -= (req_sa_[i] + req_da_[i]);
				symmetry_constraints[tail_index] += (req_sa_rev_[i] + req_da_rev_[i]);
				symmetry_constraints[head_index] -= (req_sa_rev_[i] + req_da_rev_[i]);
			}
			for(size_t i = 0; i < m_nr_; ++i) {
				g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsNotRequired);
				symmetry_constraints[head_index] += nonreq_da_[i];
				symmetry_constraints[tail_index] -= nonreq_da_[i];
				symmetry_constraints[tail_index] += nonreq_da_rev_[i];
				symmetry_constraints[head_index] -= nonreq_da_rev_[i];
			}
			for(size_t i = 0; i < n_; ++i) {
				model->addConstr(symmetry_constraints[i] == 0, "symmetry_constraints_" + std::to_string(i));
			}
		}

		void TraversingConstraints() {
			for(size_t i = 0; i < m_; ++i) {
				model->addConstr(req_sa_[i] + req_sa_rev_[i] == 1, "traversing_constraints_" + std::to_string(i));
			}
		}

		int GenerateSolutionGraph (std::shared_ptr <Graph> &digraph, std::shared_ptr <Graph> &undirected_graph) {
			std::vector <Vertex> vertex_list;
			std::vector <Edge> edge_list;
			std::vector <Edge> undirected_edge_list;
			vertex_list.reserve(n_);
			edge_list.reserve(m_);
			size_t m, m_nr, m_u;
			m = 0; m_nr = 0; m_u = 0;
			for(size_t i = 0; i < n_; ++i) {
				Vertex v;
				g_->GetVertexData(i, v);
				vertex_list.push_back(v);
			}
			for(size_t i = 0; i < m_; ++i) {
				double s_a = req_sa_[i].get(GRB_DoubleAttr_X);
				double s_a_rev = req_sa_rev_[i].get(GRB_DoubleAttr_X);
				Edge edge;
				g_->GetEdgeData(i, edge, kIsRequired);
				if(IsNearZero(s_a - 1.0, 1e-5)) {
					Edge new_edge(edge);
					new_edge.SetCost(edge.GetServiceCost());
					new_edge.SetReq(kIsRequired);
					edge_list.push_back(new_edge);
					++m;
				}
				if(IsNearZero(s_a_rev - 1.0, 1e-5)) {
					Edge new_edge(edge);
					new_edge.SetCost(edge.GetReverseServiceCost());
					new_edge.Reverse();
					new_edge.SetReq(kIsRequired);
					edge_list.push_back(new_edge);
					++m;
				}

				int d_a = std::round(req_da_[i].get(GRB_DoubleAttr_X));
				int d_a_rev = std::round(req_da_rev_[i].get(GRB_DoubleAttr_X));
				if(d_a > 0) {
					Edge new_edge(edge);
					new_edge.SetCost(edge.GetDeadheadCost());
					new_edge.SetReq(kIsNotRequired);
					for(int j = 0; j < d_a; ++j) {
						edge_list.push_back(new_edge);
						++m_nr;
					}
				}
				if(d_a_rev > 0) {
					Edge new_edge(edge);
					new_edge.SetCost(edge.GetReverseDeadheadCost());
					new_edge.SetReq(kIsNotRequired);
					new_edge.Reverse();
					for(int j = 0; j < d_a_rev; ++j) {
						edge_list.push_back(new_edge);
						++m_nr;
					}
				}

				if(IsNearZero(s_a - 0.5, 1e-5)) {
					Edge new_edge = edge;
					new_edge.SetReq(kIsRequired);
					undirected_edge_list.push_back(new_edge);
					++m_u;
				}
			}

			for(size_t i = 0; i < m_nr_; ++i) {
				int d_a = std::round(nonreq_da_[i].get(GRB_DoubleAttr_X));
				int d_a_rev = std::round(nonreq_da_rev_[i].get(GRB_DoubleAttr_X));
				Edge edge;
				g_->GetEdgeData(i, edge, kIsNotRequired);
				if(d_a > 0) {
					Edge new_edge(edge);
					new_edge.SetCost(edge.GetDeadheadCost());
					new_edge.SetReq(kIsNotRequired);
					for(int j = 0; j < d_a; ++j) {
						edge_list.push_back(new_edge);
						++m_nr;
					}
				}
				if(d_a_rev > 0) {
					Edge new_edge(edge);
					new_edge.SetCost(edge.GetReverseDeadheadCost());
					new_edge.SetReq(kIsNotRequired);
					new_edge.Reverse();
					for(int j = 0; j < d_a_rev; ++j) {
						edge_list.push_back(new_edge);
						++m_nr;
					}
				}
			}

			digraph = std::make_shared <Graph>(vertex_list, edge_list, m, m_nr);
			undirected_graph = std::make_shared <Graph>(vertex_list, undirected_edge_list, m_u, 0);
			return 0;
		}

	};

}
#endif /* LCLIBRARY_SLC_LP_BETA3_GUROBI_H_ */

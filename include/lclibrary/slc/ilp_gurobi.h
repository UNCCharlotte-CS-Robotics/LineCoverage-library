/**
 * This file is part of the LineCoverage-library.
 * The file contains SLC ILP formulation solved using Gurobi
 * Gurobi should be configured to use this class
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

#ifndef LCLIBRARY_SLC_ILP_GUROBI_H_
#define LCLIBRARY_SLC_ILP_GUROBI_H_

#include <lclibrary/core/graph.h>
#include <lclibrary/slc/ilp_base.h>
#include <gurobi_c++.h>
#include <cmath>
#include <string>

namespace lclibrary {

	class SLC_ILP_Gurobi:public SLC_ILP {
		size_t n_, m_, m_nr_;
		size_t depot_index_;
		double obj_val_, obj_bound_ = 0;
		int exit_status_ = 0;
		std::string warm_start_file_;
		double time_limit_ = 3600;

		std::vector <GRBVar> req_sa_;
		std::vector <GRBVar> req_sa_rev_;
		std::vector <GRBVar> req_da_;
		std::vector <GRBVar> req_da_rev_;
		std::vector <GRBVar> nonreq_da_;
		std::vector <GRBVar> nonreq_da_rev_;
		std::vector <GRBVar> req_za_;
		std::vector <GRBVar> req_za_rev_;
		std::vector <GRBVar> nonreq_za_;
		std::vector <GRBVar> nonreq_za_rev_;

		GRBEnv env;
		std::unique_ptr <GRBModel> model;

		public:
		SLC_ILP_Gurobi(const std::shared_ptr <const Graph> g_in, std::string filename = "", double time_limit = 3600):SLC_ILP(g_in), warm_start_file_{filename}, time_limit_{time_limit}{
			model = std::make_unique<GRBModel>(env);
		}

		int SolveILP() {
			Initialize();
			AddVars();
			if(warm_start_file_ != "") {
				std::cout << "Using warm start" << std::endl;
				WarmStart();
			}
			SymmetryConstraints();
			TraversingConstraints();
			ConnectivityConstraints();
			DepotFlowConstraint();
			FlowLimit();
			model->set(GRB_DoubleParam_TimeLimit, time_limit_);
			model->optimize();
			int optimstatus = model->get(GRB_IntAttr_Status);
			if(optimstatus == GRB_OPTIMAL) {
				exit_status_ = 0;
				obj_val_ = model->get(GRB_DoubleAttr_ObjVal);
			}
			if(optimstatus == GRB_TIME_LIMIT) {
				if(model->get(GRB_IntAttr_SolCount) > 0) {
					exit_status_ = 1;
					obj_val_ = model->get(GRB_DoubleAttr_ObjVal);
				} else {
					exit_status_ = -1;
				}
			}
			obj_bound_ = model->get(GRB_DoubleAttr_ObjBound);
			return 0;
		}

		double GetObjBound() {
			return obj_bound_;
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
			req_za_.resize(m_);
			req_za_rev_.resize(m_);
			nonreq_za_.resize(m_nr_);
			nonreq_za_rev_.resize(m_nr_);

			size_t tail_index, head_index;
			g_->GetVerticesIndexOfEdge(0, tail_index, head_index, true);
			depot_index_ = tail_index;
		}

		void AddVars() {

			for(size_t i = 0; i < m_; ++i) {
				req_sa_[i] = model->addVar(0.0, 1.0, g_->GetServiceCost(i), GRB_BINARY, "s_" + std::to_string(i));
				req_sa_rev_[i] = model->addVar(0.0, 1.0, g_->GetReverseServiceCost(i), GRB_BINARY, "s_" + std::to_string(i) + "_rev");

				req_da_[i] = model->addVar(0.0, GRB_INFINITY, g_->GetDeadheadCost(i, kIsRequired), GRB_INTEGER, "d_req_" + std::to_string(i));
				req_da_rev_[i] = model->addVar(0.0, GRB_INFINITY, g_->GetReverseDeadheadCost(i, kIsRequired), GRB_INTEGER, "d_req_" + std::to_string(i) + "_rev");

				req_za_[i] = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "z_req_" + std::to_string(i));
				req_za_rev_[i] = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "z_req_" + std::to_string(i) + "_rev");
			}

			for(size_t i = 0; i < m_nr_; ++i) {
				nonreq_da_[i] = model->addVar(0.0, GRB_INFINITY, g_->GetDeadheadCost(i, kIsNotRequired), GRB_INTEGER, "d_nreq_" + std::to_string(i));
				nonreq_da_rev_[i] = model->addVar(0.0, GRB_INFINITY, g_->GetReverseDeadheadCost(i, kIsNotRequired), GRB_INTEGER, "d_nreq_" + std::to_string(i) + "_rev");

				nonreq_za_[i] = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "z_nreq_" + std::to_string(i));
				nonreq_za_rev_[i] = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "z_nreq_" + std::to_string(i) + "_rev");

			}
		}

		void WarmStart() {
			std::ifstream warm_start_file(warm_start_file_);
			size_t u, v;
			bool req;
			double cost;
			std::vector <double> req_da_ws(m_, 0);
			std::vector <double> req_da_rev_ws(m_, 0);
			std::vector <double> nonreq_da_ws(m_nr_, 0);
			std::vector <double> nonreq_da_rev_ws(m_nr_, 0);
			while(warm_start_file >> u >> v >> req >> cost) {
				if(req) {
					for(size_t i = 0; i < m_; ++i) {
						size_t edge_u, edge_v;
						g_->GetVerticesIDOfEdge(i, edge_u, edge_v, kIsRequired);
						if(u == edge_u and v == edge_v) {
							req_sa_[i].set(GRB_DoubleAttr_Start, 1.0);
							req_sa_rev_[i].set(GRB_DoubleAttr_Start, 0.0);
							break;
						}
						if(v == edge_u and u == edge_v) {
							req_sa_rev_[i].set(GRB_DoubleAttr_Start, 1.0);
							req_sa_[i].set(GRB_DoubleAttr_Start, 0.0);
							break;
						}
					}
				} else {
					bool found = false;
					for(size_t i = 0; i < m_; ++i) {
						size_t edge_u, edge_v;
						g_->GetVerticesIDOfEdge(i, edge_u, edge_v, kIsRequired);
						if(u == edge_u and v == edge_v) {
							req_da_ws[i] += 1.0;
							found = true;
							break;
						}
						if(v == edge_u and u == edge_v) {
							req_da_rev_ws[i] += 1.0;
							found = true;
							break;
						}
					}
					if(found == true) {
						continue;
					}
					for(size_t i = 0; i < m_nr_; ++i) {
						size_t edge_u, edge_v;
						g_->GetVerticesIDOfEdge(i, edge_u, edge_v, not kIsRequired);
						if(u == edge_u and v == edge_v) {
							nonreq_da_ws[i] += 1;
							break;
						}
						if(v == edge_u and u == edge_v) {
							nonreq_da_rev_ws[i] += 1;
							break;
						}
					}
				}
			}
			for(size_t i = 0; i < m_; ++i) {
				req_da_[i].set(GRB_DoubleAttr_Start, req_da_ws[i]);
				req_da_rev_[i].set(GRB_DoubleAttr_Start, req_da_rev_ws[i]);
			}
			for(size_t i = 0; i < m_nr_; ++i) {
				nonreq_da_[i].set(GRB_DoubleAttr_Start, nonreq_da_ws[i]);
				nonreq_da_rev_[i].set(GRB_DoubleAttr_Start, nonreq_da_rev_ws[i]);
			}
			warm_start_file.close();
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

		void ConnectivityConstraints() {
			std::vector <GRBLinExpr> connectivity_flow(n_);
			size_t tail_index, head_index;
			for(size_t i = 0; i < m_; ++i) {
				g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsRequired);
				connectivity_flow[head_index] += req_za_[i];
				connectivity_flow[tail_index] -= req_za_[i];
				connectivity_flow[tail_index] += req_za_rev_[i];
				connectivity_flow[head_index] -= req_za_rev_[i];

				connectivity_flow[head_index] -= req_sa_[i];
				connectivity_flow[tail_index] -= req_sa_rev_[i];
			}
			for(size_t i = 0; i < m_nr_; ++i) {
				g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsNotRequired);
				connectivity_flow[head_index] += nonreq_za_[i];
				connectivity_flow[tail_index] -= nonreq_za_[i];
				connectivity_flow[tail_index] += nonreq_za_rev_[i];
				connectivity_flow[head_index] -= nonreq_za_rev_[i];
			}
			for(size_t i = 0; i < n_; ++i) {
				if(i == depot_index_) {
					continue;
				}
				model->addConstr(connectivity_flow[i] == 0, "connectivity_constraints_" + std::to_string(i));
			}
		}

		void DepotFlowConstraint() {
			GRBLinExpr depot_flow_constraint;
			size_t tail_index, head_index;
			for(size_t i = 0; i < m_; ++i) {
				g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsRequired);
				if(tail_index == depot_index_) {
					depot_flow_constraint += req_za_[i];
				}
				if(head_index == depot_index_) {
					depot_flow_constraint += req_za_rev_[i];
				}
			}
			for(size_t i = 0; i < m_nr_; ++i) {
				g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsNotRequired);
				if(tail_index == depot_index_) {
					depot_flow_constraint += nonreq_za_[i];
				}
				if(head_index == depot_index_) {
					depot_flow_constraint += nonreq_za_rev_[i];
				}
			}
			model->addConstr(depot_flow_constraint == m_, "depot_flow_constraint");
		}

		void FlowLimit() {
			for(size_t i = 0; i < m_; ++i) {
				model->addConstr(req_za_[i] - m_ * (req_da_[i] + req_sa_[i]) <= 0, "flow_limit_req_" + std::to_string(i));
				model->addConstr(req_za_rev_[i] - m_ * (req_da_rev_[i] + req_sa_rev_[i]) <= 0, "flow_limit_req_rev_" + std::to_string(i));
			}
			for(size_t i = 0; i < m_nr_; ++i) {
				model->addConstr(nonreq_za_[i] - m_ * nonreq_da_[i] <= 0, "flow_limit_nonreq_" + std::to_string(i));
				model->addConstr(nonreq_za_rev_[i] - m_ * nonreq_da_rev_[i] <= 0, "flow_limit_nonreq_rev_" + std::to_string(i));
			}
		}

		int GenerateSolutionGraph() {
			if(exit_status_ == -1)
				return exit_status_;
			std::vector <Vertex> vertex_list;
			std::vector <Edge> edge_list;
			vertex_list.reserve(n_);
			edge_list.reserve(m_);
			size_t m, m_nr;
			m = 0; m_nr = 0;
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
				if(s_a > 0.5) {
					Edge new_edge(edge);
					new_edge.SetCost(edge.GetServiceCost());
					new_edge.SetReq(kIsRequired);
					edge_list.push_back(new_edge);
					++m;
				}
				if(s_a_rev > 0.5) {
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

			double cost = 0;
			size_t req_count = 0;
			for(auto &e:edge_list) {
				cost += e.GetCost();
				if(e.GetReq())
					++req_count;
			}
			/* if (std::fabs(cost - obj_val_) > 1e-10) { */
			/* 	std::cerr << "Mismatch in cost: " << cost << " " << obj_val_ << std::endl; */
			/* } */
			/* std::cout << edge_list.size() << " " << m << " " << m_nr << " " << req_count<<std::endl; */
			sol_digraph_ = std::make_shared <Graph>(vertex_list, edge_list, m, m_nr);
			return exit_status_;
		}
	};

}
#endif /* LCLIBRARY_SLC_ILP_GUROBI_H_ */

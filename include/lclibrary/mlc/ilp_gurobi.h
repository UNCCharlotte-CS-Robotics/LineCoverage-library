/**
 * This file is part of the LineCoverage-library.
 * The file contains MLC ILP formulation solved using Gurobi
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

#ifndef LCLIBRARY_MLC_ILP_GUROBI_H_
#define LCLIBRARY_MLC_ILP_GUROBI_H_

#include <gurobi_c++.h>
#include <cmath>
#include <string>
#include <lclibrary/core/graph.h>
#include <lclibrary/mlc/ilp_base.h>

namespace lclibrary {

	typedef std::vector <GRBVar> VarVec;
	typedef std::vector <VarVec> Var2Vec;

	class MLC_ILP_Gurobi:public MLC_ILP {
		size_t K_; // number of routes
		double Q_; // capacity
		size_t n_, m_, m_nr_;
		size_t depot_index_;
		double obj_val_, obj_bound_ = 0;
		int exit_status_ = 0;
		std::vector <Route> warmstart_route_list_;
		double time_limit_ = 3600;
		bool use_lower_bound_ = false;
		double lower_bound_ = 0;

		Var2Vec req_sa_;
		Var2Vec req_sa_rev_;
		Var2Vec req_da_;
		Var2Vec req_da_rev_;
		Var2Vec nonreq_da_;
		Var2Vec nonreq_da_rev_;
		Var2Vec req_za_;
		Var2Vec req_za_rev_;
		Var2Vec nonreq_za_;
		Var2Vec nonreq_za_rev_;

		GRBEnv env;
		std::unique_ptr <GRBModel> model;

		public:
		MLC_ILP_Gurobi(
				const std::shared_ptr <const Graph> g_in,
				const std::vector <Route> &warmstart_route_list, double time_limit = 3600, double lower_bound = 0) :
			MLC_ILP(g_in),
			warmstart_route_list_{warmstart_route_list}, time_limit_{time_limit}{
			K_ = warmstart_route_list_.size();
			model = std::make_unique<GRBModel>(env);
			if(lower_bound != 0) {
				lower_bound_ = lower_bound;
				use_lower_bound_ = true;
			}
		}

		~MLC_ILP_Gurobi() { }

		int SolveILP() {

			Initialize();

			AddVars();
			WarmStart();

			if(use_lower_bound_) {
				LowerBound();
			}
			TraversingConstraints();
			CapacityConstraints();
			SymmetryConstraints();
			FlowBalance();
			DepotFlowConstraint();
			FlowLimit();
			BreakSymmetry();
			model->update();
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
			Q_ = g_->GetCapacity();
			n_ = g_->GetN();
			m_ = g_->GetM();
			m_nr_ = g_->GetMnr();
			req_sa_.resize(K_, VarVec(m_));
			req_sa_rev_.resize(K_, VarVec(m_));
			req_da_.resize(K_, VarVec(m_));
			req_da_rev_.resize(K_, VarVec(m_));

			nonreq_da_.resize(K_, VarVec(m_nr_));
			nonreq_da_rev_.resize(K_, VarVec(m_nr_));

			req_za_.resize(K_, VarVec(m_));
			req_za_rev_.resize(K_, VarVec(m_));
			nonreq_za_.resize(K_, VarVec(m_nr_));
			nonreq_za_rev_.resize(K_, VarVec(m_nr_));

			if(g_->IsDepotSet()) {
				depot_index_ = g_->GetDepot();
			} else {
				size_t tail_index, head_index;
				g_->GetVerticesIndexOfEdge(0, tail_index, head_index, true);
				depot_index_ = tail_index;
			}
		}

		void AddVars() {
			for(size_t k = 0; k < K_; ++k) {
				for(size_t i = 0; i < m_; ++i) {
					req_sa_[k][i] = model->addVar(0.0, 1.0, g_->GetServiceCost(i), GRB_BINARY, "s_" + std::to_string(k) + "_" + std::to_string(i));
					req_sa_rev_[k][i] = model->addVar(0.0, 1.0, g_->GetReverseServiceCost(i), GRB_BINARY, "s_" + std::to_string(k) + "_" + std::to_string(i) + "_rev");

					req_da_[k][i] = model->addVar(0.0, GRB_INFINITY, g_->GetDeadheadCost(i, kIsRequired), GRB_INTEGER, "d_req_" + std::to_string(k) + "_" + std::to_string(i));
					req_da_rev_[k][i] = model->addVar(0.0, GRB_INFINITY, g_->GetReverseDeadheadCost(i, kIsRequired), GRB_INTEGER, "d_req_" + std::to_string(k) + "_" + std::to_string(i) + "_rev");

					req_za_[k][i] = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "z_req_" + std::to_string(k) + "_" + std::to_string(i));
					req_za_rev_[k][i] = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "z_req_" + std::to_string(k) + "_" + std::to_string(i) + "_rev");
				}

				for(size_t i = 0; i < m_nr_; ++i) {
					nonreq_da_[k][i] = model->addVar(0.0, GRB_INFINITY, g_->GetDeadheadCost(i, kIsNotRequired), GRB_INTEGER, "d_nreq_" + std::to_string(k) + "_" + std::to_string(i));
					nonreq_da_rev_[k][i] = model->addVar(0.0, GRB_INFINITY, g_->GetReverseDeadheadCost(i, kIsNotRequired), GRB_INTEGER, "d_nreq_" + std::to_string(k) + "_" + std::to_string(i) + "_rev");

					nonreq_za_[k][i] = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "z_nreq_" + std::to_string(k) + "_" + std::to_string(i));
					nonreq_za_rev_[k][i] = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "z_nreq_" + std::to_string(k) + "_" + std::to_string(i) + "_rev");

				}
			}
		}

		void WarmStart() {
			std::sort(warmstart_route_list_.begin(), warmstart_route_list_.end(), [](const Route &a, const Route &b) { return a.GetRouteLength() > b.GetRouteLength(); });
			for(size_t k = 0; k < K_; ++k) {
				std::vector <double> req_sa_ws(m_, 0);
				std::vector <double> req_sa_rev_ws(m_, 0);
				std::vector <double> req_da_ws(m_, 0);
				std::vector <double> req_da_rev_ws(m_, 0);
				std::vector <double> nonreq_da_ws(m_nr_, 0);
				std::vector <double> nonreq_da_rev_ws(m_nr_, 0);
				auto route_it = warmstart_route_list_[k].GetRouteStart();
				auto route_it_end = warmstart_route_list_[k].GetRouteEnd();
				for(; route_it != route_it_end; std::advance(route_it,1)) {
					size_t u, v;
					bool req;
					warmstart_route_list_[k].GetTailHeadReq(route_it, u, v, req);
					if(u == v) {
						continue;
					}
					bool found = false;
					if(req == kIsRequired) {
						for(size_t i = 0; i < m_; ++i) {
							size_t edge_u, edge_v;
							g_->GetVerticesIDOfEdge(i, edge_u, edge_v, kIsRequired);
							if(u == edge_u and v == edge_v) {
								req_sa_ws[i] = 1.0;
								found = true;
								break;
							}
							if(v == edge_u and u == edge_v) {
								req_sa_rev_ws[i] = 1.0;
								found = true;
								break;
							}
						}
					} else {
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
							g_->GetVerticesIDOfEdge(i, edge_u, edge_v, kIsNotRequired);
							if(u == edge_u and v == edge_v) {
								nonreq_da_ws[i] += 1.0;
								found = true;
								break;
							}
							if(v == edge_u and u == edge_v) {
								nonreq_da_rev_ws[i] += 1.0;
								found = true;
								break;
							}
						}
					}
				}
				for(size_t i = 0; i < m_; ++i) {
					req_sa_[k][i].set(GRB_DoubleAttr_Start, req_sa_ws[i]);
					req_sa_rev_[k][i].set(GRB_DoubleAttr_Start, req_sa_rev_ws[i]);
					req_da_[k][i].set(GRB_DoubleAttr_Start, req_da_ws[i]);
					req_da_rev_[k][i].set(GRB_DoubleAttr_Start, req_da_rev_ws[i]);
				}
				for(size_t i = 0; i < m_nr_; ++i) {
					nonreq_da_[k][i].set(GRB_DoubleAttr_Start, nonreq_da_ws[i]);
					nonreq_da_rev_[k][i].set(GRB_DoubleAttr_Start, nonreq_da_rev_ws[i]);
				}
			}
		}

		void TraversingConstraints() {
			for(size_t i = 0; i < m_; ++i) {
				GRBLinExpr traverse_expr = 0.0;
				for(size_t k = 0; k < K_; ++k) {
					traverse_expr += req_sa_[k][i] + req_sa_rev_[k][i];
				}
				model->addConstr(traverse_expr == 1, "traversing_constraints_" + std::to_string(i));
			}
		}

		void LowerBound() {
			GRBLinExpr total_cost;
			for(size_t k = 0; k < K_; ++k) {
				for(size_t i = 0; i < m_; ++i) {
					total_cost += g_->GetServiceCost(i) * req_sa_[k][i];
					total_cost += g_->GetReverseServiceCost(i) * req_sa_rev_[k][i];
					total_cost += g_->GetDeadheadCost(i, kIsRequired) * req_da_[k][i];
					total_cost += g_->GetReverseDeadheadCost(i, kIsRequired) * req_da_rev_[k][i];
				}
				for(size_t i = 0; i < m_nr_; ++i) {
					total_cost += g_->GetDeadheadCost(i, kIsNotRequired) * nonreq_da_[k][i];
					total_cost += g_->GetReverseDeadheadCost(i, kIsNotRequired) * nonreq_da_rev_[k][i];
				}
			}
			model->addConstr(total_cost >= lower_bound_, "lower_bound");
		}

		void CapacityConstraints() {
			for(size_t k = 0; k < K_; ++k) {
				GRBLinExpr demand_expr;
				for(size_t i = 0; i < m_; ++i) {
					demand_expr += g_->GetServiceDemand(i) * req_sa_[k][i];
					demand_expr += g_->GetReverseServiceDemand(i) * req_sa_rev_[k][i];
					demand_expr += g_->GetDeadheadDemand(i, kIsRequired) * req_da_[k][i];
					demand_expr += g_->GetReverseDeadheadDemand(i, kIsRequired) * req_da_rev_[k][i];
				}
				for(size_t i = 0; i < m_nr_; ++i) {
					demand_expr += g_->GetDeadheadDemand(i, kIsNotRequired) * nonreq_da_[k][i];
					demand_expr += g_->GetReverseDeadheadDemand(i, kIsNotRequired) * nonreq_da_rev_[k][i];
				}
				model->addConstr(demand_expr <= Q_, "demand_constraints_" + std::to_string(k));
			}
		}

		void SymmetryConstraints() {
			for(size_t k = 0; k < K_; ++k) {
				std::vector <GRBLinExpr> symmetry_constraints(n_);
				for(size_t i = 0; i < n_; ++i) {
					symmetry_constraints[i] = 0.0;
				}
				size_t tail_index, head_index;
				for(size_t i = 0; i < m_; ++i) {
					g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsRequired);
					symmetry_constraints[head_index] += (req_sa_[k][i] + req_da_[k][i]);
					symmetry_constraints[tail_index] -= (req_sa_[k][i] + req_da_[k][i]);
					symmetry_constraints[tail_index] += (req_sa_rev_[k][i] + req_da_rev_[k][i]);
					symmetry_constraints[head_index] -= (req_sa_rev_[k][i] + req_da_rev_[k][i]);
				}
				for(size_t i = 0; i < m_nr_; ++i) {
					g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsNotRequired);
					symmetry_constraints[head_index] += nonreq_da_[k][i];
					symmetry_constraints[tail_index] -= nonreq_da_[k][i];
					symmetry_constraints[tail_index] += nonreq_da_rev_[k][i];
					symmetry_constraints[head_index] -= nonreq_da_rev_[k][i];
				}
				for(size_t i = 0; i < n_; ++i) {
					model->addConstr(symmetry_constraints[i] == 0, "symmetry_constraints_" + std::to_string(k) + "_" + std::to_string(i));
				}
			}
		}

		void FlowBalance() {
			for(size_t k = 0; k < K_; ++k) {
				std::vector <GRBLinExpr> flow_balance(n_);
				size_t tail_index, head_index;
				for(size_t i = 0; i < m_; ++i) {
					g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsRequired);
					flow_balance[head_index] += req_za_[k][i];
					flow_balance[tail_index] -= req_za_[k][i];
					flow_balance[tail_index] += req_za_rev_[k][i];
					flow_balance[head_index] -= req_za_rev_[k][i];

					flow_balance[head_index] -= req_sa_[k][i];
					flow_balance[tail_index] -= req_sa_rev_[k][i];
				}
				for(size_t i = 0; i < m_nr_; ++i) {
					g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsNotRequired);
					flow_balance[head_index] += nonreq_za_[k][i];
					flow_balance[tail_index] -= nonreq_za_[k][i];
					flow_balance[tail_index] += nonreq_za_rev_[k][i];
					flow_balance[head_index] -= nonreq_za_rev_[k][i];
				}
				for(size_t i = 0; i < n_; ++i) {
					if(i == depot_index_) {
						continue;
					}
					model->addConstr(flow_balance[i] == 0, "flow_balance_constraints_" + std::to_string(k) + "_"+ std::to_string(i));
				}
			}
		}

		void DepotFlowConstraint() {
			for(size_t k = 0; k < K_; ++k) {
				GRBLinExpr depot_flow_constraint;
				GRBLinExpr num_service_expr;
				size_t tail_index, head_index;
				for(size_t i = 0; i < m_; ++i) {
					num_service_expr += req_sa_[k][i];
					num_service_expr += req_sa_rev_[k][i];
					g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsRequired);
					if(tail_index == depot_index_) {
						depot_flow_constraint += req_za_[k][i];
					}
					if(head_index == depot_index_) {
						depot_flow_constraint += req_za_rev_[k][i];
					}
				}
				for(size_t i = 0; i < m_nr_; ++i) {
					g_->GetVerticesIndexOfEdge(i, tail_index, head_index, kIsNotRequired);
					if(tail_index == depot_index_) {
						depot_flow_constraint += nonreq_za_[k][i];
					}
					if(head_index == depot_index_) {
						depot_flow_constraint += nonreq_za_rev_[k][i];
					}
				}
				model->addConstr(depot_flow_constraint == num_service_expr, "depot_flow_constraint_" + std::to_string(k));
			}
		}

		void FlowLimit() {
			for(size_t k = 0; k < K_; ++k) {
				for(size_t i = 0; i < m_; ++i) {
					model->addConstr(req_za_[k][i] - m_ * (req_da_[k][i] + req_sa_[k][i]) <= 0, "flow_limit_req_" + std::to_string(k) + "_" + std::to_string(i));
					model->addConstr(req_za_rev_[k][i] - m_ * (req_da_rev_[k][i] + req_sa_rev_[k][i]) <= 0, "flow_limit_req_rev_" + std::to_string(k) + "_" + std::to_string(i));
				}
				for(size_t i = 0; i < m_nr_; ++i) {
					model->addConstr(nonreq_za_[k][i] - m_ * nonreq_da_[k][i] <= 0, "flow_limit_nonreq_" + std::to_string(k) + "_" + std::to_string(i));
					model->addConstr(nonreq_za_rev_[k][i] - m_ * nonreq_da_rev_[k][i] <= 0, "flow_limit_nonreq_rev_" + std::to_string(k) + "_" + std::to_string(i));
				}
			}
		}

		void BreakSymmetry() {
			for(size_t k = 0; k < (K_ - 1); ++k) {
				GRBLinExpr break_symmetry_expr_k;
				GRBLinExpr break_symmetry_expr_kp1;
				for(size_t i = 0; i < m_; ++i) {
					break_symmetry_expr_k += req_sa_[k][i];
					break_symmetry_expr_k += req_sa_rev_[k][i];
					break_symmetry_expr_k += req_da_[k][i];
					break_symmetry_expr_k += req_da_rev_[k][i];
					break_symmetry_expr_kp1 += req_sa_[k+1][i];
					break_symmetry_expr_kp1 += req_sa_rev_[k+1][i];
					break_symmetry_expr_kp1 += req_da_[k+1][i];
					break_symmetry_expr_kp1 += req_da_rev_[k+1][i];
				}
				for(size_t i = 0; i < m_nr_; ++i) {
					break_symmetry_expr_k += nonreq_da_[k][i];
					break_symmetry_expr_k += nonreq_da_rev_[k][i];
					break_symmetry_expr_kp1 += nonreq_da_[k+1][i];
					break_symmetry_expr_kp1 += nonreq_da_rev_[k+1][i];
				}
				model->addConstr(break_symmetry_expr_k >= break_symmetry_expr_kp1, "break_symmetry_constraint_" + std::to_string(k));
			}
		}

		int GenerateSolutionGraph() {
			if(exit_status_ == -1)
				return exit_status_;
			std::vector <Vertex> vertex_list;
			vertex_list.reserve(n_);
			for(size_t i = 0; i < n_; ++i) {
				Vertex v;
				g_->GetVertexData(i, v);
				vertex_list.push_back(v);
			}

			for(size_t k = 0; k < K_; ++k) {
				std::vector <Edge> edge_list;
				edge_list.reserve(m_);
				size_t m, m_nr;
				m = 0; m_nr = 0;
				for(size_t i = 0; i < m_; ++i) {
					double s_a = req_sa_[k][i].get(GRB_DoubleAttr_X);
					double s_a_rev = req_sa_rev_[k][i].get(GRB_DoubleAttr_X);
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

					int d_a = std::round(req_da_[k][i].get(GRB_DoubleAttr_X));
					int d_a_rev = std::round(req_da_rev_[k][i].get(GRB_DoubleAttr_X));
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
					int d_a = std::round(nonreq_da_[k][i].get(GRB_DoubleAttr_X));
					int d_a_rev = std::round(nonreq_da_rev_[k][i].get(GRB_DoubleAttr_X));
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
				if(m == 0) {
					continue;
				}
				auto sol_digraph = std::make_shared <Graph> (vertex_list, edge_list, m, m_nr);
				sol_digraph_list_.push_back(sol_digraph);
			}
			return exit_status_;
		}
	};

}
#endif /* LCLIBRARY_MLC_ILP_GUROBI_H_ */

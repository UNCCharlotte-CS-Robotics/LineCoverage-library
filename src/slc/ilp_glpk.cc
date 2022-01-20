/**
 * This file is part of the LineCoverage-library.
 * The file contains SLC ILP formulation solved using GLPK
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

#include <lclibrary/slc/ilp_glpk.h>

namespace lclibrary {

	SLC_ILP_glpk::SLC_ILP_glpk (const std::shared_ptr <const Graph> g_in) : SLC_ILP(g_in) {

		coeff_count_ = 0;
		n_ = g_->GetN();
		m_ = g_->GetM();
		m_nr_ = g_->GetMnr();

		num_of_variables_ = 2 * m_ + 2 * (m_ + m_nr_) + 2 * (m_ + m_nr_); /* service, deadheading, and flow variables */

		number_of_coeff_ = 8 * m_ + 2 * m_ + 4 * m_nr_ + 6 * m_ + 4 * m_nr_ + m_ + m_nr_;
		vec_ia.reserve(number_of_coeff_);
		vec_ja.reserve(number_of_coeff_);
		vec_ar.reserve(number_of_coeff_);

		ilp_ = glp_create_prob();
		glp_set_prob_name(ilp_, "SLC_ILP");
		glp_set_obj_dir(ilp_, GLP_MIN);
	}

	int SLC_ILP_glpk::SolveILP () {
		AssignDepot();
		AddConstraintElement(0, 0, 0.0);
		AddVars();
		AddRows();
		SymmetryConstraints();
		ServiceConstraints();
		FlowConservation();
		DepotFlow();
		FlowLimit();
		TourCost();
		LoadConstraintMatrix();
		glp_iocp parm;
		glp_init_iocp(&parm);
		parm.presolve = GLP_ON;
		/* parm.ps_heur = GLP_ON; */
		parm.sr_heur = GLP_ON;
		parm.gmi_cuts = GLP_ON;
		parm.mir_cuts = GLP_ON;
		parm.cov_cuts = GLP_ON;
		parm.clq_cuts = GLP_ON;
		glp_intopt(ilp_, &parm);
		z_ = glp_mip_obj_val(ilp_);
		std::cout << "Tour Cost: " << z_ << std::endl;
		return 0;
	}

	void SLC_ILP_glpk::AssignDepot () {
		size_t tail_index, head_index;
		g_->GetVerticesIndexOfEdge(0, tail_index, head_index, true);
		depot_index_ = tail_index;
	}

	void SLC_ILP_glpk::FlowConservation () {
		size_t start_flow_index = 2 * m_ + 2 * m_ + 2 * m_nr_;
		size_t start_constraint = n_ + m_;
		for(size_t i = 1; i <= m_; ++i) {
			size_t tail_index, head_index;
			g_->GetVerticesIndexOfEdge(i - 1, tail_index, head_index, true);
			if(head_index != depot_index_) {
				AddConstraintElement(start_constraint + head_index + 1, 2 * (i - 1) + 1, -1.0);
				AddConstraintElement(start_constraint + head_index + 1, start_flow_index + 2 * (i - 1) + 1, 1.0);
				AddConstraintElement(start_constraint + head_index + 1, start_flow_index + 2 * (i - 1) + 2, -1.0);
			}
			if(tail_index != depot_index_) {
				AddConstraintElement(start_constraint + tail_index + 1, 2 * (i - 1) + 2, -1.0);
				AddConstraintElement(start_constraint + tail_index + 1, start_flow_index + 2 * (i - 1) + 1, -1.0);
				AddConstraintElement(start_constraint + tail_index + 1, start_flow_index + 2 * (i - 1) + 2, 1.0);
			}
		}

		size_t start_nr_flow_index = 2 * m_ + 2 * m_ + 2 * m_nr_ + 2 * m_;
		for(size_t i = 1; i <= m_nr_; ++i) {
			size_t tail_index, head_index;
			g_->GetVerticesIndexOfEdge(i - 1, tail_index, head_index, false);
			if(head_index != depot_index_) {
				AddConstraintElement(start_constraint + head_index + 1, start_nr_flow_index + 2 * (i - 1) + 1, 1.0);
				AddConstraintElement(start_constraint + head_index + 1, start_nr_flow_index + 2 * (i - 1) + 2, -1.0);
			}
			if(tail_index != depot_index_) {
				AddConstraintElement(start_constraint + tail_index + 1, start_nr_flow_index + 2 * (i - 1) + 1, -1.0);
				AddConstraintElement(start_constraint + tail_index + 1, start_nr_flow_index + 2 * (i - 1) + 2, 1.0);
			}
		}
	}

	void SLC_ILP_glpk::DepotFlow () {
		size_t start_flow_index = 2 * m_ + 2 * m_ + 2 * m_nr_;
		size_t start_constraint = n_ + m_ + n_ + 1;
		for(size_t i = 1; i <= m_; ++i) {
			size_t tail_index, head_index;
			g_->GetVerticesIndexOfEdge(i - 1, tail_index, head_index, true);
			if(head_index == depot_index_) {
				AddConstraintElement(start_constraint, start_flow_index + 2 * (i - 1) + 2, 1.0);
			}
			if(tail_index == depot_index_) {
				AddConstraintElement(start_constraint, start_flow_index + 2 * (i - 1) + 1, 1.0);
			}
		}

		size_t start_nr_flow_index = 2 * m_ + 2 * m_ + 2 * m_nr_ + 2 * m_;
		for(size_t i = 1; i <= m_nr_; ++i) {
			size_t tail_index, head_index;
			g_->GetVerticesIndexOfEdge(i - 1, tail_index, head_index, false);
			if(head_index == depot_index_) {
				AddConstraintElement(start_constraint, start_nr_flow_index + 2 * (i - 1) + 2, 1.0);
			}
			if(tail_index == depot_index_) {
				AddConstraintElement(start_constraint, start_nr_flow_index + 2 * (i - 1) + 1, 1.0);
			}
		}
	}

	void SLC_ILP_glpk::FlowLimit() {
		size_t start_flow_index = 2 * m_ + 2 * m_ + 2 * m_nr_;
		size_t start_constraint = n_ + m_ + n_ + 1;
		for(size_t i = 1; i <= m_; ++i) {
				AddConstraintElement(start_constraint + 2 * (i - 1) + 1, 2 * (i - 1) + 1, -1.0 * m_);
				AddConstraintElement(start_constraint + 2 * (i - 1) + 1, 2 * m_ + 2 * (i - 1) + 1, -1.0 * m_);
				AddConstraintElement(start_constraint + 2 * (i - 1) + 1, start_flow_index + 2 * (i - 1) + 1, 1.0);

				AddConstraintElement(start_constraint + 2 * (i - 1) + 2, 2 * (i - 1) + 2, -1.0 * m_);
				AddConstraintElement(start_constraint + 2 * (i - 1) + 2, 2 * m_ + 2 * (i - 1) + 2, -1.0 * m_);
				AddConstraintElement(start_constraint + 2 * (i - 1) + 2, start_flow_index + 2 * (i - 1) + 2, 1.0);
		}
		start_constraint += 2 * m_;
		size_t start_nr_flow_index = 2 * m_ + 2 * m_ + 2 * m_nr_ + 2 * m_;
		for(size_t i = 1; i <= m_nr_; ++i) {
				AddConstraintElement(start_constraint + 2 * (i - 1) + 1, 4 * m_ + 2 * (i - 1) + 1, -1.0 * m_);
				AddConstraintElement(start_constraint + 2 * (i - 1) + 1, start_nr_flow_index + 2 * (i - 1) + 1, 1.0);

				AddConstraintElement(start_constraint + 2 * (i - 1) + 2, 4 * m_ + 2 * (i - 1) + 2, -1.0 * m_);
				AddConstraintElement(start_constraint + 2 * (i - 1) + 2, start_nr_flow_index + 2 * (i - 1) + 2, 1.0);
			}
	}

	void SLC_ILP_glpk::SymmetryConstraints () {
		for(size_t i = 1; i <= m_; ++i) {
			size_t tail_index, head_index;
			g_->GetVerticesIndexOfEdge(i - 1, tail_index, head_index, true);
			AddConstraintElement(head_index + 1, 2 * (i - 1) + 1, 1.0);
			AddConstraintElement(tail_index + 1, 2 * (i - 1) + 1, -1.0);

			AddConstraintElement(head_index + 1, 2 * (i - 1) + 2, -1.0);
			AddConstraintElement(tail_index + 1, 2 * (i - 1) + 2, 1.0);

			AddConstraintElement(head_index + 1, 2 * m_ + 2 * (i - 1) + 1, 1.0);
			AddConstraintElement(tail_index + 1, 2 * m_ + 2 * (i - 1) + 1, -1.0);

			AddConstraintElement(head_index + 1, 2 * m_ + 2 * (i - 1) + 2, -1.0);
			AddConstraintElement(tail_index + 1, 2 * m_ + 2 * (i - 1) + 2, 1.0);
		}
		for(size_t i = 1; i <= m_nr_; ++i) {
			size_t tail_index, head_index;
			g_->GetVerticesIndexOfEdge(i - 1, tail_index, head_index, false);
			AddConstraintElement(head_index + 1, (2 * m_ + 2 * m_) + 2 * (i - 1) + 1, 1.0);
			AddConstraintElement(tail_index + 1, (2 * m_ + 2 * m_) + 2 * (i - 1) + 1, -1.0);

			AddConstraintElement(head_index + 1, (2 * m_ + 2 * m_) + 2 * (i - 1) + 2, -1.0);
			AddConstraintElement(tail_index + 1, (2 * m_ + 2 * m_) + 2 * (i - 1) + 2, 1.0);
		}
		constraint_count_ = n_;
	}

	void SLC_ILP_glpk::ServiceConstraints () {
		for(size_t i = 1; i <= m_; ++i) {
			AddConstraintElement(n_ + i, 2 * (i - 1) + 1, 1.0);
			AddConstraintElement(n_ + i, 2 * (i - 1) + 2, 1.0);
		}
	}

	void SLC_ILP_glpk::AddConstraintElement(int ia, int ja, double ar) {
		vec_ia.push_back(ia);
		vec_ja.push_back(ja);
		vec_ar.push_back(ar);
	}

	void SLC_ILP_glpk::TourCost () {

		/* Service variables s_e and s_{\bar e} for each (e, \bar e) \in E_r*/
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_obj_coef(ilp_, 2 * (i - 1) + 1, g_->GetServiceCost(i - 1));
			glp_set_obj_coef(ilp_, 2 * (i - 1) + 2, g_->GetReverseServiceCost(i - 1));
		}

		/* Deadheading variables d_e and d_{\bar e} for each (e, \bar e) \in E*/
		size_t start_deadheading_index = 2 * m_;
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_obj_coef(ilp_, start_deadheading_index + 2 * (i - 1) + 1, g_->GetDeadheadCost(i - 1, true));
			glp_set_obj_coef(ilp_, start_deadheading_index + 2 * (i - 1) + 2, g_->GetReverseDeadheadCost(i - 1, true));
		}

		size_t start_nr_deadheading_index = start_deadheading_index + 2 * m_;
		for(size_t i = 1; i <= m_nr_; ++i) {
			glp_set_obj_coef(ilp_, start_nr_deadheading_index + 2 * (i - 1) + 1, g_->GetDeadheadCost(i - 1, false));
			glp_set_obj_coef(ilp_, start_nr_deadheading_index + 2 * (i - 1) + 2, g_->GetReverseDeadheadCost(i - 1, false));
		}
	}

	void SLC_ILP_glpk::AddVars () {
		glp_add_cols(ilp_, num_of_variables_);
		/* Service variables s_e and s_{\bar e} for each (e, \bar e) \in E_r*/
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_col_bnds(ilp_, 2 * (i - 1) + 1, GLP_DB, 0, 1);
			glp_set_col_bnds(ilp_, 2 * (i - 1) + 2, GLP_DB, 0, 1);
			glp_set_col_kind(ilp_, 2 * (i - 1) + 1, GLP_BV);
			glp_set_col_kind(ilp_, 2 * (i - 1) + 2, GLP_BV);
		}

		/* Deadheading variables d_e and d_{\bar e} for each (e, \bar e) \in E*/
		size_t start_deadheading_index = 2 * m_;
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_col_bnds(ilp_, start_deadheading_index + 2 * (i - 1) + 1, GLP_LO, 0, 0);
			glp_set_col_bnds(ilp_, start_deadheading_index + 2 * (i - 1) + 2, GLP_LO, 0, 0);
			glp_set_col_kind(ilp_, start_deadheading_index + 2 * (i - 1) + 1, GLP_IV);
			glp_set_col_kind(ilp_, start_deadheading_index + 2 * (i - 1) + 2, GLP_IV);
		}

		size_t start_nr_deadheading_index = start_deadheading_index + 2 * m_;
		for(size_t i = 1; i <= m_nr_; ++i) {
			glp_set_col_bnds(ilp_, start_nr_deadheading_index + 2 * (i - 1) + 1, GLP_LO, 0, 0);
			glp_set_col_bnds(ilp_, start_nr_deadheading_index + 2 * (i - 1) + 2, GLP_LO, 0, 0);
			glp_set_col_kind(ilp_, start_nr_deadheading_index + 2 * (i - 1) + 1, GLP_IV);
			glp_set_col_kind(ilp_, start_nr_deadheading_index + 2 * (i - 1) + 2, GLP_IV);
		}

		/* Flow variables d_e and d_{\bar e} for each (e, \bar e) \in E*/
		size_t start_flow_index = start_nr_deadheading_index + 2 * m_nr_;
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_col_bnds(ilp_, start_flow_index + 2 * (i - 1) + 1, GLP_LO, 0, 0);
			glp_set_col_bnds(ilp_, start_flow_index + 2 * (i - 1) + 2, GLP_LO, 0, 0);
			glp_set_col_kind(ilp_, start_flow_index + 2 * (i - 1) + 1, GLP_IV);
			glp_set_col_kind(ilp_, start_flow_index + 2 * (i - 1) + 2, GLP_IV);
		}

		size_t start_nr_flow_index = start_flow_index + 2 * m_;
		for(size_t i = 1; i <= m_nr_; ++i) {
			glp_set_col_bnds(ilp_, start_nr_flow_index + 2 * (i - 1) + 1, GLP_LO, 0, 0);
			glp_set_col_bnds(ilp_, start_nr_flow_index + 2 * (i - 1) + 2, GLP_LO, 0, 0);
			glp_set_col_kind(ilp_, start_nr_flow_index + 2 * (i - 1) + 1, GLP_IV);
			glp_set_col_kind(ilp_, start_nr_flow_index + 2 * (i - 1) + 2, GLP_IV);
		}
	}

	void SLC_ILP_glpk::AddRows () {
		num_of_constraints_ = n_ + m_ + n_ + 1 + 2 * m_ + 2 * m_nr_;
		glp_add_rows(ilp_, num_of_constraints_);
		for(size_t i = 1; i <= n_; ++i) {
			glp_set_row_bnds(ilp_, i, GLP_FX, 0, 0);
		}
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_row_bnds(ilp_, n_ + i, GLP_FX, 1, 1);
		}
		for(size_t i = 1; i <= n_; ++i) {
			glp_set_row_bnds(ilp_, n_ + m_ + i, GLP_FX, 0, 0);
		}
		glp_set_row_bnds(ilp_, n_ + m_ + n_ + 1, GLP_FX, m_, m_);
		for(size_t i = 1; i <= 2 * m_; ++i) {
			glp_set_row_bnds(ilp_, n_ + m_ + n_ + 1 + i, GLP_UP, 0.0, 0.0);
		}
		for(size_t i = 1; i <= 2 * m_nr_; ++i) {
			glp_set_row_bnds(ilp_, n_ + m_ + n_ + 1 + 2 * m_ + i, GLP_UP, 0.0, 0.0);
		}

	}

	void SLC_ILP_glpk::LoadConstraintMatrix () {
		number_of_coeff_ = vec_ia.size();
		ia_ = new int[number_of_coeff_];
		ja_ = new int[number_of_coeff_];
		ar_ = new double[number_of_coeff_];
		for (size_t i = 0; i < number_of_coeff_; ++i) {
			ia_[i] = vec_ia[i];
			ja_[i] = vec_ja[i];
			ar_[i] = vec_ar[i];
		}
		glp_load_matrix(ilp_, number_of_coeff_ - 1, ia_, ja_, ar_);
	}

	int SLC_ILP_glpk::GenerateSolutionGraph () {

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
		for(size_t i = 1; i <= m_; ++i) {
			bool s_a = std::round(glp_mip_col_val(ilp_, 2 * (i - 1) + 1));
			bool s_abar = std::round(glp_mip_col_val(ilp_, 2 * (i - 1) + 2));
			Edge edge;
			g_->GetEdgeData(i - 1, edge, true);
			if(s_a) {
				Edge new_edge(edge.GetTailVertexID(), edge.GetHeadVertexID(), true);
				new_edge.SetCost(edge.GetServiceCost());
				new_edge.SetReq(true);
				edge_list.push_back(new_edge);
				++m;
			}
			if(s_abar) {
				Edge new_edge(edge.GetTailVertexID(), edge.GetHeadVertexID(), true);
				new_edge.SetCost(edge.GetReverseServiceCost());
				new_edge.SetReq(true);
				new_edge.Reverse();
				edge_list.push_back(new_edge);
				++m;
			}

			int d_a = std::round(glp_mip_col_val(ilp_, 2 * m_ + 2 * (i - 1) + 1));
			int d_abar = std::round(glp_mip_col_val(ilp_, 2 * m_ + 2 * (i - 1) + 2));
			if(d_a > 0) {
				for(int j = 1; j <= d_a; ++j) {
					Edge new_edge(edge.GetTailVertexID(), edge.GetHeadVertexID(), true);
					new_edge.SetCost(edge.GetDeadheadCost());
					new_edge.SetReq(false);
					edge_list.push_back(new_edge);
					++m_nr;
				}
			}
			if(d_abar > 0) {
				for(int j = 1; j <= d_abar; ++j) {
					Edge new_edge(edge.GetTailVertexID(), edge.GetHeadVertexID(), true);
					new_edge.SetCost(edge.GetReverseDeadheadCost());
					new_edge.SetReq(false);
					new_edge.Reverse();
					edge_list.push_back(new_edge);
					++m_nr;
				}
			}
		}

		for(size_t i = 1; i <= m_nr_; ++i) {
			int d_a = std::round(glp_mip_col_val(ilp_, 4 * m_ + 2 * (i - 1) + 1));
			int d_abar = std::round(glp_mip_col_val(ilp_, 4 * m_ + 2 * (i - 1) + 2));
			Edge edge;
			g_->GetEdgeData(i - 1, edge, false);
			if(d_a > 0) {
				for(int j = 1; j <= d_a; ++j) {
					Edge new_edge(edge.GetTailVertexID(), edge.GetHeadVertexID(), false);
					new_edge.SetCost(edge.GetDeadheadCost());
					new_edge.SetReq(false);
					edge_list.push_back(new_edge);
					++m_nr;
				}
			}
			if(d_abar > 0) {
				for(int j = 1; j <= d_abar; ++j) {
					Edge new_edge(edge.GetTailVertexID(), edge.GetHeadVertexID(), false);
					new_edge.SetCost(edge.GetReverseDeadheadCost());
					new_edge.SetReq(false);
					new_edge.Reverse();
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
		if (std::fabs(cost - z_) > 1e-10) {
			std::cerr << "Mismatch in cost: " << cost << " " << z_ << std::endl;
		}
		std::cout << edge_list.size() << " " << m << " " << m_nr << " " << req_count<<std::endl;
		sol_digraph_ = std::make_shared <Graph> (vertex_list, edge_list, m, m_nr);
		return 0;
	}

	void SLC_ILP_glpk::PrintFlowValues() {
		size_t start_flow_index = 4 * m_ + 2 * m_nr_;
		for(size_t i = 1; i <= m_; ++i) {
			int f_a = std::round(glp_mip_col_val(ilp_, start_flow_index + 2 * (i - 1) + 1));
			int f_abar = std::round(glp_mip_col_val(ilp_, start_flow_index + 2 * (i - 1) + 2));
			Edge edge;
			g_->GetEdgeData(i - 1, edge, true);
			std::cout << edge.GetTailVertexID() << " " << edge.GetHeadVertexID() << " " << f_a << " " << f_abar << "\n";
		}

		size_t start_nr_flow_index = start_flow_index + 2 * m_;
		for(size_t i = 1; i <= m_nr_; ++i) {
			int f_a = std::round(glp_mip_col_val(ilp_, start_nr_flow_index + 2 * (i - 1) + 1));
			int f_abar = std::round(glp_mip_col_val(ilp_, start_nr_flow_index + 2 * (i - 1) + 2));
			Edge edge;
			g_->GetEdgeData(i - 1, edge, false);
			std::cout << edge.GetTailVertexID() << " " << edge.GetHeadVertexID() << " " << f_a << " " << f_abar << "\n";
		}
	}

} /* lclibrary */

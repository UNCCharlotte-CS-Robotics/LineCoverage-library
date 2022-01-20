/*!
 * The file contains SLC beta3 heuristic solver using GLPK
 * @author Saurav Agarwal
 * @contact sagarw10@uncc.edu
 */

#include <lclibrary/slc/lp_beta3_glpk.h>

namespace lclibrary {

	SLC_LP_beta3_glpk::SLC_LP_beta3_glpk (const std::shared_ptr <const Graph> g_in) : g_{g_in} {

		coeff_count_ = 0;
		n_ = g_->GetN();
		m_ = g_->GetM();
		m_nr_ = g_->GetMnr();

		num_of_variables_ = 2 * m_ + 2 * (m_ + m_nr_); /* service, deadheading */

		number_of_coeff_ = 8 * m_ + 2 * m_ + 4 * m_nr_;
		vec_ia.reserve(number_of_coeff_);
		vec_ja.reserve(number_of_coeff_);
		vec_ar.reserve(number_of_coeff_);

		lp_ = glp_create_prob();
		glp_set_prob_name(lp_, "SLC_ILP");
		glp_set_obj_dir(lp_, GLP_MIN);
	}

	int SLC_LP_beta3_glpk::Solve () {
		AddConstraintElement(0, 0, 0.0);
		AddVars();
		AddRows();
		SymmetryConstraints();
		ServiceConstraints();
		TourCost();
		LoadConstraintMatrix();
		glp_iocp parm;
		glp_init_iocp(&parm);
		parm.presolve = GLP_ON;
		parm.gmi_cuts = GLP_ON;
		glp_simplex(lp_, NULL);
		z_ = glp_get_obj_val(lp_);
		std::cout << "LP cost: " << z_ << std::endl;
		return 0;
	}

	void SLC_LP_beta3_glpk::SymmetryConstraints () {
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

	void SLC_LP_beta3_glpk::ServiceConstraints () {
		for(size_t i = 1; i <= m_; ++i) {
			if(g_->GetServiceCost(i - 1) < g_->GetReverseServiceCost(i - 1)) {
				AddConstraintElement(n_ + i, 2 * (i - 1) + 1, 1.0);
			} else {
				AddConstraintElement(n_ + i, 2 * (i - 1) + 2, 1.0);
			}
		}
	}

	void SLC_LP_beta3_glpk::AddConstraintElement(int ia, int ja, double ar) {
		vec_ia.push_back(ia);
		vec_ja.push_back(ja);
		vec_ar.push_back(ar);
	}

	void SLC_LP_beta3_glpk::TourCost () {

		/* Service variables s_e and s_{\bar e} for each (e, \bar e) \in E_r*/
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_obj_coef(lp_, 2 * (i - 1) + 1, g_->GetServiceCost(i - 1));
			glp_set_obj_coef(lp_, 2 * (i - 1) + 2, g_->GetReverseServiceCost(i - 1));
		}

		/* Deadheading variables d_e and d_{\bar e} for each (e, \bar e) \in E*/
		size_t start_deadheading_index = 2 * m_;
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_obj_coef(lp_, start_deadheading_index + 2 * (i - 1) + 1, g_->GetDeadheadCost(i - 1, true));
			glp_set_obj_coef(lp_, start_deadheading_index + 2 * (i - 1) + 2, g_->GetReverseDeadheadCost(i - 1, true));
		}

		size_t start_nr_deadheading_index = start_deadheading_index + 2 * m_;
		for(size_t i = 1; i <= m_nr_; ++i) {
			glp_set_obj_coef(lp_, start_nr_deadheading_index + 2 * (i - 1) + 1, g_->GetDeadheadCost(i - 1, false));
			glp_set_obj_coef(lp_, start_nr_deadheading_index + 2 * (i - 1) + 2, g_->GetReverseDeadheadCost(i - 1, false));
		}
	}

	void SLC_LP_beta3_glpk::AddVars () {
		glp_add_cols(lp_, num_of_variables_);
		/* Service variables s_e and s_{\bar e} for each (e, \bar e) \in E_r*/
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_col_bnds(lp_, 2 * (i - 1) + 1, GLP_DB, 0, 1);
			glp_set_col_bnds(lp_, 2 * (i - 1) + 2, GLP_DB, 0, 1);
			glp_set_col_kind(lp_, 2 * (i - 1) + 1, GLP_BV);
			glp_set_col_kind(lp_, 2 * (i - 1) + 2, GLP_BV);
		}

		/* Deadheading variables d_e and d_{\bar e} for each (e, \bar e) \in E*/
		size_t start_deadheading_index = 2 * m_;
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_col_bnds(lp_, start_deadheading_index + 2 * (i - 1) + 1, GLP_LO, 0, 0);
			glp_set_col_bnds(lp_, start_deadheading_index + 2 * (i - 1) + 2, GLP_LO, 0, 0);
			glp_set_col_kind(lp_, start_deadheading_index + 2 * (i - 1) + 1, GLP_IV);
			glp_set_col_kind(lp_, start_deadheading_index + 2 * (i - 1) + 2, GLP_IV);
		}

		size_t start_nr_deadheading_index = start_deadheading_index + 2 * m_;
		for(size_t i = 1; i <= m_nr_; ++i) {
			glp_set_col_bnds(lp_, start_nr_deadheading_index + 2 * (i - 1) + 1, GLP_LO, 0, 0);
			glp_set_col_bnds(lp_, start_nr_deadheading_index + 2 * (i - 1) + 2, GLP_LO, 0, 0);
			glp_set_col_kind(lp_, start_nr_deadheading_index + 2 * (i - 1) + 1, GLP_IV);
			glp_set_col_kind(lp_, start_nr_deadheading_index + 2 * (i - 1) + 2, GLP_IV);
		}

	}

	void SLC_LP_beta3_glpk::AddRows () {
		num_of_constraints_ = n_ + m_ + n_ + 1 + 2 * m_ + 2 * m_nr_;
		glp_add_rows(lp_, num_of_constraints_);
		for(size_t i = 1; i <= n_; ++i) {
			glp_set_row_bnds(lp_, i, GLP_FX, 0, 0);
		}
		for(size_t i = 1; i <= m_; ++i) {
			glp_set_row_bnds(lp_, n_ + i, GLP_FX, 1, 1);
		}
	}

	void SLC_LP_beta3_glpk::LoadConstraintMatrix () {
		number_of_coeff_ = vec_ia.size();
		ia_ = new int[number_of_coeff_];
		ja_ = new int[number_of_coeff_];
		ar_ = new double[number_of_coeff_];
		for (size_t i = 0; i < number_of_coeff_; ++i) {
			ia_[i] = vec_ia[i];
			ja_[i] = vec_ja[i];
			ar_[i] = vec_ar[i];
		}
		glp_load_matrix(lp_, number_of_coeff_ - 1, ia_, ja_, ar_);
	}

	int SLC_LP_beta3_glpk::GenerateSolutionGraph (std::shared_ptr <Graph> &digraph, std::shared_ptr<Graph> &undirected_graph) {

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
			double s_a = glp_get_col_prim(lp_, 2 * (i - 1) + 1);
			double s_abar = glp_get_col_prim(lp_, 2 * (i - 1) + 2);
			Edge edge;
			g_->GetEdgeData(i - 1, edge, kIsRequired);
			if(IsNearZero(s_a - 1.0, 1e-5)) {
				/* Edge new_edge(edge.GetTailVertexID(), edge.GetHeadVertexID(), true); */
				Edge new_edge = edge;
				new_edge.SetCost(edge.GetServiceCost());
				new_edge.SetReq(kIsRequired);
				edge_list.push_back(new_edge);
				++m;
			}
			if(IsNearZero(s_abar - 1.0, 1e-5)) {
				Edge new_edge = edge;
				new_edge.SetCost(edge.GetReverseServiceCost());
				new_edge.SetReq(kIsRequired);
				new_edge.Reverse();
				edge_list.push_back(new_edge);
				++m;
			}

			int d_a = std::round(glp_get_col_prim(lp_, 2 * m_ + 2 * (i - 1) + 1));
			int d_abar = std::round(glp_get_col_prim(lp_, 2 * m_ + 2 * (i - 1) + 2));
			if(d_a > 0) {
				for(int j = 1; j <= d_a; ++j) {
					Edge new_edge = edge;
					new_edge.SetCost(edge.GetDeadheadCost());
					new_edge.SetReq(kIsNotRequired);
					edge_list.push_back(new_edge);
					++m_nr;
				}
			}
			if(d_abar > 0) {
				for(int j = 1; j <= d_abar; ++j) {
					Edge new_edge = edge;
					new_edge.SetCost(edge.GetReverseDeadheadCost());
					new_edge.SetReq(kIsNotRequired);
					new_edge.Reverse();
					edge_list.push_back(new_edge);
					++m_nr;
				}
			}
		}

		for(size_t i = 1; i <= m_nr_; ++i) {
			int d_a = std::round(glp_get_col_prim(lp_, 4 * m_ + 2 * (i - 1) + 1));
			int d_abar = std::round(glp_get_col_prim(lp_, 4 * m_ + 2 * (i - 1) + 2));
			Edge edge;
			g_->GetEdgeData(i - 1, edge, false);
			if(d_a > 0) {
				for(int j = 1; j <= d_a; ++j) {
					Edge new_edge = edge;
					new_edge.SetCost(edge.GetDeadheadCost());
					new_edge.SetReq(kIsNotRequired);
					edge_list.push_back(new_edge);
					++m_nr;
				}
			}
			if(d_abar > 0) {
				for(int j = 1; j <= d_abar; ++j) {
					Edge new_edge = edge;
					new_edge.SetCost(edge.GetReverseDeadheadCost());
					new_edge.SetReq(kIsNotRequired);
					new_edge.Reverse();
					edge_list.push_back(new_edge);
					++m_nr;
				}
			}
		}
		digraph = std::make_shared <Graph>(vertex_list, edge_list, m, m_nr);
		return 0;
	}

} /* lclibrary */

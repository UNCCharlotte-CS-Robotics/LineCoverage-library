/**
 * This file is part of the LineCoverage-library.
 * The file contains functions to generate all pair shortest paths
 *
 * TODO: Use GraphEdge instead of creating new edges?
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

#ifndef LCLIBRARY_ALGORITHMS_APSP_FLOYDWARSHALL_H_
#define LCLIBRARY_ALGORITHMS_APSP_FLOYDWARSHALL_H_

#include <lclibrary/core/constants.h>
#include <lclibrary/core/typedefs.h>
#include <lclibrary/core/vertex.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/algorithms/apsp_base.h>
#include <memory>

namespace lclibrary {

	class APSP_FloydWarshall : public APSP {
		std::shared_ptr <const Graph> g_;
		size_t n_;
		size_t m_;
		size_t m_nr_;
		std::vector <std::vector <double> > distance_;
		std::vector <std::vector <double> > demand_;
		std::vector <std::vector <size_t> > helper_;
		std::vector <std::vector < EdgeTuple> > helper_edge_;
		bool compute_demand_ = false;

		void Initialize() {
			distance_.resize(n_, std::vector <double> (n_, kDoubleMax));
			helper_.resize(n_, std::vector <size_t> (n_, kNIL));
			helper_edge_.resize(n_, std::vector <EdgeTuple>(n_, MakeEdgeTuple(nullptr, kIsRequired)));
			if(compute_demand_) {
				demand_.resize(n_, std::vector <double> (n_, kDoubleMax));
			}
		}

		void GetPath(std::vector < EdgeTuple > &path, size_t i, size_t j) const {
			if(helper_[i][j] == kNIL)
				path.push_back(helper_edge_[i][j]);
			else {
				GetPath(path, i, helper_[i][j]);
				GetPath(path, helper_[i][j], j);
			}
		}

		public:
		APSP_FloydWarshall(std::shared_ptr <const Graph> &g) : g_{g} {
			n_ = g_->GetN();
			m_ = g_->GetM();
			m_nr_ = g_->GetMnr();
			Initialize();
		}

		APSP_FloydWarshall(std::shared_ptr <const Graph> &g, bool compute_demand) : g_{g}, compute_demand_{compute_demand} {
			n_ = g_->GetN();
			m_ = g_->GetM();
			m_nr_ = g_->GetMnr();
			Initialize();
		}

		void APSP_Deadheading() {
			double cost = kDoubleMax;
			for(size_t i = 0; i < n_; ++i) {
				distance_[i][i] = 0;
			}
			if(compute_demand_) {
				for(size_t i = 0; i < n_; ++i) {
					demand_[i][i] = 0;
				}
			}
			for(size_t i = 0; i < m_; ++i) {
				size_t t, h;
				g_->GetVerticesIndexOfEdge(i, t, h, kIsRequired);
				cost = g_->GetDeadheadCost(i, kIsRequired);
				if (cost < distance_[t][h]) {
					distance_[t][h] = cost;
					helper_edge_[t][h] = MakeEdgeTuple(g_->GetEdge(i, kIsRequired), false);
					if(compute_demand_) {
						demand_[t][h] = g_->GetDeadheadDemand(i, kIsRequired);
					}
				}
				cost = g_->GetReverseDeadheadCost(i, kIsRequired);
				if (cost < distance_[h][t]) {
					distance_[h][t] = cost;
					helper_edge_[h][t] = MakeEdgeTuple(g_->GetEdge(i, kIsRequired), true);
					if(compute_demand_) {
						demand_[h][t] = g_->GetReverseDeadheadDemand(i, kIsRequired);
					}
				}
			}
			for(size_t i = 0; i < m_nr_; ++i) {
				size_t t, h;
				g_->GetVerticesIndexOfEdge(i, t, h, kIsNotRequired);
				cost = g_->GetDeadheadCost(i, kIsNotRequired);
				if (cost < distance_[t][h]) {
					distance_[t][h] = cost;
					helper_edge_[t][h] = MakeEdgeTuple(g_->GetEdge(i, kIsNotRequired), false);
					if(compute_demand_) {
						demand_[t][h] = g_->GetDeadheadDemand(i, kIsNotRequired);
					}
				}
				cost = g_->GetReverseDeadheadCost(i, kIsNotRequired);
				if (cost < distance_[h][t]) {
					distance_[h][t] = cost;
					helper_edge_[h][t] = MakeEdgeTuple(g_->GetEdge(i, kIsNotRequired), true);
					if(compute_demand_) {
						demand_[h][t] = g_->GetReverseDeadheadDemand(i, kIsNotRequired);
					}
				}
			}

			for(size_t k = 0; k < n_; ++k) {
				for(size_t i = 0; i < n_; ++i) {
					for(size_t j = 0; j < n_; ++j) {
						if(distance_[i][k] + distance_[k][j] < distance_[i][j]) {
							distance_[i][j] = distance_[i][k] + distance_[k][j];
							helper_[i][j] = k;
							if(compute_demand_) {
								demand_[i][j] = demand_[i][k] + demand_[k][j];
							}
						}
					}
				}
			}
		}

		void GetPath(std::vector < Edge > &edge_list, const size_t i, const size_t j) const {
			std::vector <EdgeTuple> path;
			GetPath(path, i, j);
			for(auto &[e, is_rev] : path) {
				if(e == nullptr)
					continue;
				Edge new_edge = *e;
				new_edge.SetReq(kIsNotRequired);
				if(is_rev) {
					new_edge.SetCost(e->GetReverseDeadheadCost());
					new_edge.Reverse();
				}
				else {
					new_edge.SetCost(e->GetDeadheadCost());
				}
				edge_list.push_back(new_edge);
			}
		}

		double GetCost(const size_t i, const size_t j) const {
			return distance_[i][j];
		}

		double GetDemand(const size_t i, const size_t j) const {
			return demand_[i][j];
		}

	};

} // namespace lclibrary

#endif /* LCLIBRARY_ALGORITHMS_APSP_FLOYDWARSHALL_H_ */

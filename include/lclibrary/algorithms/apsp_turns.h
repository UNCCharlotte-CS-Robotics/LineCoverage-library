/**
 * This file is part of the LineCoverage-library.
 * The file contains functions to generate all pair shortest paths with turning costs
 *
 * TODO: Add APSP_Turns as another base class? Perhaps use traits?
 * Work in progress
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

#ifndef LCLIBRARY_ALGORITHMS_APSP_TURNS_H_
#define LCLIBRARY_ALGORITHMS_APSP_TURNS_H_

#include <memory>
#include <unordered_map>
#include <lclibrary/core/constants.h>
#include <lclibrary/core/typedefs.h>
#include <lclibrary/core/vertex.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/algorithms/apsp_base.h>
#include <lclibrary/utils/edge_cost_with_circular_turns.h>

namespace lclibrary {

	class APSP_Turns : public APSP {
		std::shared_ptr <const Graph> g_;
		size_t n_;
		size_t m_;
		size_t m_nr_;
		size_t num_deadheading_arcs_;
		size_t n_depots_;

		struct ArcData {
			const Edge* edge_ = nullptr;
			bool req_;
			bool rev_;
			size_t t_ID_, h_ID_;
			size_t g_index_;
			double deadhead_cost_ = kDoubleMax;
			std::vector <size_t> outgoing_arcs_;
			std::vector <double> out_req_cost_;
			std::vector <size_t> incoming_arcs_;
			std::vector <double> in_req_turn_cost_;
			ArcData(const Edge* e, bool req_in, bool rev_in, size_t t, size_t h) : edge_(e), req_(req_in), rev_(rev_in), t_ID_(t), h_ID_(h) {}
		};

		struct ShortestPathReq {
			int status_ = 0;
			size_t start_deadheading_ = kNIL, end_deadheading_ = kNIL;
			double cost_ = kDoubleMax;
		};

		struct DepotData {
			size_t ID_;
			std::vector <size_t> outgoing_arcs_;
			std::vector <size_t> incoming_arcs_;
			std::vector <ShortestPathReq> distance_depot_to_req_;
			std::vector <ShortestPathReq> distance_req_to_depot_;
			DepotData(size_t id):ID_(id) {}
		};

		std::shared_ptr <const EdgeCost_CircularTurns> cost_fn_;
		std::vector <size_t> depots_IDs_;
		std::unordered_map <size_t, size_t> depot_map_; /*! Stores a map of the index of depots to actual ID of the depot <ID, index>*/

		std::vector <std::vector <double> > distance_;
		std::vector <std::vector <ShortestPathReq> > distance_req_;
		std::vector <std::vector <size_t> > helper_;
		std::vector <std::vector < EdgeTuple> > helper_edge_;

		std::vector <ArcData> all_arcs_;
		std::vector <DepotData> depots_;

		void GenerateAllDeadheadingArcs() {
			for(size_t reqi = 0; reqi < m_; ++reqi) {
				size_t t, h;
				g_->GetVerticesIDOfEdge(reqi, t, h, kIsRequired);
				all_arcs_.push_back(ArcData(g_->GetEdge(reqi, kIsRequired), kIsRequired, false, t, h));
				all_arcs_[reqi].deadhead_cost_ = g_->GetDeadheadCost(reqi, kIsRequired);
				all_arcs_[reqi].g_index_ = reqi;
			}
			for(size_t reqi = 0; reqi < m_; ++reqi) {
				size_t t, h;
				g_->GetVerticesIDOfEdge(reqi, t, h, kIsRequired);
				all_arcs_.push_back(ArcData(g_->GetEdge(reqi, kIsRequired), kIsRequired, true, h, t));
				all_arcs_[m_ + reqi].deadhead_cost_ = g_->GetReverseDeadheadCost(reqi, kIsRequired);
				all_arcs_[m_ + reqi].g_index_ = reqi;
			}
			for(size_t nreqi = 0; nreqi < m_nr_; ++nreqi) {
				size_t t, h;
				g_->GetVerticesIDOfEdge(nreqi, t, h, kIsNotRequired);
				all_arcs_.push_back(ArcData(g_->GetEdge(nreqi, kIsNotRequired), kIsNotRequired, false, t, h));
				all_arcs_[2 * m_ + nreqi].deadhead_cost_ = g_->GetDeadheadCost(nreqi, kIsNotRequired);
				all_arcs_[2 * m_ + nreqi].g_index_ = nreqi;
			}
			for(size_t nreqi = 0; nreqi < m_nr_; ++nreqi) {
				size_t t, h;
				g_->GetVerticesIDOfEdge(nreqi, t, h, kIsNotRequired);
				all_arcs_.push_back(ArcData(g_->GetEdge(nreqi, kIsNotRequired), kIsNotRequired, true, h, t));
				all_arcs_[2 * m_ + m_nr_ + nreqi].deadhead_cost_ = g_->GetReverseDeadheadCost(nreqi, kIsNotRequired);
				all_arcs_[2 * m_ + m_nr_ + nreqi].g_index_ = nreqi;
			}
		}

		inline size_t EdgeToIndex(size_t e_i, bool req, bool rev) const {
			return 2 * m_ * size_t(not req) + size_t(rev) * (size_t(req) * m_ + size_t(not req) * m_nr_) + e_i;
		}

		inline void IndexToEdge(const size_t index, size_t &e_i, bool &req, bool &rev) const {
			if(index < m_) {
				e_i = index; req = kIsRequired; rev = false;
			} else if (index < 2 * m_) {
				e_i = index - m_; req = kIsRequired; rev = true;
			} else if (index < 2 * m_ + m_nr_) {
				e_i = index - 2 * m_; req = kIsNotRequired; rev = false;
			} else {
				e_i = index - 2 * m_ - m_nr_; req = kIsNotRequired; rev = true;
			}
		}

		void GenerateAdjacency() {
			for(size_t i = 0; i < m_; ++i) {
				ArcData& arc_i = all_arcs_[i];
				ArcData& rev_arc_i = all_arcs_[m_ + i];
				for(size_t j = 0; j < num_deadheading_arcs_; ++j) {
					const ArcData& arc_j = all_arcs_[j];
					if(arc_j.t_ID_ == arc_i.h_ID_) {
						if(arc_j.h_ID_ != arc_i.t_ID_) { // don't add if twin opposite arc
							arc_i.outgoing_arcs_.push_back(j);
						}
					}

					if(arc_j.t_ID_ == arc_i.t_ID_) { // for the opposite arc
						if(arc_j.h_ID_ != arc_i.h_ID_) { // don't add if twin opposite arc
							rev_arc_i.outgoing_arcs_.push_back(j);
						}
					}

					if(arc_j.h_ID_ == arc_i.t_ID_) {
						arc_i.incoming_arcs_.push_back(j);
					}

					if(arc_j.h_ID_ == arc_i.h_ID_) {
						rev_arc_i.incoming_arcs_.push_back(j);
					}
				}
			}

			for(size_t i = 0; i < m_nr_; ++i) {
				ArcData& arc_i = all_arcs_[2 * m_ + i];
				ArcData& rev_arc_i = all_arcs_[2 * m_ + m_nr_ + i];
				for(size_t j = 0; j < num_deadheading_arcs_; ++j) {
					const ArcData& arc_j = all_arcs_[j];
					if(arc_j.t_ID_ == arc_i.h_ID_) {
						if(arc_j.h_ID_ != arc_i.t_ID_) {
							arc_i.outgoing_arcs_.push_back(j);
						}
					}

					if(arc_j.t_ID_ == arc_i.t_ID_) {
						if(arc_j.h_ID_ != arc_i.h_ID_) {
							rev_arc_i.outgoing_arcs_.push_back(j);
						}
					}
				}
			}
		}

		void GetDepots() {
			std::vector <size_t> depot_indices;
			g_->GetDepotIDs(depots_IDs_);
			g_->GetDepotIndices(depot_indices);
			n_depots_ = depots_IDs_.size();
			std::cout << "Depots size: " << n_depots_ << std::endl;
			depots_.reserve(n_depots_);
			for(size_t i = 0; i < n_depots_; ++i) {
				std::cout << "GetDepots: " << depots_IDs_[i] << std::endl;
				depot_map_[depots_IDs_[i]] = i;
				depots_.push_back(DepotData(depots_IDs_[i]));
				depots_[i].distance_depot_to_req_.resize(2 * m_);
				depots_[i].distance_req_to_depot_.resize(2 * m_);
				for(size_t j = 0; j < num_deadheading_arcs_; ++j) {
					if(all_arcs_[j].t_ID_ == depots_IDs_[i]) {
						depots_[i].outgoing_arcs_.push_back(j);
					}
					if(all_arcs_[j].h_ID_ == depots_IDs_[i]) {
						depots_[i].incoming_arcs_.push_back(j);
					}
				}
			}
		}

		void Initialize() {
			all_arcs_.reserve(num_deadheading_arcs_);
			distance_.resize(num_deadheading_arcs_, std::vector <double> (num_deadheading_arcs_, kDoubleMax));
			distance_req_.resize(2 * m_, std::vector <ShortestPathReq> (2 * m_));
			helper_.resize(num_deadheading_arcs_, std::vector <size_t> (num_deadheading_arcs_, kNIL));
			/* helper_edge_.resize(num_deadheading_arcs_, std::vector <ArcData>(num_deadheading_arcs_)); */
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
		APSP_Turns(std::shared_ptr <const Graph> g, std::shared_ptr <const EdgeCost_CircularTurns> &cost_fn) : g_{g} {
			cost_fn_ = cost_fn;
			n_ = g_->GetN();
			m_ = g_->GetM();
			m_nr_ = g_->GetMnr();
			num_deadheading_arcs_ = 2 * m_ + 2 * m_nr_;
			Initialize();
			GenerateAllDeadheadingArcs();
			GenerateAdjacency();
			GetDepots();
		}

		bool APSP_Deadheading() {

			for(size_t i = 0; i < num_deadheading_arcs_; ++i) { // set cost to self to 0
				distance_[i][i] = 0;
			}

			for(size_t i = 0; i < num_deadheading_arcs_; ++i) { // Get distances for adjacent edges
				const auto &arc_i = all_arcs_[i];
				for(const auto j:arc_i.outgoing_arcs_) {
					const auto &arc_j = all_arcs_[j];
					double turn_cost = kDoubleMax;
					if(cost_fn_->ComputeTurnCost(arc_i.edge_, arc_j.edge_, false, false, arc_i.rev_, arc_j.rev_, turn_cost)) {
						return kFail;
					}
					double cost = turn_cost + arc_j.deadhead_cost_;
					if(cost < distance_[i][j]) {
						distance_[i][j] = cost;
					}
				}
			}

			for(size_t k = 0; k < num_deadheading_arcs_; ++k) { // n^3 iterations Floyd-Warshall
				for(size_t i = 0; i < num_deadheading_arcs_; ++i) {
					for(size_t j = 0; j < num_deadheading_arcs_; ++j) {
						if(distance_[i][k] + distance_[k][j] < distance_[i][j]) {
							distance_[i][j] = distance_[i][k] + distance_[k][j];
							helper_[i][j] = k;
						}
					}
				}
			}

			for(size_t i = 0; i < m_; ++i) { // Add opposite arcs to outgoing
				all_arcs_[i].outgoing_arcs_.push_back(m_ + i);
				all_arcs_[m_ + i].outgoing_arcs_.push_back(i);
			}

			// Compute cost of going from a required arc to an adjacent deadheading arc
			// The cost includes the turning cost and the deadheading cost of the adjacent arc
			for(size_t i = 0; i < 2 * m_; ++i) {
				ArcData& req_arc_i = all_arcs_[i];
				req_arc_i.out_req_cost_.clear();
				req_arc_i.out_req_cost_.reserve(req_arc_i.outgoing_arcs_.size());
				for(const auto &out_arc_j:req_arc_i.outgoing_arcs_) {
					const auto &out_arc = all_arcs_[out_arc_j];
					double turn_cost;
					if(cost_fn_->ComputeTurnCost(req_arc_i.edge_, out_arc.edge_, true, false, req_arc_i.rev_, out_arc.rev_, turn_cost)) {
						return kFail;
					}
					double out_cost = out_arc.deadhead_cost_;
					req_arc_i.out_req_cost_.push_back(turn_cost + out_cost);
				}
			}

			// Compute cost of going into a required arc from an adjacent deadheading arc
			// The cost includes only the turning cost
			for(size_t i = 0; i < 2 * m_; ++i) {
				ArcData& req_arc = all_arcs_[i];
				req_arc.in_req_turn_cost_.reserve(req_arc.incoming_arcs_.size());
				for(const auto &in_arc:req_arc.incoming_arcs_) {
					double turn_cost;
					if(cost_fn_->ComputeTurnCost(all_arcs_[in_arc].edge_, req_arc.edge_, false, true, all_arcs_[in_arc].rev_, req_arc.rev_, turn_cost)) {
						return kFail;
					}
					req_arc.in_req_turn_cost_.push_back(turn_cost);
				}
			}

			for(size_t i = 0; i < 2 * m_; ++i) { // Compute shortest path from a required edge to another
				const ArcData& arc_i = all_arcs_[i];
				for(size_t j = 0; j < 2 * m_; ++j) {
					if(i == j) { // cost to itself is 0
						distance_req_[i][j].cost_ = 0;
						distance_req_[i][j].status_ = -1;
						continue;
					}

					const ArcData& arc_j = all_arcs_[j];
					if(arc_i.h_ID_ == arc_j.t_ID_) { // If the required arcs are connected then get direct cost
						double turn_cost;
						if(cost_fn_->ComputeTurnCost(arc_i.edge_, arc_j.edge_, true, true, arc_i.rev_, arc_j.rev_, turn_cost)) {
							return kFail;
						}
						distance_req_[i][j].cost_ = turn_cost;
						distance_req_[i][j].status_ = 0;
					}

					for(size_t out_iter = 0; out_iter < arc_i.outgoing_arcs_.size(); ++out_iter) { // check for each outgoing arc from i
						size_t out_i = arc_i.outgoing_arcs_[out_iter];
						double out_cost = arc_i.out_req_cost_[out_iter];

						for(size_t in_iter = 0; in_iter < arc_j.incoming_arcs_.size(); ++in_iter) { // check for each incoming arc to j
							size_t in_j = arc_j.incoming_arcs_[in_iter];
							double turn_cost = arc_j.in_req_turn_cost_[in_iter];
							double cost = out_cost + distance_[out_i][in_j] + turn_cost;
							if(cost < distance_req_[i][j].cost_) {
								distance_req_[i][j].cost_ = cost;
								distance_req_[i][j].status_ = 2;
								distance_req_[i][j].start_deadheading_ = out_i;
								distance_req_[i][j].end_deadheading_ = in_j;
							}
						}
					}
				}
			}

			for(auto &depot:depots_) {
				for(size_t i = 0; i < 2 * m_; ++i) {
					const auto &arc_i = all_arcs_[i];
					if(arc_i.t_ID_ == depot.ID_) { // Depot lies on tail, cost from depot is 0
						depot.distance_depot_to_req_[i].cost_ = 0;
						depot.distance_depot_to_req_[i].status_ = 0;
						continue;
					}

					for(const auto &out_depot_index:depot.outgoing_arcs_) { // Compute cost from depot to req arc
						const auto &out_depot_arc = all_arcs_[out_depot_index];
						double out_cost = out_depot_arc.deadhead_cost_;

						for(size_t in_iter = 0; in_iter < arc_i.incoming_arcs_.size(); ++in_iter) {
							size_t in_req = arc_i.incoming_arcs_[in_iter];
							double turn_cost = arc_i.in_req_turn_cost_[in_iter];
							double cost = out_cost + distance_[out_depot_index][in_req] + turn_cost;
							if(cost < depot.distance_depot_to_req_[i].cost_) {
								depot.distance_depot_to_req_[i].cost_ = cost;
								depot.distance_depot_to_req_[i].status_ = 2;
								depot.distance_depot_to_req_[i].start_deadheading_ = out_depot_index;
								depot.distance_depot_to_req_[i].end_deadheading_ = in_req;
							}
						}
					}
				}

				for(size_t i = 0; i < 2 * m_; ++i) {
					const auto &arc_i = all_arcs_[i];
					if(arc_i.h_ID_ == depot.ID_) { // Depot lies on head, cost to depot is 0
						depot.distance_req_to_depot_[i].cost_ = 0;
						depot.distance_req_to_depot_[i].status_ = 0;
						continue;
					}

					for(size_t out_iter = 0; out_iter < arc_i.outgoing_arcs_.size(); ++out_iter) { // compute cost from req arc to depot
						size_t out_i = arc_i.outgoing_arcs_[out_iter];
						double out_cost = arc_i.out_req_cost_[out_iter];

						for(const auto &in_depot:depot.incoming_arcs_) {
							double cost = out_cost + distance_[out_i][in_depot];
							if(cost < depot.distance_req_to_depot_[i].cost_) {
								depot.distance_req_to_depot_[i].cost_ = cost;
								depot.distance_req_to_depot_[i].status_ = 2;
								depot.distance_req_to_depot_[i].start_deadheading_ = out_i;
								depot.distance_req_to_depot_[i].end_deadheading_ = in_depot;
							}
						}
					}

				}
			}
			return kSuccess;
		}

		double GetDepotToRequiredCost(size_t depot_ID, size_t req_index, bool reverse) const {
			size_t arc_index = EdgeToIndex(req_index, kIsRequired, reverse);
			size_t depot_index;
			auto depot_map_find = depot_map_.find(depot_ID);
			if (depot_map_find != depot_map_.end()) {
				depot_index = depot_map_find->second;
			} else {
				std::cerr << "Depot not found " << depot_ID << "\n";
				return kDoubleMax;
			}
			return depots_[depot_index].distance_depot_to_req_[arc_index].cost_;
		}

		double GetRequiredToDepotCost(size_t depot_ID, size_t req_index, bool reverse) const {
			size_t arc_index = EdgeToIndex(req_index, kIsRequired, reverse);
			size_t depot_index;
			auto depot_map_find = depot_map_.find(depot_ID);
			if (depot_map_find != depot_map_.end()) {
				depot_index = depot_map_find->second;
			} else {
				std::cerr << "Depot not found " << depot_ID << "\n";
				return kDoubleMax;
			}
			return depots_[depot_index].distance_req_to_depot_[arc_index].cost_;
		}

		double GetRequiredToRequiredCost(size_t req1_index, size_t req2_index, bool rev1, bool rev2) const {
			size_t arc1_index = EdgeToIndex(req1_index, kIsRequired, rev1);
			size_t arc2_index = EdgeToIndex(req2_index, kIsRequired, rev2);
			return distance_req_[arc1_index][arc2_index].cost_;
		}

		int GetDepotToRequiredPath(const size_t depot_ID, const size_t req_index, const bool reverse, std::vector <GraphEdge> &path) const {
			size_t arc_index = EdgeToIndex(req_index, kIsRequired, reverse);
			size_t depot_index;
			auto depot_map_find = depot_map_.find(depot_ID);
			if (depot_map_find != depot_map_.end()) {
				depot_index = depot_map_find->second;
			} else {
				std::cerr << "Depot not found " << depot_ID << "\n";
				return kFail;
			}
			const auto &path_def = depots_[depot_index].distance_depot_to_req_[arc_index];
			if(path_def.status_ == 0) {
				return kSuccess;
			}
			const auto &start_arc = all_arcs_[path_def.start_deadheading_];
			if(path_def.start_deadheading_ == path_def.end_deadheading_) {
				path.push_back(GraphEdge(start_arc.g_index_, start_arc.req_, start_arc.rev_, false));
				return kSuccess;
			}
			GetDeadheadingPath(path_def.start_deadheading_, path_def.end_deadheading_, path);
			return kSuccess;
		}

		int GetRequiredToDepotPath(const size_t depot_ID, const size_t req_index, const bool reverse, std::vector <GraphEdge> &path) const {
			size_t arc_index = EdgeToIndex(req_index, kIsRequired, reverse);
			size_t depot_index;
			auto depot_map_find = depot_map_.find(depot_ID);
			if (depot_map_find != depot_map_.end()) {
				depot_index = depot_map_find->second;
			} else {
				std::cerr << "Depot not found " << depot_ID << "\n";
				return kFail;
			}
			const auto &path_def = depots_[depot_index].distance_req_to_depot_[arc_index];
			if(path_def.status_ == 0) {
				return kSuccess;
			}
			const auto &start_arc = all_arcs_[path_def.start_deadheading_];
			if(path_def.start_deadheading_ == path_def.end_deadheading_) {
				path.push_back(GraphEdge(start_arc.g_index_, start_arc.req_, start_arc.rev_, false));
				return kSuccess;
			}
			GetDeadheadingPath(path_def.start_deadheading_, path_def.end_deadheading_, path);
			return kSuccess;
		}

		void GetRequiredToRequiredPath(size_t req1_index, size_t req2_index, bool rev1, bool rev2, std::vector <GraphEdge> &path) const {
			size_t arc1_index = EdgeToIndex(req1_index, kIsRequired, rev1);
			size_t arc2_index = EdgeToIndex(req2_index, kIsRequired, rev2);
			const auto &path_def = distance_req_[arc1_index][arc2_index];
			if(path_def.status_ == -1) {
				std::cerr << "negative status\n";
				return;
			}
			if(path_def.status_ == 0) {
				return;
			}
			GetDeadheadingPath(path_def.start_deadheading_, path_def.end_deadheading_, path);
		}

		void GetDeadheadingPathAux(const size_t i, const size_t j, std::vector <GraphEdge> &path) const {
			if(helper_[i][j] == kNIL) {
				path.push_back(GraphEdge(all_arcs_[i].g_index_, all_arcs_[i].req_, all_arcs_[i].rev_, false));
			} else {
				GetDeadheadingPathAux(i, helper_[i][j], path);
				GetDeadheadingPathAux(helper_[i][j], j, path);
			}
		}

		void GetDeadheadingPath(const size_t i, const size_t j, std::vector <GraphEdge> &path) const {
			if(i == j) {
				path.push_back(GraphEdge(all_arcs_[i].g_index_, all_arcs_[i].req_, all_arcs_[i].rev_, false));
				return;
			}
			if(helper_[i][j] == kNIL) {
				path.push_back(GraphEdge(all_arcs_[i].g_index_, all_arcs_[i].req_, all_arcs_[i].rev_, false));
				path.push_back(GraphEdge(all_arcs_[j].g_index_, all_arcs_[j].req_, all_arcs_[j].rev_, false));
			} else {
				GetDeadheadingPathAux(i, j, path);
				path.push_back(GraphEdge(all_arcs_[j].g_index_, all_arcs_[j].req_, all_arcs_[j].rev_, false));
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

	};

} // namespace lclibrary

#endif /* LCLIBRARY_ALGORITHMS_APSP_TURNS_H_ */

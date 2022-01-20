/**
 * This file is part of the LineCoverage-library.
 * The file contains description of the class Route with turns
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

/** TODO: Work in progress
 */

#ifndef LCLIBRARY_CORE_ROUTE_TURNS_H_
#define LCLIBRARY_CORE_ROUTE_TURNS_H_

#include <lclibrary/core/typedefs.h>
#include <lclibrary/core/math_utils.h>
#include <lclibrary/core/edge.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/algorithms/apsp_turns.h>
#include <lclibrary/core/edge_cost_base.h>
#include <fstream>
#include <memory>
#include <algorithm>

namespace lclibrary {

	struct KinematicEdge {
		Vec2d start;
		Vec2d end;
		bool serv = true;
		enum Type {kStraight, kAccelerate, kDecelerate, kCicular};
		Type type = kStraight;
		double start_vel = 0, end_vel = 0, angular_vel = 0, acc = 0;
		double orientation, time;
		double cost;

		KinematicEdge() {}
		KinematicEdge(const Vec2d &start_in, const Vec2d &end_in, const bool serv_in, const Type type_in) : start(start_in), end(end_in), serv(serv_in), type(type_in) {}

		void SetVel(const double s_vel, const double e_vel, const double ang_vel) {start_vel = s_vel; end_vel = e_vel; angular_vel = ang_vel;}

	};

	class RouteTurns {
		size_t m_;
		size_t depot_ID_;
		GraphEdgeList req_edges_seq_;
		GraphEdgeList edges_seq_;
		std::vector <KinematicEdge> kinematic_edges_;
		std::shared_ptr <const Graph> g_;
		std::shared_ptr <const APSP_Turns> apsp_;
		std::shared_ptr <const EdgeCost_CircularTurns> cost_fn_;
		double cost_;

		public:

		RouteTurns(size_t depot, GraphEdgeList &req_edges, std::shared_ptr <const Graph> g, std::shared_ptr <const APSP_Turns> apsp) : depot_ID_(depot), req_edges_seq_(req_edges), g_(g), apsp_(apsp) {
			m_ = req_edges_seq_.size();
			cost_fn_ = g_->GetTurnsCostFunction();
		}

		void FormKinematicRoute() {
			if(m_ == 0) {
				return;
			}
			edges_seq_.reserve(m_);
			cost_= 0;
			auto start_edge = req_edges_seq_.front();
			apsp_->GetDepotToRequiredPath(depot_ID_, start_edge.edge_index_, start_edge.rev_, edges_seq_);
			edges_seq_.push_back(start_edge);
			cost_ += apsp_->GetDepotToRequiredCost(depot_ID_, start_edge.edge_index_, start_edge.rev_);
			cost_ += g_->GetCost(start_edge);
			/* if(start_edge.rev_ == false) { */
			/* 	cost_ += g_->GetServiceCost(start_edge.edge_index_); */
			/* } else { */
			/* 	cost_ += g_->GetReverseServiceCost(start_edge.edge_index_); */
			/* } */
			bool start_flag = false;
			for(auto &e:req_edges_seq_) {
				if(start_flag == false) {
					start_flag = true;
					continue;
				}
				cost_ += apsp_->GetRequiredToRequiredCost(start_edge.edge_index_, e.edge_index_, start_edge.rev_, e.rev_);
				apsp_->GetRequiredToRequiredPath(start_edge.edge_index_, e.edge_index_, start_edge.rev_, e.rev_, edges_seq_);
				edges_seq_.push_back(e);
				start_edge = e;
				cost_ += g_->GetCost(start_edge);
				/* if(start_edge.rev_ == false) { */
				/* 	cost_ += g_->GetServiceCost(start_edge.edge_index_); */
				/* } else { */
				/* 	cost_ += g_->GetReverseServiceCost(start_edge.edge_index_); */
				/* } */
			}
			auto const &end_edge = req_edges_seq_.back();
			apsp_->GetRequiredToDepotPath(depot_ID_, end_edge.edge_index_, end_edge.rev_, edges_seq_);
			cost_ += apsp_->GetRequiredToDepotCost(depot_ID_, end_edge.edge_index_, end_edge.rev_);

			/* Form kinematic edges */
			kinematic_edges_.reserve(2 * m_);

			CircularTurn circ_turn;
			/* Depot to first edge */
			auto prev_edge = edges_seq_[0];
			Vec2d t_xy, h_xy;
			g_->GetVertexCoordinateofEdge(prev_edge, t_xy, h_xy);
			KinematicEdge prev_kin_edge(t_xy, h_xy, prev_edge.serv_, KinematicEdge::kStraight);
			prev_kin_edge.time = prev_kin_edge.cost = g_->GetCost(prev_edge);
			prev_kin_edge.start_vel = prev_kin_edge.end_vel =  cost_fn_->GetSpeed(prev_edge.serv_);
			for(size_t i = 1; i < edges_seq_.size(); ++i) {
				auto next_edge = edges_seq_[i];
				double cost;
				ComputeTurnCost(prev_edge, next_edge, cost, circ_turn);

				size_t prev_t, prev_h, next_t, next_h;
				g_->GetVerticesIDOfEdge(prev_edge, prev_t, prev_h);
				g_->GetVerticesIDOfEdge(next_edge, next_t, next_h);
				if(circ_turn.status == -1 or (prev_h != next_t )) {
					std::cerr << "ComputeTurnCost returned invalid output " << i << "\n";
					Vec2d te_xy, he_xy; size_t te, he;
					g_->GetVertexCoordinateofEdge(prev_edge, te_xy, he_xy);
					g_->GetVerticesIDOfEdge(prev_edge, te, he);
					std::cerr << te << " " << he << std::endl;
					std::cerr << te_xy << " " << he_xy << std::endl;
					g_->GetVertexCoordinateofEdge(next_edge, te_xy, he_xy);
					g_->GetVerticesIDOfEdge(next_edge, te, he);
					std::cerr << te << " " << he << std::endl;
					std::cerr << te_xy << " " << he_xy << std::endl;
					return;
				}
				if(circ_turn.status == 0) {
					g_->GetVertexCoordinateofEdge(next_edge, t_xy, h_xy);
					KinematicEdge next_kin_edge(t_xy, h_xy, next_edge.serv_, KinematicEdge::kStraight);
					next_kin_edge.time = next_kin_edge.cost = g_->GetCost(next_edge);
					next_kin_edge.start_vel = next_kin_edge.end_vel =  cost_fn_->GetSpeed(next_edge.serv_);
					kinematic_edges_.push_back(prev_kin_edge);
					prev_edge = next_edge;
					prev_kin_edge = next_kin_edge;
					continue;
				}

				if(prev_kin_edge.start_vel > circ_turn.vel) {
					prev_kin_edge.end = circ_turn.e1_acceleration_start_point;
					KinematicEdge dec_edge(prev_kin_edge.end, circ_turn.e1_arc_start_point, prev_kin_edge.serv, KinematicEdge::kDecelerate);
					dec_edge.SetVel(prev_kin_edge.start_vel, circ_turn.vel, 0);
					dec_edge.acc = circ_turn.acc;
					dec_edge.time = (prev_kin_edge.start_vel - circ_turn.vel) / dec_edge.acc;
					kinematic_edges_.push_back(prev_kin_edge);
					kinematic_edges_.push_back(dec_edge);
				} else {
					prev_kin_edge.end = circ_turn.e1_arc_start_point;
					kinematic_edges_.push_back(prev_kin_edge);
				}

				KinematicEdge arc(circ_turn.e1_arc_start_point, circ_turn.e2_arc_end_point, (prev_kin_edge.serv or next_edge.serv_), KinematicEdge::kCicular);
				arc.SetVel(circ_turn.vel, circ_turn.vel, circ_turn.angular_vel);
				arc.cost = cost;
				arc.time = std::abs(circ_turn.arc_ang1 - circ_turn.arc_ang2) / circ_turn.angular_vel;
				kinematic_edges_.push_back(arc);

				g_->GetVertexCoordinateofEdge(next_edge, t_xy, h_xy);
				KinematicEdge next_kin_edge(t_xy, h_xy, next_edge.serv_, KinematicEdge::kStraight);
				next_kin_edge.time = next_kin_edge.cost = g_->GetCost(next_edge);
				next_kin_edge.start_vel = next_kin_edge.end_vel =  cost_fn_->GetSpeed(next_edge.serv_);

				if(next_kin_edge.end_vel == circ_turn.vel) {
					next_kin_edge.start = circ_turn.e2_arc_end_point;
				} else {
					KinematicEdge acc_edge;
					if(next_kin_edge.end_vel < circ_turn.vel) {
						acc_edge = KinematicEdge(circ_turn.e2_arc_end_point, circ_turn.e2_acceleration_end_point, next_kin_edge.serv, KinematicEdge::kDecelerate);
						acc_edge.time = (circ_turn.vel - next_kin_edge.end_vel) / circ_turn.acc;
					} else {
						acc_edge = KinematicEdge(circ_turn.e2_arc_end_point, circ_turn.e2_acceleration_end_point, next_kin_edge.serv, KinematicEdge::kAccelerate);
						acc_edge.time = (next_kin_edge.end_vel - circ_turn.vel) / circ_turn.acc;
					}
					acc_edge.SetVel(circ_turn.vel, next_kin_edge.end_vel, 0);
					acc_edge.acc = circ_turn.acc;
					kinematic_edges_.push_back(acc_edge);
					next_kin_edge.start = circ_turn.e2_acceleration_end_point;
				}
				prev_edge = next_edge;
				prev_kin_edge = next_kin_edge;
			}
			kinematic_edges_.push_back(prev_kin_edge);
			double total_cost = 0;
			for(auto const &ke:kinematic_edges_) {
				total_cost += ke.cost;
			}
			std::cout << "KE cost: " << total_cost << std::endl;
		}

		void ComputeTurnCost(const GraphEdge &prev_edge, const GraphEdge &next_edge, double &cost, CircularTurn &circ_turn) {
				cost_fn_->ComputeTurnCost(g_->GetEdge(prev_edge.edge_index_, prev_edge.req_), g_->GetEdge(next_edge.edge_index_, next_edge.req_), prev_edge.serv_, next_edge.serv_, prev_edge.rev_, next_edge.rev_, cost, circ_turn);
		}

		bool CheckRoute() const {
			return kSuccess;
		}

		void PrintRoute() const {
			std::cout << "Cost: " << cost_ << std::endl;
			std::cout << "R depot: " << depot_ID_ << std::endl;
		}

		size_t GetRouteLength() const {
			return req_edges_seq_.size();
		}

		void WriteRoute(const std::string &filename) {
			std::ofstream fobj(filename);
			for(auto const &e:kinematic_edges_) {
				fobj << e.type << " " << e.start.x << " " << e.start.y << " " << e.end.x << " " << e.end.y << " " << e.start_vel << " " << e.end_vel << " " << e.angular_vel << " " << e.time << " " << e.serv << std::endl;
			}
			fobj.close();
		}

	};
} // namespace lclibrary

#endif /* LCLIBRARY_CORE_ROUTE_TURNS_H_ */

/**
 * This file is part of the LineCoverage-library.
 * The file contains class for travel time as Edge cost and circular arcs for turns
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

#ifndef LCLIBRARY_UTILS_EDGE_COST_WITH_CIRCULAR_TURNS_H_
#define LCLIBRARY_UTILS_EDGE_COST_WITH_CIRCULAR_TURNS_H_

#include <lclibrary/core/edge_cost_base.h>
#include <fstream>

namespace lclibrary {

	struct CircularTurn {
		Vec2d e1_acceleration_start_point, e1_arc_start_point;
		Vec2d e2_acceleration_end_point, e2_arc_end_point;
		Vec2d center_circular_arc;
		double vel;
		double angular_vel;
		double radius;
		double arc_ang1, arc_ang2;
		double dist_from_pivot;
		double acc;
		int status;
	};

	inline std::ostream& operator << (std::ostream &ostream_obj, const CircularTurn &circ_turn) {
		ostream_obj << circ_turn.e1_acceleration_start_point << " " << circ_turn.e1_arc_start_point << std::endl;
		ostream_obj << circ_turn.e2_arc_end_point << " " << circ_turn.e2_acceleration_end_point << std::endl;
		ostream_obj << circ_turn.vel << " " << circ_turn.angular_vel << std::endl;
		ostream_obj << circ_turn.arc_ang1 << " " << circ_turn.arc_ang2 << " " << circ_turn.status << std::endl;
		return ostream_obj;
	}

	class EdgeCost_CircularTurns : public EdgeCost {

		double service_speed_;
		double deadheading_speed_;
		double wind_speed_;
		double wind_direction_;
		double angular_velocity_;
		double acc_;
		double delta_;

		public:

		EdgeCost_CircularTurns(const double s, const double d, const double w_s, const double w_d, const double ang_v, const double acc, const double delta) : service_speed_{s}, deadheading_speed_{d}, wind_speed_{w_s}, wind_direction_{w_d}, angular_velocity_{ang_v}, acc_{acc}, delta_{delta} {}

		inline double GetSpeed(const bool serv) const {
			if(serv) {
				return service_speed_;
			} else {
				return deadheading_speed_;
			}
		}

		bool VerifyEdgePair(const Edge* e1, const Edge* e2, const bool serv1, const bool serv2, const bool rev1, const bool rev2) {
			Vec2d e1_t, e1_h;
			Vec2d e2_t, e2_h;
			/* std::cout << "Inside ComputeTurnCost\n"; */
			e1->GetTailVertexXY(e1_t); e1->GetHeadVertexXY(e1_h);
			e2->GetTailVertexXY(e2_t); e2->GetHeadVertexXY(e2_h);
			if(rev1) {
				std::swap(e1_t, e1_h);
			}
			if(rev2) {
				std::swap(e2_t, e2_h);
			}
			double init_vel1 = service_speed_;
			double init_vel2 = service_speed_;
			if(!serv1) {
				init_vel1 = deadheading_speed_;
			}
			if(!serv2) {
				init_vel2 = deadheading_speed_;
			}

			if(std::abs(e1_h.x - e2_t.x) > kEps or std::abs(e1_h.y - e2_t.y) > kEps) {
				return kFail;
			}

			double norm21 = (e1_h - e1_t).Norm();
			double norm32 = (e2_h - e2_t).Norm();
			if(norm21 < kEps or norm32 < kEps) {
				return kFail;
			}

			double max_l1 = norm21/2.0;
			double max_l2 = norm32/2.0;
			if(max_l1 < (init_vel1 * init_vel1) / (2.0 * acc_)) {
				return kFail;
			}
			if(max_l2 < (init_vel2 * init_vel2) / (2.0 * acc_)) {
				return kFail;
			}

		}

		int ComputeServiceCost(const Edge &e, double &cost, double &cost_rev) const {
			const Vertex *t, *h;
			e.GetVertices(t, h);
			if(t == nullptr or h == nullptr)
				return 1;
			auto t_xy = t->GetXY();
			auto h_xy = h->GetXY();
			cost = ComputeTravelTime(t_xy, h_xy, service_speed_);
			cost_rev = ComputeTravelTime(h_xy, t_xy, service_speed_);
			return 0;

		}

		int ComputeDeadheadCost(const Edge &e, double &cost, double &cost_rev) const {
			const Vertex *t, *h;
			e.GetVertices(t, h);
			if(t == nullptr or h == nullptr)
				return 1;
			auto t_xy = t->GetXY();
			auto h_xy = h->GetXY();
			cost = ComputeTravelTime(t_xy, h_xy, deadheading_speed_);
			cost_rev = ComputeTravelTime(h_xy, t_xy, deadheading_speed_);
			return 0;

		}

		double ComputeTravelTime(const Vec2d t_xy, const Vec2d h_xy, const double vel) const {
			double d, tht;
			t_xy.DistTht(h_xy, d, tht);
			if (std::abs(d) < 1e-10)
				return 0;
			Vec2d wind_vec (wind_speed_ * cos(wind_direction_), wind_speed_ * sin(wind_direction_));
			auto travel_vec = t_xy.TravelVec(h_xy);
			double cos_phi = 0;
			wind_vec.CosAngle(travel_vec, cos_phi);
			double bBy2 = -wind_speed_ * cos_phi;
			double c = wind_speed_ * wind_speed_ - vel * vel;
			double g = -bBy2 + std::sqrt(bBy2 * bBy2 - c);
			return d/g;
		}

		int ComputeTurnCost(const Edge* e1, const Edge* e2, const bool serv1, bool serv2, const bool rev1, const bool rev2, double &cost) const {
			CircularTurn circ_turn;
			return ComputeTurnCost(e1, e2, serv1, serv2, rev1, rev2, cost, circ_turn);
		}

		int ComputeTurnCost(const Edge* e1, const Edge* e2, const bool serv1, bool serv2, const bool rev1, const bool rev2, double &cost, CircularTurn &circ_turn) const {
			Vec2d e1_t, e1_h;
			Vec2d e2_t, e2_h;
			/* std::cout << "Inside ComputeTurnCost\n"; */
			e1->GetTailVertexXY(e1_t); e1->GetHeadVertexXY(e1_h);
			e2->GetTailVertexXY(e2_t); e2->GetHeadVertexXY(e2_h);
			if(rev1) {
				std::swap(e1_t, e1_h);
			}
			if(rev2) {
				std::swap(e2_t, e2_h);
			}
			double init_vel1 = service_speed_;
			double init_vel2 = service_speed_;
			if(!serv1) {
				init_vel1 = deadheading_speed_;
			}
			if(!serv2) {
				init_vel2 = deadheading_speed_;
			}
			double vel = std::min(init_vel1, init_vel2);

			if(std::abs(e1_h.x - e2_t.x) > kEps or std::abs(e1_h.y - e2_t.y) > kEps) {
				cost = kDoubleNaN;
				circ_turn.status = -1;
				return kFail;
			}

			double norm21 = (e1_h - e1_t).Norm();
			double norm32 = (e2_h - e2_t).Norm();
			if(norm21 < kEps or norm32 < kEps) {
				cost = kDoubleNaN;
				circ_turn.status = -1;
				return kFail;
			}

			auto vec21 = (e2_t - e1_t)/norm21;
			auto vec32 = (e2_h - e2_t)/norm32;

			auto vec_bisector = (-vec21 + vec32);
			vec_bisector.Normalize();
			double sin_theta = std::abs(vec21.x * vec_bisector.y - vec21.y * vec_bisector.x);
			double cos_theta = std::abs(vec21.Dot(vec_bisector));
			if(IsNearZero(sin_theta)) {
				cost = 0;
				circ_turn.status = 0;
				return kSuccess;
			}

			if(IsNearZero(sin_theta - 1)) {
				cost = 0;
				circ_turn.status = 0;
				return kSuccess;
			}

			double rad_optimal = vel/angular_velocity_;
			double lambda = rad_optimal/sin_theta;

			/* std::cout << "Case 1: " << vel << " " << lambda << " " << lambda * cos_theta << std::endl; */
			circ_turn.status = 1;

			if(lambda - rad_optimal > delta_) {
				lambda = delta_/(1-sin_theta);
				vel = angular_velocity_ * lambda * sin_theta;
				circ_turn.status = 2;
				/* std::cout << "Case 2: " << vel << " " << lambda << " " << lambda * cos_theta << std::endl; */
			}
			double max_l1 = norm21/2.0;
			double max_l2 = norm32/2.0;
			double l_lambda = lambda * cos_theta;
			double cot_omega = cos_theta / (sin_theta * angular_velocity_);
			if(l_lambda + (init_vel1 * init_vel1 - vel * vel) / (2 * acc_) > max_l1) {
				double det = cos_theta * cos_theta / (sin_theta * sin_theta * angular_velocity_ * angular_velocity_) + init_vel1 * init_vel1 / (acc_ * acc_) - 2.0 * max_l1 / acc_;
				if(det >= 0) {
					double sqrt_det = std::sqrt(det);
					double vl1 = acc_ * (cot_omega + sqrt_det);
					double vl2 = acc_ * (cot_omega - sqrt_det);
					if(vl1 > vel) {
						vel = std::min(std::max(0.0, vl2), vel);
						rad_optimal = vel / angular_velocity_;
						lambda = rad_optimal / sin_theta;
						l_lambda = lambda * cos_theta;
						circ_turn.status = 3;
						/* std::cout << "Case 3: " << vel << " " << lambda << " " << lambda * cos_theta << std::endl; */
					}
				}
			}
			if(l_lambda + (init_vel2 * init_vel2 - vel * vel) / (2 * acc_) > max_l2) {
				double det = cos_theta * cos_theta / (sin_theta * sin_theta * angular_velocity_ * angular_velocity_) + init_vel2 * init_vel2 / (acc_ * acc_) - 2.0 * max_l2 / acc_;
				if(det >= 0) {
					double sqrt_det = std::sqrt(det);
					double vl1 = acc_ * (cot_omega + sqrt_det);
					double vl2 = acc_ * (cot_omega - sqrt_det);
					if(vl1 > vel) {
						vel = std::min(std::max(0.0, vl2), vel);
						rad_optimal = vel / angular_velocity_;
						lambda = rad_optimal / sin_theta;
						circ_turn.status = 4;
						/* std::cout << "Case 4: " << vel << " " << lambda << " " << lambda * cos_theta << std::endl; */
					}
				}
			}

			rad_optimal = vel / angular_velocity_;
			lambda = rad_optimal / sin_theta;

			auto p1_vec = vec21 * vec_bisector.Dot(vec21) - vec_bisector;
			auto p2_vec = vec32 * vec_bisector.Dot(vec32) - vec_bisector;
			double arc_ang1 = atan2(p1_vec.y, p1_vec.x);
			double arc_ang2 = atan2(p2_vec.y, p2_vec.x);
			if(std::abs(arc_ang1 - arc_ang2) > M_PI) {
				if(std::abs(arc_ang1) > std::abs(arc_ang2)) {
					if(arc_ang1 < 0) {
						arc_ang1 = arc_ang1 + 2.0 * M_PI;
					} else {
						arc_ang1 = arc_ang1 - 2.0 * M_PI;
					}
				} else {
					if(arc_ang2 < 0) {
						arc_ang2 = arc_ang2 + 2.0 * M_PI;
					} else {
						arc_ang2 = arc_ang2 - 2.0 * M_PI;
					}
				}
			}
			auto p1 = e1_h - vec21 * lambda * cos_theta;
			auto p2 = e2_t + vec32 * lambda * cos_theta;
			cost = std::abs(arc_ang1 - arc_ang2)/angular_velocity_;
			/* std::cout << cost << " " << std::abs(arc_ang1 - arc_ang2) << " " << angular_velocity_ << std::endl; */
			if(init_vel1 > vel) {
				cost += (init_vel1 - vel) / acc_;
			} else {
				cost += (vel - init_vel1) / acc_;
			}
			if(init_vel2 > vel) {
				cost += (init_vel2 - vel) / acc_;
			} else {
				cost += (vel - init_vel2) / acc_;
			}
			/* std::cout << cost << std::endl; */

			/* std::cout << arc_ang1 << " " << arc_ang2 << " " << vel << " " << cost << std::endl; */
			/* std::cout << p1.x << " " << p1.y << " " << vel << " " << cost << std::endl; */
			/* std::cout << p2.x << " " << p2.y << " " << vel << " " << cost << std::endl; */

			if(init_vel1 > vel) {
				circ_turn.e1_acceleration_start_point = p1 - vec21 * (init_vel1 * init_vel1 - vel * vel) / (2.0 * acc_);
			} else {
				circ_turn.e1_acceleration_start_point = p1 - vec21 * (vel * vel - init_vel1 * init_vel1) / (2.0 * acc_);
			}
			circ_turn.e1_arc_start_point = p1;
			circ_turn.e2_arc_end_point = p2;
			if(init_vel2 > vel) {
				circ_turn.e2_acceleration_end_point = p2 + vec32 * (init_vel2 * init_vel2 - vel * vel) / (2.0 * acc_);
			} else {
				circ_turn.e2_acceleration_end_point = p2 + vec32 * (vel * vel - init_vel2 * init_vel2) / (2.0 * acc_);
			}
			circ_turn.vel = vel;
			circ_turn.angular_vel = angular_velocity_;
			circ_turn.acc = acc_;
			circ_turn.radius = rad_optimal;
			circ_turn.arc_ang1 = arc_ang1; circ_turn.arc_ang2 = arc_ang2;
			circ_turn.center_circular_arc = e2_t + vec_bisector * lambda;
			circ_turn.dist_from_pivot = lambda - rad_optimal;
			return kSuccess;
		}

	};

}

#endif /* LCLIBRARY_UTILS_EDGE_COST_WITH_CIRCULAR_TURNS_H_ */

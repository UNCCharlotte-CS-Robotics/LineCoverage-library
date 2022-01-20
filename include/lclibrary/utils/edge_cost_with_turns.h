/**
 * This file is part of the LineCoverage-library.
 * The file contains class for edge cost with turn costs
 * The travel time is computed as a cubic trajectory with the given velocity as the max velocity
 * Turn cost is computed as cubic trajectory with given angular velocity as the max angular velocity
 *
 * TODO: This is an incomplete implementation
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

#ifndef LCLIBRARY_UTILS_EDGE_TURNS_HPP_
#define LCLIBRARY_UTILS_EDGE_TURNS_HPP_

#include <lclibrary/core/edge_cost_base.h>

namespace lclibrary {
	class EdgeCost_Turns:public EdgeCost {

		double service_speed_;
		double deadheading_speed_;
		double asym_speed_;
		double max_velocity_;
		double max_angular_velocity_;

		public:

		EdgeCost_Turns(const double s, const double d, const double a_s, const double max_vel, const double max_omega) : service_speed_{s}, deadheading_speed_{d}, asym_speed_{a_s}, max_velocity_{max_vel}, max_angular_velocity_{max_omega} {}

		int ComputeServiceCost(const Edge &e, double &cost, double &cost_rev) {
			const Vertex *t, *h;
			e.GetVertices(t, h);
			if(t == nullptr or h == nullptr)
				return 1;
			auto t_xy = t->GetXY();
			auto h_xy = h->GetXY();
			double true_service_speed, true_reverse_service_speed;

			if(h_xy.x > t_xy.x or h_xy.y > t_xy.y) {
				true_service_speed = service_speed_ + asym_speed_;
				true_reverse_service_speed = service_speed_ - asym_speed_;
			}
			else {
				true_service_speed = service_speed_ - asym_speed_;
				true_reverse_service_speed = service_speed_ + asym_speed_;
			}

			if (e.IsSemiCircular()) {
				auto arc = e.GetCircularArc();
				double arc_length = arc.arc_length;
				cost =  3.*arc_length/true_service_speed;
				cost_rev =  3.*arc_length/true_reverse_service_speed;
				return 0;
			}
			cost = ComputeTravelTime(t_xy, h_xy, true_service_speed);
			cost_rev = ComputeTravelTime(h_xy, t_xy, true_reverse_service_speed);
			return 0;

		}

		int ComputeDeadheadCost(const Edge &e, double &cost, double &cost_rev) {
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

		double ComputeTravelTime(const Vec2d t_xy, const Vec2d h_xy, const double vel) {
			auto d = t_xy.Dist(h_xy);
			if (std::abs(d) < 1e-10)
				return 0;
			return 3.*d/vel;
		}

		int GetTurnCost(const Edge &e1, const Edge &e2, double &cost, bool rev1 = false, bool rev2 = false) {
			const Vertex *t, *h;
			e1.GetVertices(t, h);
			if(t == nullptr or h == nullptr)
				return 1;
			auto t1_xy = t->GetXY();
			auto h1_xy = h->GetXY();
			if(rev1)
				std::swap(t1_xy, h1_xy);
			auto d1 = t1_xy.Dist(h1_xy);
			e2.GetVertices(t, h);
			if(t == nullptr or h == nullptr)
				return 1;
			auto t2_xy = t->GetXY();
			auto h2_xy = h->GetXY();
			if(rev2)
				std::swap(t2_xy, h2_xy);
			auto d2 = t2_xy.Dist(h2_xy);
			if(!IsNearZero(h1_xy.x - t2_xy.x) and !IsNearZero(h1_xy.y - t2_xy.y))
				return 1;

			if(IsNearZero(d1) or IsNearZero(d2))
				return 1;

			auto vec1 = t1_xy.TravelVec(h1_xy);
			auto vec2 = t2_xy.TravelVec(h2_xy);
			if(e1.IsSemiCircular()) {
				auto arc = e1.GetCircularArc();
				auto vec1a = vec1;
				vec1a.x = 2 * (h1_xy.x - arc.center.x);
				vec1a.y = 2 * (h1_xy.y - arc.center.y);
				if(vec1a.Dot(vec1) < 0){
					vec1a.x = -vec1a.x; vec1a.y = -vec1a.y;
				}
				vec1 = vec1a;
			}
			if(e2.IsSemiCircular()) {
				auto arc = e2.GetCircularArc();
				auto vec2a = vec2;
				vec2a.x = 2 * (t2_xy.x - arc.center.x);
				vec2a.y = 2 * (t2_xy.y - arc.center.y);
				if(vec2a.Dot(vec2) < 0){
					vec2a.x = -vec2a.x; vec2a.y = -vec2a.y;
				}
				vec2 = vec2a;
			}
			double inner_product = vec1.x * vec2.x + vec1.y * vec2.y;
			double angle = std::acos(inner_product/(d1 * d2));
			return 3*angle/max_angular_velocity_;
		}

	};

}

#endif /* LCLIBRARY_UTILS_EDGE_TURNS_HPP_ */

/*!
 * @author Saurav Agarwal
 * @contact sagarw10@uncc.edu
 */

/**
 * This file is part of the LineCoverage-library.
 * The file contains class for travel time as Edge cost
 * Considers wind conditions to generate asymmetric costs
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

#ifndef LCLIBRARY_UTILS_EDGE_TRAVELTIME_H_
#define LCLIBRARY_UTILS_EDGE_TRAVELTIME_H_

#include <lclibrary/core/edge_cost_base.h>

namespace lclibrary {
	class EdgeCost_TravelTime:public EdgeCost {

		double service_speed_;
		double deadheading_speed_;
		double wind_speed_;
		double wind_direction_;

		public:

		EdgeCost_TravelTime(const double s, const double d, const double w_s, const double w_d) : service_speed_{s}, deadheading_speed_{d}, wind_speed_{w_s}, wind_direction_{w_d} {}

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

	};

}

#endif /* LCLIBRARY_UTILS_EDGE_TRAVELTIME_H_ */

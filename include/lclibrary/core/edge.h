/**
 * This file is part of the LineCoverage-library.
 * The file contains a class Edge for creating edges of a graph.
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

/**
 * TODO: Change to smart pointers
 *
 */

#ifndef LCLIBRARY_CORE_EDGE_H_
#define LCLIBRARY_CORE_EDGE_H_

#include <lclibrary/core/constants.h>
#include <lclibrary/core/vertex.h>
#include <lclibrary/core/vec2d.h>
#include <algorithm>
#include <tuple>

namespace lclibrary {

	typedef std::tuple <const Edge*, bool> EdgeTuple;
	inline EdgeTuple MakeEdgeTuple(const Edge *e, bool is_rev) {
		return std::make_tuple(e, is_rev);
	}

	class Edge {

		private:
			const Vertex *tail_vertex_ = nullptr; /*! tail Vertex */
			const Vertex *head_vertex_ = nullptr; /*! head Vertex */
			size_t tail_vertex_ID_ = kNIL, head_vertex_ID_ = kNIL;
			bool is_required_ = kIsRequired;
			double cost_ = 0;
			double service_cost_ = 0; /*! service cost of Edge */
			double service_cost_rev_ = 0; /*! reverse cost of Edge for asymmetric edge */
			double deadhead_cost_ = 0;
			double deadhead_cost_rev_ = 0;

			double demand_ = 0;
			double service_demand_ = 0;
			double service_demand_rev_ = 0;
			double deadhead_demand_ = 0;
			double deadhead_demand_rev_ = 0;

		public:

			/*! Constructors */
			Edge(size_t t, size_t h, bool req_i, double c_s, double c_srev, double c_d, double c_drev) : tail_vertex_ID_{t}, head_vertex_ID_{h}, is_required_{req_i}, service_cost_{c_s}, service_cost_rev_{c_srev}, deadhead_cost_{c_d}, deadhead_cost_rev_{c_drev} { }

			Edge(Vertex* t, Vertex* h, bool req_i, double c_s, double c_srev, double c_d, double c_drev) : tail_vertex_{t}, head_vertex_{h}, is_required_{req_i}, service_cost_{c_s}, service_cost_rev_{c_srev}, deadhead_cost_{c_d}, deadhead_cost_rev_{c_drev} { SetVertices(t, h); }

			Edge(Vertex* t, Vertex* h, bool req_i, double c_d, double c_drev) : Edge(t, h, req_i, c_d, c_drev, c_d, c_drev) { SetVertices(t, h);  }

			Edge(size_t t, size_t h, bool req_i, double c_d, double c_drev) : Edge(t, h, req_i, c_d, c_drev, c_d, c_drev) {  }

			Edge(size_t t, size_t h, bool req_i = kIsRequired) : tail_vertex_ID_{t}, head_vertex_ID_{h}, is_required_{req_i} { }

			Edge(Vertex* t, Vertex* h, bool req_i = kIsRequired) : tail_vertex_{t}, head_vertex_{h}, is_required_{req_i} { SetVertices(t, h);}

			Edge() { }

			void SetVertices(const Vertex *t, const Vertex *h) {
				tail_vertex_ = t; head_vertex_ = h;
				tail_vertex_ID_ = kNIL; head_vertex_ID_ = kNIL;
				if(tail_vertex_ != nullptr)
					tail_vertex_ID_ = tail_vertex_->GetID();
				if(head_vertex_ != nullptr)
					head_vertex_ID_ = head_vertex_->GetID();
			}

			void SetDataRequiredEdge(const Vertex *t, const Vertex *h, const double s_c, const double s_crev, const double d_c, const double d_crev) {
				SetVertices(t, h);
				is_required_ = kIsRequired;
				service_cost_ = s_c; service_cost_rev_ = s_crev;
				deadhead_cost_ = d_c; deadhead_cost_rev_ = d_crev;
			}

			void SetDataNonRequiredEdge(const Vertex *t, const Vertex *h, const double d_c, const double d_crev) {
				SetVertices(t, h);
				is_required_ = kIsNotRequired;
				deadhead_cost_ = d_c; deadhead_cost_rev_ = d_crev;
			}

			void GetVertices(const Vertex* &t, const Vertex* &h) const {
				t = tail_vertex_; h = head_vertex_;
			}

			void GetVertices(Vertex &t, Vertex &h) const {
				t = *tail_vertex_; h = *head_vertex_;
			}

			int GetTailVertexXY(Vec2d &xy) const{
				if(tail_vertex_ == nullptr)
					return kFail;
				xy = tail_vertex_->GetXY();
				return kSuccess;
			}

			int GetHeadVertexXY(Vec2d &xy) const{
				if(head_vertex_ == nullptr)
					return kFail;
				xy = head_vertex_->GetXY();
				return kSuccess;
			}

			int GetTailVertexXY(double &x, double &y) const{
				if(tail_vertex_ == nullptr)
					return kFail;
				auto xy = tail_vertex_->GetXY();
				x = xy.x; y = xy.y;
				return kSuccess;
			}

			inline int GetHeadVertexXY(double &x, double &y) const{
				if(head_vertex_ == nullptr)
					return kFail;
				auto xy = head_vertex_->GetXY();
				x = xy.x; y = xy.y;
				return kSuccess;
			}

			inline size_t GetTailVertexID () const {return tail_vertex_ID_;}
			inline size_t GetHeadVertexID () const {return head_vertex_ID_;}

			inline void SetCosts(const double c){ cost_ = c; service_cost_ = c; service_cost_rev_ = c; deadhead_cost_ = c; deadhead_cost_rev_ = c; }
			inline void SetServiceCost(const double c){ service_cost_ = c; service_cost_rev_ = c; }
			inline void SetServiceCost(const double c, const double crev){ service_cost_ = c; service_cost_rev_ = crev; }

			inline void SetDeadheadCost(const double c){ deadhead_cost_ = c; deadhead_cost_rev_ = c; }
			inline void SetDeadheadCost(const double c, const double crev){ deadhead_cost_ = c; deadhead_cost_rev_ = crev; }

			inline double GetServiceCost() const { return service_cost_; }
			inline double GetReverseServiceCost() const { return service_cost_rev_; }

			inline double GetDeadheadCost() const { return deadhead_cost_; }
			inline double GetReverseDeadheadCost() const { return deadhead_cost_rev_; }

			inline void SetReq(const bool req_i) { is_required_ = req_i; }
			inline bool GetReq() const { return is_required_; }

			inline void SetCost(const double c) { cost_ = c; }
			inline double GetCost() const { return cost_; }

			void SetDemand(const double demand) { demand_ = demand; }
			void SetServiceDemands(const double q_s, const double qbar_s) { service_demand_ = q_s; service_demand_rev_ = qbar_s; }
			void SetDeadheadDemands(const double q_d, const double qbar_d) { deadhead_demand_ = q_d; deadhead_demand_rev_ = qbar_d; }

			inline double GetDemand() const { return demand_; }
			inline double GetServiceDemand() const { return service_demand_; }
			inline double GetReverseServiceDemand() const { return service_demand_rev_; }
			inline double GetDeadheadDemand() const { return deadhead_demand_; }
			inline double GetReverseDeadheadDemand() const { return deadhead_demand_rev_; }

			void SetDemandsToCosts() {
				service_demand_ = service_cost_;
				service_demand_rev_ = service_cost_rev_;
				deadhead_demand_ = deadhead_cost_;
				deadhead_demand_rev_ = deadhead_cost_rev_;
			}

			/* Computes and sets service and deadhead costs to be the Euclidean costs
			 * Costs are symmetric nad same for service and deahead */
			void ComputeCost() {
				auto t_xy = tail_vertex_->GetXY();
				auto h_xy = head_vertex_->GetXY();
				auto dist = t_xy.Dist(h_xy);
				SetServiceCost(dist, dist);
				SetDeadheadCost(dist, dist);
				cost_ = dist;
			}

			/* Computes and sets service and deadhead costs using the ramp mode
			 * Requires accelearation and velocity
			 * Costs are symmetric and same for service and deadhead */
			void ComputeTravelTimeRamp(const double acc, const double vel) {
				auto t_xy = tail_vertex_->GetXY();
				auto h_xy = head_vertex_->GetXY();
				auto dist = t_xy.Dist(h_xy);
				double t_ramp = vel/acc;
				double dist_ramp = vel * t_ramp / 2.0;
				double total_time;
				if(dist < 2 * dist_ramp) {
					total_time = std::sqrt(4 * dist/acc);
				}
				else {
					total_time = 2 * t_ramp + (dist - 2 * dist_ramp)/vel;
				}
				SetServiceCost(total_time, total_time);
				SetDeadheadCost(total_time, total_time);
				cost_ = total_time;
			}

			void Reverse() {
				std::swap(tail_vertex_, head_vertex_);
				std::swap(service_cost_, service_cost_rev_);
				std::swap(deadhead_cost_, deadhead_cost_rev_);
				std::swap(tail_vertex_ID_, head_vertex_ID_);
				std::swap(service_demand_, service_demand_rev_);
				std::swap(deadhead_demand_, deadhead_demand_rev_);
			}

	};
} // namespace lclibrary

#endif /* LCLIBRARY_CORE_EDGE_HPP_ */

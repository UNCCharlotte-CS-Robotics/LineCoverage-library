/**
 * This file is part of the LineCoverage-library.
 * The file contains base class for MEM
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
 * TODO: use smart pointers
 */

#ifndef LCLIBRARY_MEM_BASE_H_
#define LCLIBRARY_MEM_BASE_H_

#include <lclibrary/core/core.h>
#include <lclibrary/utils/utils.h>
#include <lclibrary/algorithms/algorithms.h>
#include <queue>

namespace lclibrary {

	struct RouteSavings {
		size_t p_;
		size_t q_;
		double savings_;
		size_t savings_perm_;
		double demands_;
		size_t depot_;
		RouteSavings() : p_{kNIL}, q_{kNIL}, savings_{-kDoubleMax} {}
		RouteSavings(const size_t p, const size_t q) : p_{p}, q_{q}, savings_{-kDoubleMax} {}
	};

	struct MEM_Route {
		MEM_Route *route1_ = nullptr;
		MEM_Route *route2_ = nullptr;
		EdgeTuple edge_ = MakeEdgeTuple(nullptr, false);
		size_t vertex_idx_start_ = kNIL;
		size_t vertex_idx_end_ = kNIL;
		size_t edge_idx_start_ = kNIL;
		size_t edge_idx_end_ = kNIL;
		bool start_reversed_ = false;
		bool end_reversed_ = false;
		bool reversed_ = false;
		double cost_ = 0;
		double demand_ = 0;
		double req_cost_ = 0;
		double req_cost_rev_ = 0;
		double req_demand_ = 0;
		double req_demand_rev_ = 0;
		size_t depot_;

		void XOR(const bool reverse) {reversed_ = reversed_ xor reverse;}
		void GetVertices(size_t &t, size_t &h) const {
			if(reversed_) {
				t = vertex_idx_end_; h = vertex_idx_start_;
			}
			else {
				h = vertex_idx_end_; t = vertex_idx_start_;
			}
		}
		void GetEdges(size_t &e_start, size_t &e_end) const {
			e_start = edge_idx_start_; e_end = edge_idx_end_;
		}
	};

	typedef std::priority_queue <RouteSavings, std::vector <RouteSavings>, std::less <RouteSavings> > SavingsHeap;

	inline bool operator<(const RouteSavings &lhs, const RouteSavings &rhs) {
		return lhs.savings_ < rhs.savings_;
	}

	class MEM_Base {
		SavingsHeap savings_heap_;

		virtual bool InitializeRoutes() = 0;
		virtual bool ComputeSavings(RouteSavings &) = 0;
		virtual void Merge(const RouteSavings &) = 0;
		virtual bool IsTourEmpty(const size_t) = 0;
		virtual size_t NumOfRoutes() = 0;

		public:

		MEM_Base() {}

		void MEM() {
			size_t m = NumOfRoutes();
			std::vector <RouteSavings> initial_savings;
			for (size_t i = 0; i < m; ++i) {
				for (size_t j = i + 1; j < m; ++j) {
					RouteSavings rs(i, j);
					if(ComputeSavings(rs) == kSuccess) {
						/* std::cout << "Init: " << rs.depot_  << " " << rs.savings_<< std::endl; */
						initial_savings.push_back(rs);
					}
				}
			}
			savings_heap_ = SavingsHeap(initial_savings.begin(), initial_savings.end());

			/* int count = 0; */
			while(!savings_heap_.empty()) {
				auto pqs = savings_heap_.top();
				savings_heap_.pop();
				if(IsTourEmpty(pqs.p_))
					continue;
				if(IsTourEmpty(pqs.q_))
					continue;
				/* std::cout << "saving: " << pqs.depot_ << " " << pqs.savings_ << std::endl; */
				Merge(pqs);
				auto r = NumOfRoutes() - 1;
				for(size_t i = 0; i < r; ++i) {
					if(IsTourEmpty(i))
						continue;
					RouteSavings rs(r, i);
					if(ComputeSavings(rs) == kSuccess) {
						savings_heap_.push(rs);
					}
				}
			}
		}
	};

}
#endif /* LCLIBRARY_MEM_BASE_H_ */

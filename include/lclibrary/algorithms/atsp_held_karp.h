/**
 * This file is part of the LineCoverage-library.
 * The file contains DP algorithm for ATSP given by Bellman-Held-Karp
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

#ifndef LCLIBRARY_ALGORITHMS_ATSP_DP_BHK_H_
#define LCLIBRARY_ALGORITHMS_ATSP_DP_BHK_H_

#include <lclibrary/core/constants.h>
#include <lclibrary/algorithms/atsp_base.h>

namespace lclibrary {

	class ATSP_DP_BHK: public ATSP {
		struct Path {
			size_t v;
			size_t s;
			double c;
			Path() : v{kNIL}, s{kNIL}, c{kDoubleMax} {}
		};
		size_t n_;
		const std::vector < std::vector <double> > &d_;
		std::vector < std::vector <Path> > subpath_;
		size_t max_subsets_;
		double cost_;

		void SolveATSP(size_t v, size_t s) {
			size_t min_v = kNIL;
			size_t s_min = kNIL;

			for(size_t i = 0; i < n_; ++i) {

				if(i == v || (s & (1 << i))) {
					continue;
				}

				size_t s_new = (s | (1 << i));

				double path_cost = d_[v][i];
				if(s_new == max_subsets_) {
					path_cost += d_[i][0];
				}
				else if(subpath_[i][s_new].v == kNIL) {
					SolveATSP(i, s_new);
					path_cost += subpath_[i][s_new].c;
				}
				else {
					path_cost += subpath_[i][s_new].c;
				}

				if(path_cost < subpath_[v][s].c) {
					subpath_[v][s].c = path_cost;
					min_v = i;
					s_min = s_new;
				}
			}
			subpath_[v][s].v = min_v;
			subpath_[v][s].s = s_min;
		}

		public:
		ATSP_DP_BHK(const size_t n, const std::vector < std::vector <double> > &d) : n_{n}, d_{d} {
			max_subsets_ = (1 << n) - 1;
			subpath_.resize(n, std::vector <Path>(max_subsets_));
			SolveATSP(0, 1);
			cost_ = subpath_[0][1].c;
		}


		int GetPath(std::vector < size_t > &path) const {
			path.push_back(0);
			double cost = 0;
			size_t v = 0; size_t s = 1;
			for(size_t i = 0; i < n_ - 1; ++i) {
				auto p = subpath_[v][s];
				cost += d_[v][p.v];
				v = p.v;
				s = p.s;
				path.push_back(v);
			}
			path.push_back(0);
			cost += d_[v][0];
			if(path.size() != (n_ + 1))
				return 1;
			if(!IsNearZero(cost - cost_))
				return 1;
			return 0;
		}

	};

} // namespace lclibrary

#endif /* LCLIBRARY_ALGORITHMS_ATSP_DP_BHK_H_ */

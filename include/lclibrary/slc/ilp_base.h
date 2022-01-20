/**
 * This file is part of the LineCoverage-library.
 * The file contains the base class for SLC ILP solver
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

#ifndef LCLIBRARY_SLC_ILP_H_
#define LCLIBRARY_SLC_ILP_H_

#include <lclibrary/core/constants.h>
#include <lclibrary/slc/slc_base.h>
#include <memory>

namespace lclibrary {

	class SLC_ILP : public SLC_Base {

		public:
		SLC_ILP(std::shared_ptr <const Graph> g_in) : SLC_Base (g_in) {}
		virtual int SolveILP() = 0;
		virtual int GenerateSolutionGraph() = 0;

		int Solve() {
			SolveILP();
			int exit_status = GenerateSolutionGraph();
			if(exit_status == 0 or exit_status == 1) {
				sol_digraph_->PrintNM();
				route_ = EulerTourGeneration(sol_digraph_);

				if(g_->IsDepotSet()) {
					route_.RotateToDepot(g_->GetVertexID(g_->GetDepot()));
				sol_digraph_->SetDepot(g_->GetDepotID());
				}

				std::cout << sol_digraph_->GetCost() << std::endl;
			}
			return exit_status;
		}

		double GetRouteCost() {
			return route_.GetCost();
		}
	};

}
#endif /* LCLIBRARY_SLC_ILP_H_ */

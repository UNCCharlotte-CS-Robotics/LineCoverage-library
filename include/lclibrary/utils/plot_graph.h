/**
 * This file is part of the LineCoverage-library.
 * The file contains functins for plotting using Gnuplot
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

#ifndef LCLIBRARY_UTILS_PLOTGRAPH_H_
#define LCLIBRARY_UTILS_PLOTGRAPH_H_

#include <cmath>
#include <memory>
#include <fstream>
#include <filesystem>
#include <lclibrary/core/config.h>
#include <lclibrary/core/core.h>
#include <lclibrary/core/graph_wrapper.h>

namespace lclibrary {

	void PlotDataRequiredEdges(
			const std::shared_ptr <const Graph> &,
			const std::string,
			std::string color = "1");

	bool PlotDataNonRequiredEdges(
			const std::shared_ptr <const Graph> &,
			const std::string,
			std::string color = "2");

	void PlotDataRequiredEdgesArrows(
			const std::shared_ptr <const Graph> &,
			const std::string,
			std::string color = "1");

	bool PlotDataNonRequiredEdgesArrows(
			const std::shared_ptr <const Graph> &,
			const std::string,
			std::string color = "2");

	void GnuplotMap(
			const std::shared_ptr <const Graph> &,
			const std::string,
			const std::string,
			const std::string,
			bool plot_non_required = false);

	void GnuplotMap(
			const std::vector <std::shared_ptr <Graph>> &,
			const std::string,
			const std::string,
			const std::string,
			bool plot_non_required = false);

	void GnuplotMapArrows(
			const std::shared_ptr <const Graph> &,
			const std::string,
			const std::string,
			const std::string,
			bool plot_non_required = false);

	int PlotMap(const Config &);
	int PlotMap(const Config &, std::shared_ptr <const Graph>);

} /* lclibrary */
#endif /*  LCLIBRARY_UTILS_PLOTGRAPH_H_*/

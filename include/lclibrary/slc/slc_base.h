/**
 * This file is part of the LineCoverage-library.
 * The file contains base class for SLC
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

#ifndef LCLIBRARY_SLC_SLC_BASE_H_
#define LCLIBRARY_SLC_SLC_BASE_H_

#include <lclibrary/core/core.h>
#include <lclibrary/utils/utils.h>
#include <lclibrary/algorithms/is_balanced.h>
#include <lclibrary/algorithms/euler_tour.h>
#include <lclibrary/algorithms/connected_components.h>
#include <memory>

namespace lclibrary {

	class SLC_Base {

		protected:
		std::shared_ptr <const Graph> g_;
		std::shared_ptr <Graph> sol_digraph_;
		Route route_;
		bool use_2opt_ = true;

		public:
		SLC_Base(const std::shared_ptr <const Graph> &g_in) : g_{g_in} {};
		virtual int Solve() = 0;
		virtual bool CheckSolution() {
			if (!IsBalancedDigraph(sol_digraph_))
				return false;
			ConnectedComponents cc(sol_digraph_);
			cc.StronglyCCBalanced();
			if(cc.GetNumCC() != 1)
				return false;
			return true;
		}

		virtual void Use2Opt(const bool use) {
			use_2opt_ = use;
		}

		void Gnuplot(
				const std::string data_file_name,
				const std::string gnuplot_file_name,
				const std::string output_plot_file_name,
				bool plot_non_required) const {
			/* GnuplotMapArrows(sol_digraph_, data_file_name, gnuplot_file_name, output_plot_file_name, plot_non_required); */
			GnuplotMap(sol_digraph_, data_file_name, gnuplot_file_name, output_plot_file_name, plot_non_required);
		}

		void GenerateVideo(std::string video_dir, const size_t num_frames = 900) const {
			VideoGenerator(sol_digraph_, &route_, video_dir, num_frames);
		}

		void GetRoute(Route &r) const {
			r = route_;
		}

		void WriteKML(const std::string file_name) const {
			route_.WriteKML(file_name);
		}

		void WriteKMLReverse(const std::string file_name) const {
			route_.WriteKMLReverse(file_name);
		}

		void WriteRouteData(const std::string file_name) const {
			route_.WriteRouteData(file_name);
		}

		void WriteRouteEdgeData(const std::string file_name) const {
			route_.WriteRouteEdgeData(file_name);
		}

		void WriteGeoJSON(const std::string file_name, const std::string var_name = "graph_data") const {
			WriteGeoJSON_All(sol_digraph_, file_name, var_name);
		}

		void GetSolDigraph(Graph &digraph) const {
			digraph = *sol_digraph_;
		}

		double GetRouteCost() {
			return route_.GetCost();
		}

		size_t GetNumLocalMoves() const {
			return route_.GetNumLocalMoves();
		}

		virtual void GetComputationTimes(std::vector <double> &comp_t) {}
		virtual void GetCosts(std::vector <double> &costs) {}

		int RouteOutput (const Config &config) const {
			std::string sol_dir = config.sol_dir;
			if(not std::filesystem::exists(sol_dir)) {
				std::filesystem::create_directory(sol_dir);
			}

			config.WriteConfig(sol_dir + "config.yaml");
			std::string filename_prepend = sol_dir + config.problem + "_" + config.solver_slc + "_";
			if(config.route_output.plot) {
				std::string plot_dir = sol_dir + "/plot/";
				std::filesystem::create_directory(plot_dir);
				std::string gnuplot_filename = plot_dir + "/plot.gp";
				std::string plot_filename =  filename_prepend + "route";

				Gnuplot(plot_dir + "/plot_data", gnuplot_filename, plot_filename, true);
				auto gnuplot_status = std::system(("gnuplot " + gnuplot_filename).c_str());
				std::filesystem::remove_all(plot_dir);
				if(gnuplot_status != 0) {
					std::cerr << "gnuplot failed\n";
					return kFail;
				}
			}

			if(config.route_output.kml) {
				WriteKML(filename_prepend + "route.kml");
			}

			if(config.route_output.data) {
				WriteRouteData(filename_prepend + "route");
			}

			if(config.route_output.edge_data) {
				WriteRouteEdgeData(filename_prepend + "route_edges");
			}

			if(config.route_output.geojson) {
				WriteGeoJSON(filename_prepend + "route.json");
			}

			return kSuccess;
		}

	};

}
#endif /* LCLIBRARY_SLC_SLC_BASE_H_ */

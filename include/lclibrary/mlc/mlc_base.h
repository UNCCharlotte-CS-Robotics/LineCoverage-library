/**
 * This file is part of the LineCoverage-library.
 * The file contains base class for MLC
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

#ifndef LCLIBRARY_MLC_MLC_BASE_H_
#define LCLIBRARY_MLC_MLC_BASE_H_

#include <lclibrary/core/core.h>
#include <lclibrary/utils/utils.h>
#include <lclibrary/algorithms/algorithms.h>

namespace lclibrary {

	class MLC_Base {

		protected:
		std::shared_ptr <const Graph> g_;
		std::vector <std::shared_ptr <Graph>> sol_digraph_list_;
		std::vector <Route> route_list_;
		bool use_2opt_ = true;

		public:
		MLC_Base(const std::shared_ptr <const Graph> g_in) : g_{g_in} {};
		virtual int Solve() = 0;

		virtual bool CheckSolution() {
			for(const auto &sol_digraph:sol_digraph_list_) {
				if (IsBalancedDigraph(sol_digraph) == false)
					return false;
				ConnectedComponents cc(sol_digraph);
				cc.StronglyCCBalanced();
				if(cc.GetNumCC() != 1)
					return false;
			}
			return true;
		}

		virtual ~MLC_Base() {
		}

		virtual void Use2Opt(const bool use) {
			use_2opt_ = use;
		}

		void Gnuplot(const std::string data_file_name, const std::string gnuplot_file_name, const std::string output_plot_file_name, bool plot_non_required) const {
			if(sol_digraph_list_.empty()) {
				std::cerr << "MLC not solved\n";
				return;
			}
			GnuplotMap(sol_digraph_list_, data_file_name, gnuplot_file_name, output_plot_file_name, plot_non_required);
		}

		/* void GenerateVideo(std::string video_dir, const size_t num_frames = 900) { */
		/* 	VideoGenerator(sol_digraph_, &route_, video_dir, num_frames); */
		/* } */

		void GetRoutes(std::vector <Route> &routes) const{
			routes = route_list_;
		}

		void WriteRouteEdgeData(const std::string file_name) const {
			for(size_t i = 0; i < route_list_.size(); ++i) {
				route_list_[i].WriteRouteEdgeData(file_name + std::to_string(i));
			}
		}

		void WriteKML(const std::string file_name) const{
			for(size_t i = 0; i < route_list_.size(); ++i) {
				route_list_[i].WriteKML(file_name + std::to_string(i) + ".kml");
			}
		}

		void WriteGeoJSON(const std::string file_name, const std::string var_name = "graph_data") const {
			for(size_t i = 0; i < sol_digraph_list_.size(); ++i) {
				WriteGeoJSON_All(sol_digraph_list_[i], file_name + std::to_string(i) + ".json", var_name);
			}
		}

		void WriteKMLReverse(const std::string file_name) const{
			for(size_t i = 0; i < route_list_.size(); ++i) {
				route_list_[i].WriteKMLReverse(file_name + "_reverse_" + std::to_string(i) + ".kml");
			}
		}
		void WriteRouteData(const std::string file_name) const{
			for(size_t i = 0; i < route_list_.size(); ++i) {
				route_list_[i].WriteRouteData(file_name + std::to_string(i));
			}
		}

		void GetSolDigraphList(std::vector <std::shared_ptr<const Graph>> &list) const{
			list.clear();
			for(const auto &g:sol_digraph_list_) {
				list.push_back(g);
			}
		}

		double GetRouteCost() const {
			double total_cost = 0;
			for(const auto &route:route_list_) {
				total_cost += route.GetCost();
			}
			return total_cost;
		}

		size_t GetNumOfRoutes() const {
			return route_list_.size();
		}

		size_t GetTotalNumTurns() const {
			size_t num_turns = 0;
			for(const auto &route:route_list_) {
				num_turns += route.GetNumTurns();
			}
			return num_turns;

		}

		void WriteWaypointsRoutes(std::string filename) const {
			for(size_t i = 0; i < route_list_.size(); ++i) {
				route_list_[i].WriteWayPoints(filename + std::to_string(i));
			}
		}

		int RouteOutput (const Config &config) const {
			std::string sol_dir = config.sol_dir;
			if(not std::filesystem::exists(sol_dir)) {
				std::filesystem::create_directory(sol_dir);
			}
			config.WriteConfig(sol_dir + "config.yaml");
			std::string filename_prepend = sol_dir + config.problem + "_" + config.solver_mlc + "_";
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
				WriteKML(filename_prepend + "route");
			}

			if(config.route_output.data) {
				WriteRouteData(filename_prepend + "route");
			}

			if(config.route_output.edge_data) {
				WriteRouteEdgeData(filename_prepend + "route_edges");
			}

			if(config.route_output.geojson) {
				WriteGeoJSON(filename_prepend + "route");
			}

			return kSuccess;
		}

		virtual double GetObjBound() {return 0;}

	};

}
#endif /* LCLIBRARY_MLC_MLC_BASE_H_ */

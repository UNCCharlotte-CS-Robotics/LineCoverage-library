/**
 * This file is part of the LineCoverage-library.
 * Solve SLC
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

#include <lclibrary/core/config.h>
#include <lclibrary/core/core.h>
#include <lclibrary/utils/plot_graph.h>
#include <lclibrary/utils/write_geojson.h>
#include <lclibrary/slc/slc.h>
#include <lclibrary/utils/convert_osm_json_graph.h>
#include <chrono>
#include <filesystem>
#include <cstdlib>

int main (int argc, char **argv) {
	if(argc < 2) {
		std::cerr << "Usage: " << argv[0] << " <config_file.yaml" << std::endl;
		return 1;
	}

	std::string config_file = argv[1];
	lclibrary::Config config(config_file);

	if(argc == 3) {
		std::string data_dir = argv[2];
		config = lclibrary::Config(config_file, data_dir);
	}

	if (config.ParseConfig()) {
		std::cerr << "Config parsing failed with config file: " << config_file << std::endl;
	}

	if(config.convert_osm_graph == true) {
		if(lclibrary::OSMjsonGraph(config)) {
			std::cerr << "OSM JSON to graph failed\n";
			return 1;
		}
	}

	if(config.plot_input_graph.plot == true) {
		if(lclibrary::PlotMap(config)) {
			std::cerr << "Plotting of input map failed\n";
			return 1;
		}
	}

	if(config.writeGeoJSON.write) {
		if(lclibrary::WriteGeoJSON(config)) {
			std::cerr << "WriteGeoJSON of input map failed\n";
			return 1;
		}
	}

	if(config.problem != "slc") {
		std::cerr << "Problem not set to slc in config\n";
		return 1;
	}

	auto t_start_all = std::chrono::high_resolution_clock::now();

	std::shared_ptr <lclibrary::Graph> g;

	if(lclibrary::GraphCreateWithCostFn(config, g)) {
		std::cerr << "Graph creation failed\n";
		return 1;
	}

	if(g->IsDepotSet()){
		if(g->CheckDepotRequiredVertex() == lclibrary::kFail) {
			std::cerr << "Depot is not a required vertex\n";
			return 1;
		}
	}

	std::unique_ptr <lclibrary::SLC_Base> slc_solver;

	if(config.solver_slc == "ilp_gurobi") {
#ifndef LCLIBRARY_USE_GUROBI
		std::cerr << "Gurobi not configured\n";
		return 1;
#endif
	}

	int solver_status = 1;
	if(config.solver_slc == "beta2_atsp" or config.solver_slc == "ilp_gurobi") {
		slc_solver = std::make_unique <lclibrary::SLC_Beta2ATSP> (g);
	} else if(config.solver_slc == "beta2_gtsp"){
		slc_solver = std::make_unique <lclibrary::SLC_Beta2GTSP> (g);
	} else if(config.solver_slc == "beta3_atsp"){
		slc_solver = std::make_unique <lclibrary::SLC_Beta3ATSP> (g);
	} else if(config.solver_slc == "ilp_glpk") {
		slc_solver = std::make_unique <lclibrary::SLC_ILP_glpk> (g);
	}

	if(slc_solver == nullptr) {
		std::cerr << "Invalid SLC solver\n";
		return 1;
	}

	slc_solver->Use2Opt(config.use_2opt);
	solver_status = slc_solver->Solve();

	if(config.solver_slc == "ilp_gurobi") {
		std::string ws_file = config.database.dir + "/gurobi_warmstart";
		slc_solver->WriteRouteData(ws_file);
		slc_solver.reset(nullptr);
#ifdef LCLIBRARY_USE_GUROBI
		slc_solver = std::make_unique <lclibrary::SLC_ILP_Gurobi> (g, ws_file, config.ilp_time_limit);
		solver_status = slc_solver->Solve();
		if(solver_status == 1) {
			std::cout << "ILP did not converge. Time limit reached\n";
		}
#endif
		std::filesystem::remove(ws_file);
	}

	auto t_end_all = std::chrono::high_resolution_clock::now();
	double time_all = std::chrono::duration<double, std::milli>(t_end_all-t_start_all).count();
	double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end_all - t_start_all).count();


	std::cout << std::boolalpha;
	std::cout << config.problem << ": " << config.solver_slc << ": solution connectivity check " << slc_solver->CheckSolution() << std::endl;

	slc_solver->RouteOutput(config);

	if(config.route_output.agg_results) {
		std::string result_filename = config.sol_dir + "results";
		std::cout << result_filename << std::endl;
		std::ofstream result_file;
		if(config.route_output.append) {
			result_file.open(result_filename, std::ios_base::app);
		} else {
			result_file.open(result_filename);
		}

		result_file << g->GetN() << " " << g->GetM() << " " << g->GetMnr() << " " << g->GetLength() << " " << lclibrary::GetNumCCRequiredGraph(g);

		if(config.solver_slc == "beta2_atsp" or config.solver_slc == "beta2_gtsp" or config.solver_slc == "beta3_atsp") {
			std::vector <double> comp_t;
			std::vector <double> costs;
			slc_solver->GetComputationTimes(comp_t);
			slc_solver->GetCosts(costs);
			result_file << " " << config.use_2opt << " " << slc_solver->GetNumLocalMoves() << " " << comp_t[0] << " " << comp_t[1] << " " << comp_t[2] << " " << comp_t[3] << " " << time_all;
			result_file << " " << costs[0] << " " << costs[1] << std::endl;
		}

		if(config.solver_slc == "ilp_gurobi" or config.solver_slc == "ilp_glpk") {
			result_file << " " << slc_solver->GetRouteCost() << " " << elapsed_time_ms  << " " << solver_status << " " << slc_solver->CheckSolution() << std::endl;
		}

		result_file.close();
	}
}

/**
 * This file is part of the LineCoverage-library.
 * Solve MLC
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

#include <lclibrary_config.h>
#include <lclibrary/core/config.h>
#include <lclibrary/core/core.h>
#include <lclibrary/utils/plot_graph.h>
#include <lclibrary/utils/write_geojson.h>
#include <lclibrary/mlc/mlc.h>
#include <lclibrary/utils/convert_osm_json_graph.h>
#include <chrono>
#include <filesystem>
#include <cstdlib>
#ifdef LCLIBRARY_USE_GUROBI
#include <lclibrary/mlc/ilp_gurobi.h>
#endif

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

	if(argc == 4) {
		std::string data_dir = argv[2];
		double capacity = std::stod(argv[3]);
		config = lclibrary::Config(config_file, data_dir, capacity);
	}

	double lower_bound = 0;
	if(argc == 5) {
		std::string data_dir = argv[2];
		double capacity = std::stod(argv[3]);
		config = lclibrary::Config(config_file, data_dir, capacity);
		lower_bound = std::stod(argv[4]);
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

	if(config.problem != "mlc") {
		std::cerr << "Problem not set to mlc in config\n";
		return 1;
	}

	auto t_start_all = std::chrono::high_resolution_clock::now();

	std::shared_ptr <lclibrary::Graph> g;

	if(lclibrary::GraphCreateWithCostFn(config, g)) {
		std::cerr << "Graph creation failed\n";
		return 1;
	}
	g->PrintNM();

	if(g->IsDepotSet() == false){
		std::cerr << "Depot is not set. Depot is required for MLC\n";
		return 1;
	}

	std::unique_ptr <lclibrary::MLC_Base> mlc_solver;

	if(config.solver_mlc == "ilp_gurobi") {
#ifndef LCLIBRARY_USE_GUROBI
		std::cerr << "Gurobi not configured\n";
		return 1;
#endif
	}

	int solver_status = 1;
	if(config.solver_mlc == "mem" or config.solver_mlc == "ilp_gurobi") {
		mlc_solver = std::make_unique <lclibrary::MLC_MEM> (g);
	}

	if(mlc_solver == nullptr) {
		std::cerr << "Invalid MLC solver\n";
		return 1;
	}

	mlc_solver->Use2Opt(config.use_2opt);
	solver_status = mlc_solver->Solve();

	if(config.solver_mlc == "ilp_gurobi") {
		std::vector <lclibrary::Route> route_list;
		mlc_solver->GetRoutes(route_list);
		mlc_solver.reset(nullptr);
#ifdef LCLIBRARY_USE_GUROBI
		if(argc == 5) {
			mlc_solver = std::make_unique <lclibrary::MLC_ILP_Gurobi> (g, route_list, config.ilp_time_limit, lower_bound);
		} else {
			mlc_solver = std::make_unique <lclibrary::MLC_ILP_Gurobi> (g, route_list, config.ilp_time_limit);
		}
		solver_status = mlc_solver->Solve();
		if(solver_status == 1) {
			std::cout << "ILP did not converge. Time limit reached\n";
		}
#endif
	}

	auto t_end_all = std::chrono::high_resolution_clock::now();
	double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end_all - t_start_all).count();

	std::cout << std::boolalpha;
	std::cout << config.problem << ": " << config.solver_mlc << ": solution connectivity check " << mlc_solver->CheckSolution() << std::endl;

	mlc_solver->RouteOutput(config);

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

		if(config.solver_mlc == "mem") {
			result_file << " " << mlc_solver->GetRouteCost() << " " << mlc_solver->GetNumOfRoutes() << " " << elapsed_time_ms  << " " << solver_status << " " << mlc_solver->CheckSolution() << std::endl;
		}

		if(config.solver_mlc == "ilp_gurobi") {
			result_file << " " << mlc_solver->GetRouteCost() << " " << mlc_solver->GetNumOfRoutes() << " " << mlc_solver->GetObjBound() << " " << elapsed_time_ms  << " " << solver_status << " " << mlc_solver->CheckSolution()<< std::endl;
		}

		result_file.close();
	}

}

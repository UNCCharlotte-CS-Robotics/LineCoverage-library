/**
 * This file is part of the LineCoverage-library.
 * The file contains class for parsing and storing configuration. Uses yaml-cpp.
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

#ifndef LCLIBRARY_CORE_CONFIG_H_
#define LCLIBRARY_CORE_CONFIG_H_

#include <lclibrary/core/constants.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>

namespace lclibrary {

	class Config {

		public:
			std::string config_file_;

			struct Database {
				std::string path;
				std::string data_dir;
				std::string dir;
				bool arg = false;
			} database;

			std::string sol_dir;
			struct Filenames {
				std::string map_json;
				std::string nodes_ll;
				std::string nodes_data;
				std::string req_edges;
				std::string nonreq_edges;
			} filenames;

			struct InputGraph {
				bool lla = true;
				bool costs = false;
			} input_graph;

			bool convert_osm_graph = false;
			bool add_pairwise_nonreq_edges = false;

			struct PlotInGraph {
				bool plot = false;
				bool plot_nreq_edges = false;
				std::string name;
			} plot_input_graph;

			struct WriteGeoJSON {
				bool write = false;
				bool non_req_edges = false;
				std::string filename;
				std::string var_name;
			} writeGeoJSON;

			std::string problem;
			std::string solver_slc;
			std::string solver_mlc;
			std::string solver_mlc_md;

			bool use_2opt;
			double ilp_time_limit;
			double capacity;
			bool cap_arg = false;

			std::string cost_function;
			struct TravelTime {
				double service_speed;
				double deadhead_speed;
				double wind_speed;
				double wind_dir;
			} travel_time;

			struct TravelTimeCircTurns {
				double service_speed;
				double deadhead_speed;
				double wind_speed;
				double wind_dir;
				double angular_vel;
				double acc;
				double delta;
			} travel_time_circ_turns;

			struct RouteOutput {
				bool plot = false;
				bool kml = false;
				bool data = false;
				bool edge_data = false;
				bool geojson = false;
				bool agg_results = false;
				bool append = false;
				bool clear_dir = false;
			} route_output;

			enum DepotMode {mean, custom, none } depot_mode;
			enum DepotsMode {cluster_auto, cluster, user } depots_mode;
			size_t depot_ID = 0;
			std::vector <size_t> depot_IDs;
			size_t num_depots;
			size_t num_runs;
			bool use_seed = false;
			double seed = 0;
			bool nd_arg = false;

			Config () {}

			Config (const std::string &config_file) {
				config_file_ = config_file;
			}

			Config (const std::string &config_file, const std::string &data_dir):Config(config_file) {
				database.data_dir = data_dir;
				database.arg = true;
			}

			Config (const std::string &config_file, const std::string &data_dir, const double cap):Config(config_file, data_dir) {
				capacity = cap;
				cap_arg = true;
			}

			Config (const std::string &config_file, const std::string &data_dir, const double cap, const size_t n):Config(config_file, data_dir, cap){
				num_depots = n;
				nd_arg = true;
			}

			void CopyConfig(const std::string &filename) const {
				std::filesystem::copy(config_file_, filename, std::filesystem::copy_options::overwrite_existing);
			}

			int WriteConfig(const std::string &filename) const {
				YAML::Node yaml_config_ = YAML::LoadFile(config_file_);
				if(database.arg == true) {
					yaml_config_["database"]["data_dir"] = database.data_dir;
				}

				if(cap_arg == true) {
					yaml_config_["capacity"] = capacity;
				}

				if(nd_arg == true) {
					yaml_config_["depots"]["num_depts"] = num_depots;
				}

				std::ofstream fout(filename);
				fout << yaml_config_;
				fout.close();
				return kSuccess;
			}

			int ParseConfig () {
				std::cout << "Using config file: " << config_file_ << std::endl;
				if(not std::filesystem::exists(config_file_)) {
					std::cerr << "Could not find config file " << config_file_ << std::endl;
					return kFail;
				}
				YAML::Node yaml_config_ = YAML::LoadFile(config_file_);

				if(database.arg == true) {
					yaml_config_["database"]["data_dir"] = database.data_dir;
				}

				problem = yaml_config_["problem"].as<std::string>();

				if(cap_arg == true) {
					yaml_config_["capacity"] = capacity;
				}

				if(nd_arg == true) {
					yaml_config_["depots"]["num_depts"] = num_depots;
				}

				if(problem == "slc" or problem == "mlc") {
					std::string depot_config = yaml_config_["depot"]["mode"].as<std::string>();
					if(depot_config == "mean") {
						depot_mode = mean;
					} else if (depot_config == "custom"){
						depot_mode = custom;
						depot_ID = yaml_config_["depot"]["ID"].as<size_t>();
					} else if (depot_config == "none") {
						depot_mode = none;
					}
				}

				if(problem == "mlc_md") {
					std::string depots_config = yaml_config_["depots"]["mode"].as<std::string>();
					if(depots_config == "cluster_auto") {
						depots_mode = cluster_auto;
					} else if (depots_config == "cluster"){
						depots_mode = cluster;
						num_depots = yaml_config_["depots"]["num_depots"].as<size_t>();
					} else if (depots_config == "user") {
						depots_mode = user;
						auto yaml_depots = yaml_config_["depots"]["IDs"];
						for(YAML::const_iterator it = yaml_depots.begin(); it != yaml_depots.end(); ++it) {
							depot_IDs.push_back(it->as<size_t>());
						}
					}
					use_seed = yaml_config_["depots"]["use_seed"].as<bool>();
					if(use_seed) {
						seed = yaml_config_["depots"]["seed"].as<double>();
					}
					num_runs = yaml_config_["depots"]["num_runs"].as<size_t>();
				}


				auto database_yaml = yaml_config_["database"];
				database.path = database_yaml["path"].as<std::string>();
				database.data_dir = database_yaml["data_dir"].as<std::string>();
				database.dir = database.path + "/" + database.data_dir + "/";

				if(not std::filesystem::exists(database.dir)) {
					std::cerr << "Database does not exist\n";
					std::cerr << database.dir << std::endl;
					return kFail;
				}

				auto filenames_yaml = yaml_config_["filenames"];
				filenames.map_json = filenames_yaml["map_json"].as<std::string>();
				filenames.nodes_ll = filenames_yaml["nodes_ll"].as<std::string>();
				filenames.nodes_data = filenames_yaml["nodes_data"].as<std::string>();
				filenames.req_edges = filenames_yaml["req_edges"].as<std::string>();
				filenames.nonreq_edges = filenames_yaml["nonreq_edges"].as<std::string>();

				convert_osm_graph = yaml_config_["convert_osm_json"].as<bool>();
				add_pairwise_nonreq_edges = yaml_config_["add_pairwise_nonreq_edges"].as<bool>();

				plot_input_graph.plot = yaml_config_["plot_input_graph"]["plot"].as<bool>();
				plot_input_graph.plot_nreq_edges = yaml_config_["plot_input_graph"]["plot_nreq_edges"].as<bool>();
				plot_input_graph.name = yaml_config_["plot_input_graph"]["name"].as<std::string>();

				input_graph.lla = yaml_config_["input_graph"]["lla"].as<bool>();
				input_graph.costs = yaml_config_["input_graph"]["costs"].as<bool>();

				auto writeGeoJSON_yaml = yaml_config_["writeGeoJSON"];
				writeGeoJSON.write = writeGeoJSON_yaml["write"].as<bool>();
				writeGeoJSON.non_req_edges = writeGeoJSON_yaml["non_req_edges"].as<bool>();
				writeGeoJSON.filename = writeGeoJSON_yaml["filename"].as<std::string>();
				writeGeoJSON.var_name = writeGeoJSON_yaml["var_name"].as<std::string>();

				if(problem == "slc") {
					solver_slc = yaml_config_["solver_slc"].as<std::string>();
					sol_dir = database.dir + problem + "_" + solver_slc + "/";
				}
				if(problem == "mlc") {
					solver_mlc = yaml_config_["solver_mlc"].as<std::string>();
					sol_dir = database.dir + problem + "_" + solver_mlc + "/";
				}
				if(problem == "mlc_md") {
					solver_mlc_md = yaml_config_["solver_mlc_md"].as<std::string>();
					sol_dir = database.dir + problem + "_" + solver_mlc_md + "/";
				}

				use_2opt = yaml_config_["use_2opt"].as<bool>();
				ilp_time_limit = yaml_config_["ilp_time_limit"].as<double>();
				capacity = yaml_config_["capacity"].as<double>();

				cost_function = yaml_config_["cost_function"].as<std::string>();
				if(cost_function == "travel_time") {
					auto travel_time_yaml = yaml_config_["travel_time_config"];
					travel_time.service_speed = travel_time_yaml["service_speed"].as<double>();
					travel_time.deadhead_speed = travel_time_yaml["deadhead_speed"].as<double>();
					travel_time.wind_speed = travel_time_yaml["wind_speed"].as<double>();
					travel_time.wind_dir = travel_time_yaml["wind_dir"].as<double>() * M_PI/180.;
				}

				if(cost_function == "travel_time_circturns") {
					auto travel_time_yaml = yaml_config_["travel_time_circturns_config"];
					travel_time_circ_turns.service_speed = travel_time_yaml["service_speed"].as<double>();
					travel_time_circ_turns.deadhead_speed = travel_time_yaml["deadhead_speed"].as<double>();
					travel_time_circ_turns.wind_speed = travel_time_yaml["wind_speed"].as<double>();
					travel_time_circ_turns.wind_dir = travel_time_yaml["wind_dir"].as<double>() * M_PI/180.;
					travel_time_circ_turns.acc = travel_time_yaml["acceleration"].as<double>();
					travel_time_circ_turns.angular_vel = travel_time_yaml["angular_vel"].as<double>() * M_PI/180.;
					travel_time_circ_turns.delta = travel_time_yaml["delta"].as<double>();
				}

				auto route_output_yaml = yaml_config_["route_output"];
				route_output.plot = route_output_yaml["plot"].as<bool>();
				route_output.kml = route_output_yaml["kml"].as<bool>();
				route_output.data = route_output_yaml["data"].as<bool>();
				route_output.edge_data = route_output_yaml["edge_data"].as<bool>();
				route_output.geojson = route_output_yaml["geojson"].as<bool>();
				route_output.agg_results = route_output_yaml["agg_results"].as<bool>();
				route_output.append = route_output_yaml["append"].as<bool>();
				route_output.clear_dir = route_output_yaml["clear_dir"].as<bool>();
				if(std::filesystem::exists(sol_dir)) {
					if(route_output.clear_dir) {
						std::filesystem::remove_all(sol_dir);
					}
				}

				return kSuccess;
			}

	};

} // namespace lclibrary

#endif /* LCLIBRARY_CORE_CONFIG_H_ */

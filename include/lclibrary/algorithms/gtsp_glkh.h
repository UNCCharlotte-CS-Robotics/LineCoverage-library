/**
 * This file is part of the LineCoverage-library.
 * The file contains interfacing class for solving ATSP using LKH
 *
 * TODO: I do not like the STRINGIFY macro. There should be a better way to do this.
 *			 What is GLKH_WRITE_PATH? It is coming from the Makefile! Find a better way to handle this.
 *			 GLKH and LKH are not easy to call and use without modification to the source files. The license does not permit me to handle this in a convenient way. Perhaps I should write a wrapper library to take care of this.
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

#ifndef LCLIBRARY_ALGORITHMS_GTSP_GLKH_H_
#define LCLIBRARY_ALGORITHMS_GTSP_GLKH_H_

#ifdef LCLIBRARY_USE_GLKH
#include <lclibrary_config.h>
#include <lclibrary/core/constants.h>
#include <lclibrary/utils/create_temp_dir.h>
#include <cstdint>
#include <filesystem>

namespace lclibrary {

	class GTSP_GLKH {
		size_t n_;
		size_t num_cc_;
		const std::vector < std::vector <double> > &d_;
		const std::vector < size_t > &vertex_cc_;
		double cost_;
		std::vector <size_t> path_;
		std::vector <std::vector <size_t>> clusters_;
		std::string write_dir_;
		std::string parameter_file_name_;
		std::string problem_file_name_;
		std::string out_file_name_;
		std::string pi_file_name_;

		void WriteParameterFile() const {
			std::ofstream parameter_file(parameter_file_name_);
			std::cout << parameter_file_name_ << std::endl;
			parameter_file <<  "PROBLEM_FILE = " + problem_file_name_ + "\n";
			parameter_file <<  "ASCENT_CANDIDATES = 500\n";
			parameter_file <<  "MAX_CANDIDATES = 30\n";
			parameter_file <<  "INITIAL_PERIOD = 1000\n";
			parameter_file <<  "PRECISION = 10\n";
			parameter_file <<  "MAX_TRIALS = 1000\n";
			parameter_file <<  "TOUR_FILE = " + out_file_name_ + "\n";
			parameter_file <<  "PI_FILE = " + pi_file_name_ + "\n";
			parameter_file <<  "POPULATION_SIZE = 1\n";
			parameter_file <<  "RUNS = 1\n";
			parameter_file <<  "TRACE_LEVEL = 0";
			parameter_file.close();
		}

		void WriteProblemFile() const {
			std::ofstream problem_file(problem_file_name_);
			problem_file << "NAME : agtsp\n";
			problem_file << "TYPE : AGTSP\n";
			problem_file << "DIMENSION : " << n_ << "\n";
			problem_file << "GTSP_SETS : " << num_cc_ << "\n";
			problem_file << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
			problem_file << "EDGE_WEIGHT_FORMAT: FULL_MATRIX\n";
			problem_file << "EDGE_WEIGHT_SECTION\n";
			double sum_max = 0;
			for(size_t i = 0; i < n_; ++i) {
				sum_max += *(std::max_element(d_[i].cbegin(), d_[i].cend()));
			}
			size_t multiplier = kIntMax/sum_max - 1;
			if (multiplier > 3) {
				multiplier = 3;
			}
			std::cout << "multiplier: " << multiplier << std::endl;
			multiplier = std::pow(10, multiplier);
			for(size_t i = 0; i < n_; ++i) {
				for(size_t j = 0; j < n_; ++j) {
					problem_file << (size_t)(d_[i][j] * multiplier) << " ";
				}
				problem_file << "\n";
			}
			problem_file << "GTSP_SET_SECTION\n";
			for(size_t i = 0; i < clusters_.size(); ++i) {
				problem_file << i + 1 << " ";
				for(size_t j = 0; j < clusters_[i].size(); ++j) {
					problem_file << clusters_[i][j] + 1 << " ";
				}
				problem_file << "-1\n";
			}
			problem_file << "EOF";
			problem_file.close();
		}

		void ReadTourFile() {
			std::cout << "Reading tour file " << out_file_name_ << "\n";
			std::ifstream tour_file(out_file_name_);
			std::string line;
			while(tour_file >> line) {
				if (line == "TOUR_SECTION")
					break;
			}
			size_t vertex, start_vertex = kNIL;
			bool assign_start_vertex = false;
			for(size_t i = 0; i < num_cc_; ++i) {
				tour_file >> vertex;
				if(assign_start_vertex == false) {
					start_vertex = vertex;
					assign_start_vertex = true;
				}
				path_.push_back(vertex - 1);
			}
			path_.push_back(start_vertex - 1);
			tour_file.close();
			std::cout << "Path size: " << path_.size() << " " << num_cc_<< std::endl;
		}

		public:
		GTSP_GLKH(const size_t n, const size_t num_cc, const std::vector < std::vector <double> > &d, std::vector <size_t> &vertex_cc) : n_{n}, num_cc_{num_cc}, d_{d}, vertex_cc_{vertex_cc} {
			path_.reserve(n_);
			clusters_.resize(num_cc_);
			SolveGTSP();
		}

		void SolveGTSP() {
			for (size_t i = 0; i < n_; ++i) {
				clusters_[vertex_cc_[i]].push_back(i);
			}
			std::string glkh_path = LCLIBRARY_GLKH_PATH;
			write_dir_ = glkh_path + "TMP";
			parameter_file_name_ = write_dir_ + "/gtsp.par";
			problem_file_name_ = write_dir_ + "/gtsp.in";
			out_file_name_ = write_dir_ + "/gtsp.out";
			pi_file_name_ = write_dir_ + "/gtsp.pi";
			WriteParameterFile();
			WriteProblemFile();
			std::string command_string = "glkh.sh " + glkh_path + " " + parameter_file_name_;
			std::cout << command_string << std::endl;
			const char *command_char = command_string.c_str();
			auto glkh_exit = system(command_char);
			if(glkh_exit != 0) {
				std::cerr << "GLKH failed\n";
			}
			ReadTourFile();
			std::cout << "write_dir_: " << write_dir_ << std::endl;
			std::filesystem::remove(parameter_file_name_);
			std::filesystem::remove(problem_file_name_);
			std::filesystem::remove(out_file_name_);
			std::filesystem::remove(pi_file_name_);
		}

		int GetPath(std::vector < size_t > &path) const {
			path = path_;
			return 0;
		}

	};

} // namespace lclibrary

#endif /* LCLIBRARY_USE_GLKH */
#endif /* LCLIBRARY_ALGORITHMS_GTSP_GLKH_H_ */

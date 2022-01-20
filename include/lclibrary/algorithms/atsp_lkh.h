/**
 * This file is part of the LineCoverage-library.
 * The file contains interfacing class for solving ATSP using LKH
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

#ifndef LCLIBRARY_ALGORITHMS_ATSP_LKH_H_
#define LCLIBRARY_ALGORITHMS_ATSP_LKH_H_

#include <lclibrary/core/constants.h>
#include <lclibrary/algorithms/atsp_base.h>
#include <lclibrary/utils/create_temp_dir.h>

namespace lclibrary {

	class ATSP_LKH: public ATSP {
		size_t n_;
		const std::vector < std::vector <double> > &d_;
		double cost_;
		std::vector <size_t> path_;
		std::string parameter_file_name_;
		std::string problem_file_name_;
		std::string out_file_name_;

		void WriteParameterFile() const {
			std::ofstream parameter_file(parameter_file_name_);
			parameter_file << "PROBLEM_FILE = " + problem_file_name_ + "\n";
			parameter_file << "TOUR_FILE = " + out_file_name_ + "\n";
			parameter_file << "RUNS = 1\n";
			parameter_file << "TRACE_LEVEL = 0\n";
			parameter_file.close();
		}

		void WriteProblemFile() const {
			std::ofstream problem_file(problem_file_name_);
			problem_file << "NAME : atsp\n";
			problem_file << "TYPE : ATSP\n";
			problem_file << "DIMENSION : " << n_ << "\n";
			problem_file << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
			problem_file << "EDGE_WEIGHT_FORMAT: FULL_MATRIX \n";
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
			problem_file.close();
		}

		void ReadTourFile() {
			std::ifstream tour_file(out_file_name_);
			std::string line;
			while(tour_file >> line) {
				if (line == "TOUR_SECTION")
					break;
			}
			size_t vertex, start_vertex = kNIL;
			bool assign_start_vertex = false;
			for(size_t i = 0; i < n_; ++i) {
				tour_file >> vertex;
				if(assign_start_vertex == false) {
					start_vertex = vertex;
					assign_start_vertex = true;
				}
				path_.push_back(vertex - 1);
			}
			path_.push_back(start_vertex - 1);
			tour_file.close();
		}

		public:
		ATSP_LKH(size_t n, const std::vector < std::vector <double> > &d) : n_{n}, d_{d} {
			path_.reserve(n_);
			SolveATSP();
		}

		void SolveATSP() {
			std::string tmp_dir = CreateTempDir().string();
			parameter_file_name_ = tmp_dir + "/atsp.par";
			problem_file_name_ = tmp_dir + "/atsp.in";
			out_file_name_ = tmp_dir + "/atsp.out";
			WriteParameterFile();
			WriteProblemFile();
			std::string command_string = "LKH " + parameter_file_name_;
			std::cout << command_string << std::endl;
			const char *command_char = command_string.c_str();
			auto lkh_exit = system(command_char);
			if(lkh_exit != 0) {
				std::cerr << "LKH failed\n";
			}
			ReadTourFile();
			std::filesystem::remove_all(tmp_dir);
		}

		int GetPath(std::vector < size_t > &path) const {
			path = path_;
			return 0;
		}

	};

} // namespace lclibrary

#endif /* LCLIBRARY_ALGORITHMS_ATSP_LKH_H_ */

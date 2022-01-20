/**
 * This file is part of the LineCoverage-library.
 * The file contains functions for Graph IO
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

#ifndef	LCLIBRARY_CORE_GRAPH_IO_H_
#define	LCLIBRARY_CORE_GRAPH_IO_H_

#include <iostream>
#include <fstream>
#include <memory>
#include <lclibrary/core/vertex.h>
#include <lclibrary/core/edge.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/algorithms/required_graph.h>

namespace lclibrary {

	/*! Create graph with only required edges */
	int CreateGraph(
			std::shared_ptr <Graph> &,
			const std::string &,
			const std::string &,
			const bool,
			const bool,
			const bool filter_vertices = false);

	/*! Create graph with required and non-required edges */
	int CreateGraph(
			std::shared_ptr <Graph> &,
			const std::string &,
			const std::string &,
			const std::string &,
			const bool,
			const bool);

	void WriteGraphInfo (std::shared_ptr <const Graph> ,
			const std::string &);

	void WriteNodes(
			std::shared_ptr <const Graph> ,
			const std::string,
			const bool is_with_lla = not kIsWithLLA);

	void WriteRequiredEdges(
			std::shared_ptr <const Graph> ,
			const std::string);

	void WriteNonRequiredEdges(
			std::shared_ptr <const Graph> ,
			const std::string);

	void VertexParser (
			std::vector <Vertex> &,
			const std::ifstream &,
			const bool is_with_lla = kIsWithLLA);

	void FileParser (
			std::shared_ptr<Graph> &,
			std::ifstream &,
			std::ifstream &,
			const bool,
			const bool,
			const bool filter_vertices = false);

	void FileParser (
			std::shared_ptr<Graph> &,
			std::ifstream &,
			std::ifstream &,
			std::ifstream &,
			const bool,
			const bool,
			const bool filter_vertices = false);

	void EdgeParser (
			std::vector <Edge> &,
			std::ifstream &, const bool, size_t &);

	void EdgeParser (
			std::vector <Edge> &,
			std::ifstream &,
			std::ifstream &,
			const bool,
			size_t &,
			size_t &);

	void DepotListParser(
			const std::string &,
			std::vector <size_t> &);
}
#endif /* LCLIBRARY_CORE_GRAPH_IO_H_ */


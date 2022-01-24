/**
 * This file is part of the LineCoverage-library.
 * Python wrapper for Vertex class in include/lclibrary/core/vertex.h
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

#include <filesystem>
#include <memory>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <lclibrary/core/vertex.h>
#include <lclibrary/core/edge.h>
#include <lclibrary/core/graph.h>
#include <lclibrary/core/route.h>
#include <lclibrary/slc/rpp_3by2.h>
#include <lclibrary/slc/slc_beta2_atsp.h>

using namespace boost::python;
namespace b_np = boost::python::numpy;

void VertexVecFromBnpArray(b_np::ndarray const &np_vertex_ids, std::vector <lclibrary::Vertex> &vertex_list) {
	int *np_vertex_ids_ptr = reinterpret_cast<int*>(np_vertex_ids.get_data());
	int num_vertices = np_vertex_ids.shape(0);
	vertex_list.reserve(num_vertices);
	for(int i = 0; i < num_vertices; ++i) {
		vertex_list.push_back(lclibrary::Vertex(np_vertex_ids_ptr[i]));
	}
}

void EdgeVecFromBnpArray(b_np::ndarray const &edges_tail, b_np::ndarray const &edges_head, b_np::ndarray const &edges_req, b_np::ndarray const &edges_cost, std::vector <lclibrary::Edge> &edge_list) {
	int* edges_tail_ptr = reinterpret_cast<int*>(edges_tail.get_data());
	int* edges_head_ptr = reinterpret_cast<int*>(edges_head.get_data());
	bool* edges_req_ptr = reinterpret_cast<bool*>(edges_req.get_data());
	double* edges_cost_ptr = reinterpret_cast<double*>(edges_cost.get_data());
	int num_edges = edges_cost.shape(0);
	edge_list.reserve(num_edges);
	for(int i = 0; i < num_edges; ++i) {
		edge_list.push_back(lclibrary::Edge(edges_tail_ptr[i], edges_head_ptr[i], edges_req_ptr[i]));
		edge_list.back().SetCosts(edges_cost_ptr[i]);
	}
}

b_np::ndarray SolveRPP(b_np::ndarray const &vertex_ids, b_np::ndarray const &edges_tail, b_np::ndarray const &edges_head, b_np::ndarray const &edges_req, b_np::ndarray const &edges_cost) {
	std::vector <lclibrary::Vertex> vertices;
	std::vector <lclibrary::Edge> edge_list;
	VertexVecFromBnpArray(vertex_ids, vertices);
	EdgeVecFromBnpArray(edges_tail, edges_head, edges_req, edges_cost, edge_list);
	auto graph = std::make_shared <lclibrary::Graph>(vertices, edge_list);
	lclibrary::SLC_RPP slc_rpp(graph);
	slc_rpp.Solve();
	lclibrary::Route route;
	slc_rpp.GetRoute(route);
	boost::python::tuple shape = boost::python::make_tuple(route.GetRouteLength(), 3);
	b_np::dtype dtype = b_np::dtype::get_builtin<int>();
	b_np::ndarray np_edges = b_np::zeros(shape, dtype);
	shape = boost::python::make_tuple(route.GetRouteLength());
	dtype = b_np::dtype::get_builtin<double>();
	int edge_count = 0;
	for(auto route_it = route.GetRouteStart(); route_it != route.GetRouteEnd(); ++route_it) {
		auto e = *route_it;
		np_edges[edge_count][0] = int(e.GetTailVertexID());
		np_edges[edge_count][1] = int(e.GetHeadVertexID());
		np_edges[edge_count][2] = int(e.GetReq());
		++edge_count;
	}
	return np_edges;

}

b_np::ndarray SolveSLC_Beta2_atsp(b_np::ndarray const &vertex_ids, b_np::ndarray const &edges_tail, b_np::ndarray const &edges_head, b_np::ndarray const &edges_req, b_np::ndarray const &edges_cost) {
	std::vector <lclibrary::Vertex> vertices;
	std::vector <lclibrary::Edge> edge_list;
	VertexVecFromBnpArray(vertex_ids, vertices);
	EdgeVecFromBnpArray(edges_tail, edges_head, edges_req, edges_cost, edge_list);
	auto graph = std::make_shared <lclibrary::Graph>(vertices, edge_list);
	lclibrary::SLC_Beta2ATSP slc_solver(graph);
	slc_solver.Use2Opt(false);
	auto solver_status = slc_solver.Solve();
	if(solver_status) {
		std::cerr << "SLC Beta2 ATSP failed\n";
	}
	lclibrary::Route route;
	slc_solver.GetRoute(route);
	boost::python::tuple shape = boost::python::make_tuple(route.GetRouteLength(), 3);
	b_np::dtype dtype = b_np::dtype::get_builtin<int>();
	b_np::ndarray np_edges = b_np::zeros(shape, dtype);
	shape = boost::python::make_tuple(route.GetRouteLength());
	dtype = b_np::dtype::get_builtin<double>();
	int edge_count = 0;
	for(auto route_it = route.GetRouteStart(); route_it != route.GetRouteEnd(); ++route_it) {
		auto e = *route_it;
		np_edges[edge_count][0] = int(e.GetTailVertexID());
		np_edges[edge_count][1] = int(e.GetHeadVertexID());
		np_edges[edge_count][2] = int(e.GetReq());
		++edge_count;
	}
	return np_edges;

}

BOOST_PYTHON_MODULE(py_lclibrary) {
	Py_Initialize();
	b_np::initialize();
	class_<lclibrary::Vertex> ("Vertex", init <size_t> ())
		.def("GetID", &lclibrary::Vertex::GetID)
		.def("SetID", &lclibrary::Vertex::SetID);

	class_<lclibrary::Edge> ("Edge", init <size_t, size_t>())
		.def("GetCost", &lclibrary::Edge::GetCost)
		.def("SetCosts", &lclibrary::Edge::SetCosts)
		.def("GetTailVertexID", &lclibrary::Edge::GetTailVertexID)
		.def("GetHeadVertexID", &lclibrary::Edge::GetHeadVertexID);

	def("SolveRPP", &SolveRPP);
}

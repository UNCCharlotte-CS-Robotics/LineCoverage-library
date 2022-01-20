/**
 * This file is part of the LineCoverage-library.
 * The following Vertex Class is for creating, deleting and modifying vertices in a Graph.
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

#ifndef LCLIBRARY_CORE_VERTEX_H_
#define LCLIBRARY_CORE_VERTEX_H_

#include <lclibrary/core/constants.h>
#include <lclibrary/core/vec2d.h>

#include <iostream>
#include <list>

namespace lclibrary {

	class Edge;
	typedef std::list <const Edge *> AdjacentEdgeList;

	class Vertex {
		size_t id_; /*! Actual node ID of the Vertex */
		AdjacentEdgeList adjacent_edges_list_; /*! Stores adjacent vertices of the Vertex */
		double lla_[3]; /*! Latitude, longitude, altitude */
		Vec2d xy_; /*! XY coordinate */

		public:

		/*! Constructor: takes in id_ of the Vertex */
		Vertex(size_t id_i) : id_{id_i} { SetLLA(0, 0, 0); SetXY(0, 0); }

		Vertex() : Vertex(0) {}

		/*! Copy constructor (does not copy adjacency list)*/
		Vertex(const Vertex &v){
			CopyDataFromVertex(v);
		}

		void operator = (const Vertex &v ) {
			CopyDataFromVertex(v);
		}

		void CopyDataFromVertex(const Vertex &v) {
			auto v_xy = v.GetXY();
			xy_.x = v_xy.x; xy_.y = v_xy.y;
			double v_lla[3];
			v.GetLLA(v_lla);
			for(size_t i = 0; i < 3; ++i){
				lla_[i] = v_lla[i];
			}
			id_ = v.GetID();
		}

		void SetLLA(const double lat, const double lon, const double alt) {
			lla_[0] = lat; lla_[1] = lon; lla_[2] = alt;
		}

		void SetXY(const double X, const double Y) { xy_.x = X; xy_.y = Y; }

		Vec2d GetXY() const{ return xy_; }

		void PrintXY() const{ std::cout << id_ <<" " << xy_.x << " " << xy_.y << std::endl; }

		void GetLLA(double * v_lla) const{
			for(size_t i = 0; i < 3; ++i){
				v_lla[i] = lla_[i];
			}
		}

		/*! Adds adjacent edge/arc */
		void AddEdge(const Edge *e) {
			adjacent_edges_list_.push_back(e);
		}

		/*! Clears adjacent edge list */
		void ClearAdjacentList(){
			adjacent_edges_list_.clear();
		}

		AdjacentEdgeList::const_iterator GetAdjacencyStart() const{
			return adjacent_edges_list_.cbegin();
		}

		bool IsAdjacencyEnd(const AdjacentEdgeList::const_iterator it) const {
			if (it == adjacent_edges_list_.cend())
				return kFail;
			else
				return kSuccess;
		}

		bool GetAdjacentEdge(const Edge *&e, const AdjacentEdgeList::const_iterator it) const {
			if(it == adjacent_edges_list_.end()) {
				e = nullptr;
				return kFail;
			}
			e = *it;
			return kSuccess;
		}

		/*! Returns size of adjacent list */
		size_t GetAdjListSize() const{ return adjacent_edges_list_.size(); }

		size_t GetID() const { return id_; }

		void SetID(const size_t a){ id_ = a; }

	};

} // namespace lclibrary

#endif /* LCLIBRARY_CORE_VERTEX_H_ */

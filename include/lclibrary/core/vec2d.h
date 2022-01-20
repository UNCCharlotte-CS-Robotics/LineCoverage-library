/**
 * This file is part of the LineCoverage-library.
 * The following Vec2d Class is for 2D-vector based computations
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

#ifndef LCLIBRARY_CORE_VEC2D_H_
#define LCLIBRARY_CORE_VEC2D_H_

#include <lclibrary/core/math_utils.h>
#include <lclibrary/core/constants.h>
#include <cmath>
#include <ostream>

namespace lclibrary {

	class Vec2d {
		public:
			double x;
			double y;
			Vec2d(): x{0.0}, y{0.0}{}
			Vec2d(const double x_i, const double y_i): x{x_i}, y{y_i}{}
			Vec2d (const double p1[2], const double p2[2]){
				x = p2[0] - p1[0];
				y = p2[1] - p1[1];
			}

			/*! Computes perpendicular Vector */
			Vec2d Perpendicular() const{
				Vec2d v_perpendicular;
				v_perpendicular.x = -y;
				v_perpendicular.y = x;
				return v_perpendicular;
			}

			/*! Adds two vectors */
			void Add(Vec2d const &v) {
				x += v.x; y += v.y;
			}

			/*! Divide vector by a scalar */
			int Divide(const double scalar) {
				if(std::abs(scalar) < kEps) {
					return kFail;
				}
				x = x/scalar; y = y/scalar;
				return kSuccess;
			}

			/*! Computes dot product of two Vectors */
			double Dot(Vec2d const &v) const{
				return v.x * x + v.y * y;
			}

			/*! Returns square of Euclidean distance from origin */
			double NormSqr() const{
				return x * x + y *y;
			}

			/*! Returns Euclidean distance from origin */
			double Norm() const{
				return std::sqrt(NormSqr());
			}

			/*! Gives cosine of the angle between this and Vector v */
			int CosAngle(Vec2d const &v, double &ang) const{
				if (std::abs(Norm()) < 1e-10 || std::abs(v.Norm()) < 1e-10)
					return kFail;
				ang = Dot(v)/(Norm() * v.Norm());
				return kSuccess;
			}

			/*! Gives the distance between the Vector and another Vector v */
			double DistSqr(Vec2d const &v) const{
				double del_x = x - v.x;
				double del_y = y - v.y;
				double dSqr = (del_x * del_x + del_y * del_y);
				return dSqr;
			}

			double Dist(Vec2d const &v) const {
				return std::sqrt(DistSqr(v));
			}

			/*! Computes distance and angle with another Vector (v-this)*/
			void DistTht(Vec2d const &v, double &d, double &tht) const{
				d = std::sqrt(DistSqr(v));
				double del_x = -x + v.x;
				double del_y = -y + v.y;
				tht = std::atan2(del_y, del_x);
				if(tht < 0)
					tht += M_PI;
			}

			bool IsZero() const {
				if(IsNearZero(x) and IsNearZero(y)) {
					return true;
				}
				return false;
			}

			/*! Gives the direction of travel Vector */
			Vec2d TravelVec(Vec2d const &v) const{
				Vec2d travel_vec;
				travel_vec.x = -x + v.x;
				travel_vec.y = -y + v.y;
				return travel_vec;
			}

			Vec2d operator+ (const Vec2d &vec){
				return Vec2d(x + vec.x, y + vec.y);
			}

			Vec2d operator- (const Vec2d &vec){
				return Vec2d(x - vec.x, y - vec.y);
			}

			Vec2d operator- (){
				return Vec2d(-x, -y);
			}

			Vec2d operator/ (const double &scalar){
				return Vec2d(x / scalar, y / scalar);
			}

			Vec2d operator* (const double &scalar){
				return Vec2d(x * scalar, y * scalar);
			}

			int Normalize () {
				double norm = Norm();
				if(norm < kEps) {
					return kFail;
				}
				x = x/norm;
				y = y/norm;
				return kSuccess;
			}
	};

	inline std::ostream& operator<<(std::ostream &ostream_obj, const Vec2d &vec) {
		ostream_obj << vec.x << " " << vec.y;
		return ostream_obj;
	}

} // namespace lclibrary
#endif /* LCLIBRARY_CORE_VEC2D_H_ */

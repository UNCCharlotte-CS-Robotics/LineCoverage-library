/**
 * This file is part of the LineCoverage-library.
 * The file contains class to convert LLA to XY and vice versa
 *
 * TODO:
 * Use geodesy library?
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

#ifndef LCLIBRARY_CORE_TRANSFORM_LLA_XY_H_
#define LCLIBRARY_CORE_TRANSFORM_LLA_XY_H_

#include <cmath>
#include <unordered_map>
#include <fstream>
#include <iostream>

#define WGS84_EquitorialRadius 6378137.000000
#define WGS84_PolarSemiMinorAxis 6356752.314245
#define WGS84_flattening 0.003352810664
/* 
input:
refLat,refLon : geodetic coordinate which you defined as 0,0 in cartesian coordinate (the unit is in radian)

lat,lon : geodetic coordinate which you want to calculate its cartesian coordinate (the unit is in radian)

xOffset,yOffset : the result in cartesian coordinate x,y (the unit is in meters)

*/
namespace lclibrary{

	class LLAtoXY {
		private:
			double a, b, f;
			double refLat, refLon, refAlt;
			double r_n, r_m;
		public:
			inline double degTorad (double deg){
				return M_PI * deg / 180.0;
			}
			inline double radTodeg (double rad){
				return 180.0 * rad / M_PI;
			}

			LLAtoXY() {};
			LLAtoXY(std::string nodeFileName, std::string outNodeFileName){
				parse_write(nodeFileName, outNodeFileName);
			}

			void GetRefLatLong(double &rlat, double &rlon, double &ralt) {
				rlat = refLat;
				rlon = refLon;
				ralt = refAlt;
			}

			void SetRefLatLong(const double rlat, const double rlon, const double ralt) {
				refLat = degTorad(rlat);
				refLon = degTorad(rlon);
				refAlt = ralt;
				double R = WGS84_EquitorialRadius;
				double sinMu0 = sin(refLat);
				double sinSqMu0 = sinMu0 * sinMu0;

				double f = WGS84_flattening;
				double r_m_deno = 1.0 - (2.0 * f - f * f) * sinSqMu0;
				double r_n_deno = std::sqrt(r_m_deno);
				r_n = R / r_n_deno;
				r_m = r_n * (1.0 - (2.0 * f - f * f)) / r_m_deno;
			}

			void llaToFlat(double lat, double lon, double alt, double& xOffset, double& yOffset, double& zOffset) {

				double R = WGS84_EquitorialRadius;
				double sinMu0 = sin(refLat);
				double sinSqMu0 = sinMu0 * sinMu0;

				double f = WGS84_flattening;
				double r_m_deno = 1.0 - (2.0 * f - f * f) * sinSqMu0;
				double r_n_deno = std::sqrt(r_m_deno);
				r_n = R / r_n_deno;
				r_m = r_n * (1.0 - (2.0 * f - f * f)) / r_m_deno;
				double dMu = degTorad(lat) - refLat;
				double dL = degTorad(lon) - refLon;
				yOffset = dMu/(atan(1.0/r_m));
				xOffset = dL/(atan(1.0/(r_n * cos (refLat))));
				zOffset = alt - refAlt;
			}
			void flatToLLA(double xOffset, double yOffset, double zOffset, double &lat, double &lon, double &alt) {
				double dMu = yOffset * (atan(1.0/r_m));
				double dL = xOffset * (atan(1.0/(r_n * cos (refLat))));
				lat = radTodeg(dMu + refLat);
				lon = radTodeg(dL + refLon);
				alt = zOffset - refAlt;
			}

			void parse_write(std::string nodeFileName, std::string outNodeFileName){
				std::fstream nodeInFile;
				nodeInFile.open(nodeFileName);
				if (!nodeInFile) {
					std::cerr << "Unable to open file " << nodeFileName << "\n";
					exit(1);   // call system to stop
				}

				std::ofstream nodeOutFile;
				nodeOutFile.open(outNodeFileName);
				if (!nodeOutFile) {
					std::cerr << "Unable to open file " << outNodeFileName << "\n";
					exit(1);   // call system to stop
				}
				nodeOutFile.precision(16);
				double dummyAlt = 229;

				std::unordered_map <size_t, size_t> nodeMap;
				size_t nodeID;
				double nodeLat, nodeLon;
				size_t countNode = 0;
				bool flag = 0;
				while (nodeInFile >> nodeID){
					nodeMap[nodeID] = countNode++;
					nodeInFile >> nodeLat;
					nodeInFile >> nodeLon;
					if ( !flag ){
						flag = 1;
						refLat = degTorad(nodeLat);
						refLon = degTorad(nodeLon);;
						refAlt = dummyAlt;
					}
					double xOffset, yOffset, zOffset;
					llaToFlat(nodeLat, nodeLon, dummyAlt + 50.0, xOffset, yOffset, zOffset);
					nodeOutFile << nodeID << " " << xOffset << " " << yOffset << " " << nodeLat << " " << nodeLon << " " << zOffset << "\n";

				}
				nodeInFile.close();
				nodeOutFile.close();

			}

	};
} // namespace lclibrary

#endif /* LCLIBRARY_CORE_TRANSFORM_LLA_XY_H_ */

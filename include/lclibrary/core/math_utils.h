/**
 * This file is part of the LineCoverage-library.
 * The file contains math utility functions
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

/**
 * TODO: Add more helpful functions instead of spilling all over the library
 */

#ifndef LCLIBRARY_UTILS_MATH_UTILS_H_
#define LCLIBRARY_UTILS_MATH_UTILS_H_

#include <lclibrary/core/constants.h>
#include <cmath>

namespace lclibrary {

	inline bool IsNearZero(double const x, double const eps = kEps) {
		if(std::abs(x) < eps)
			return true;
		else
			return false;
	}

} /* lclibrary */
#endif /*  LCLIBRARY_UTILS_MATH_UTILS_H_ */

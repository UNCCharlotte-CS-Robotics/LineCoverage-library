/**
 * This file is part of the LineCoverage-library.
 * The file contains functions to generate video frames using gnuplot
 *
 * TODO:	The implementation is incomplete
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

#ifndef LCLIBRARY_UTILS_VIDEOGENERATOR_H_
#define LCLIBRARY_UTILS_VIDEOGENERATOR_H_

#include <lclibrary/core/graph.h>
#include <lclibrary/core/route.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <memory>

namespace lclibrary {

	class VideoGenerator {
		std::shared_ptr <const Graph> g_;
		const Route *route_;
		std::string video_dir_;
		size_t num_frames_;
		int scale_order_;
		double scale_;
		double size_x_, size_y_;
		double minX_, maxX_, minY_, maxY_;
		std::string req_filename_, nreq_filename_, req_curr_filename_, nreq_curr_filename_, req_point_filename_, nreq_point_filename_;
		std::string frame_filename_, plot_filename_;

		public:
		VideoGenerator(
				std::shared_ptr <const Graph> g_in,
				const Route *r,
				std::string video_dir,
				size_t frames = 900) :
			g_{g_in},
			route_{r},
			video_dir_{video_dir},
			num_frames_{frames} {

			if(video_dir_.back() != '/')
				video_dir_ += '/';
			g_->GetLimits(minX_, maxX_, minY_, maxY_);
			double ratioXY = (maxY_ - minY_)/(maxX_ - minX_);
			size_x_ = 12;
			size_y_ = ratioXY * size_x_;
			scale_order_ = std::max((int(log10 (maxX_ - minX_))), (int(log10 (maxY_ - minY_))));
			if(scale_order_ < 1)
				scale_order_ = 1;
			scale_ = std::pow(10, scale_order_);
			req_filename_ = video_dir_ + "req";
			nreq_filename_ = video_dir_ + "nreq";
			req_curr_filename_ = video_dir_ + "req_curr";
			nreq_curr_filename_ = video_dir_ + "nreq_curr";
			req_point_filename_ = video_dir_ + "req_point";
			nreq_point_filename_ = video_dir_ + "nreq_point";
			frame_filename_ = video_dir_ + "frames/f";
			plot_filename_ = video_dir_ + "plot.gp";
			GenerateVideo();

		}
		void GnuplotMap(
				const std::string,
				bool curr_req = true,
				bool req = true,
				bool non_req = false,
				bool at_end = false);

		void GenerateVideo();

	};

} /* lclibrary */
#endif /*  LCLIBRARY_UTILS_VIDEOGENERATOR_H_*/

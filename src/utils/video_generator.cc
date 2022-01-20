/**
 * This file is part of the LineCoverage-library.
 * The file contains function definitions to generate video frames using gnuplot
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

#include <lclibrary/utils/video_generator.h>
#include <cmath>

namespace lclibrary {

	void VideoGenerator::GnuplotMap(const std::string output_plot_file_name, bool curr_req, bool req, bool non_req,bool at_end) {
		std::ofstream out_file(plot_filename_);
		out_file.precision(16);
		out_file << "set terminal pdfcairo enhanced font 'Times,14' size " << size_x_ <<"cm, " << size_y_ << "cm crop\n";
		out_file << "set o \"" << output_plot_file_name << "\"\n";
		out_file << "unset key\nunset colorbox\nunset grid\n";
		out_file << "set cbrange [0:19]\n";
		out_file << "set xrange [" << minX_/scale_ - 0.05 << ":" << maxX_/scale_ + 0.05 << "]\n";
		out_file << "set yrange [" << minY_/scale_ - 0.05 << ":" << maxY_/scale_ + 0.05 << "]\n";
		out_file << "set palette defined (0 \"#d62728\", 1 \"#1f77b4\", 2 \"#2ca02c\", 3 \"#9467bd\", 4 \"#000000\", 5 \"#8c564b\", 6 \"#e377c2\", 7 \"#ff7f0e\", 8 \"#bcbd22\", 9 \"#17becf\", 10 \"#d69f9f\", 11 \"#89b2cf\", 12 \"#91ca91\", 13 \"#beabcf\", 14 \"#dbdbdb\", 15 \"#bfa6a1\", 16 \"#d5b6cc\", 17 \"#eeb98b\", 18 \"#d5d5a2\", 19 \"#a3ccd1\")\n";
		out_file << "set xlabel \'X-axis (x 10^"<< scale_order_<<" m)\' offset 0,1.0 \n";
		out_file << "set ylabel 'Y-axis (x 10^"<< scale_order_ <<" m)' offset 2.0,0\n";
		out_file << "set size ratio -1\n";
		out_file << "set xtics -10,0.5,10\n";
		out_file << "set ytics -10,0.5,10\n";
		out_file << "set style textbox noborder\n";
		out_file << "set xtics border offset -0.0,0.3\n";
		out_file << "plot ";
		if (req) {
			out_file << "\"" << req_filename_ << "\" u ($1/" << scale_ << "):($2/" << scale_ <<"):3  w lines lw 1 palette, ";
		}
		if (non_req)
			out_file << "\"" << nreq_filename_ << "\" u ($1/"<<scale_<<"):($2/"<<scale_ <<"):3  w lines lw 1 dashtype 2 palette, ";

		if(curr_req) {
			if(!at_end)
				out_file << "\"" << req_curr_filename_ << "\" u ($1/"<<scale_<<"):($2/"<<scale_ <<"):3  w lines lw 1 palette,";
			out_file << "\"" << req_point_filename_ << "\" u ($1/"<<scale_<<"):($2/"<<scale_ <<"):3  w points pt 7 ps .4 palette\n";
		}
		else {
			if(!at_end)
				out_file << "\"" << nreq_curr_filename_ << "\" u ($1/"<<scale_<<"):($2/"<<scale_ <<"):3  w lines lw 1 dashtype 2 palette,";
			out_file << "\"" << nreq_point_filename_ << "\" u ($1/"<<scale_<<"):($2/"<<scale_ <<"):3  w points pt 6 ps .4 palette\n ";
		}
		out_file.close();
	}

	void VideoGenerator::GenerateVideo () {
		auto route_it = route_->GetRouteStart();
		/* double step = std::ceil(route_->GetCost()/num_frames_); */
		double step = route_->GetCost()/(0.9 * num_frames_);
		std::cout << "Step size: " << step << std::endl;

		double to = 0;
		double tf = route_->GetEdgeCost(route_it);
		double tc = 0;

		Vec2d po;
		Vec2d pf;
		Vec2d pc;
		route_->GetXY(route_it, po, pf);

		bool at_end = false;
		bool curr_req = true;

		for (size_t iStep = 0; iStep <= num_frames_; ++iStep){
			std::ofstream req_out (req_filename_);
			std::ofstream nreq_out (nreq_filename_);
			req_out.precision(16);
			nreq_out.precision(16);

			std::ofstream req_curr_out (req_curr_filename_);
			std::ofstream nreq_curr_out (nreq_curr_filename_);
			req_curr_out.precision(16);
			nreq_curr_out.precision(16);

			std::ofstream req_point_out (req_point_filename_);
			std::ofstream nreq_point_out (nreq_point_filename_);
			req_point_out.precision(16);
			nreq_point_out.precision(16);

			tc = step * iStep;
			if(tc > tf) {
				to = tf;
				if(route_it != route_->GetRouteEnd())
					++route_it;
				if(route_it == route_->GetRouteEnd()) {
					at_end = true;
					pc = pf;
				}
				else {
					tf = to + route_->GetEdgeCost(route_it);
					route_->GetXY(route_it, po, pf);
				}
			}
			else if(tc > to) {
				pc.x = (po.x * (tc - tf) + pf.x * ( to - tc  )) / (to - tf);
				pc.y = (po.y * (tc - tf) + pf.y * ( to - tc  )) / (to - tf);
			}
			/* else { */
			/* 	pc = po; */
			/* } */

			bool req = false;
			bool non_req = false;
			for(auto it = route_->GetRouteStart(); it != route_it; ++it) {
				Vec2d xyo, xyf;
				route_->GetXY(it, xyo, xyf);
				if (route_->IsReqEdge(it)) {
					req = true;
					req_out << xyo.x << " " << xyo.y << " " << "1" << std::endl;
					req_out << xyf.x << " " << xyf.y << " " << "1" << std::endl;
					req_out << std::endl;
				}
				else {
					non_req = true;
					nreq_out << xyo.x << " " << xyo.y << " " << "2" << std::endl;
					nreq_out << xyf.x << " " << xyf.y << " " << "2" << std::endl;
					nreq_out << std::endl;
				}
			}

			if(!at_end)
				curr_req = route_->IsReqEdge(route_it);
			if (curr_req) {
				req_point_out << pc.x << " " << pc.y << " " << "1" << std::endl;
				if(!at_end) {
					curr_req = route_->IsReqEdge(route_it);
					req_curr_out << po.x << " " << po.y << " " << "1" << std::endl;
					req_curr_out << pc.x << " " << pc.y << " " << "1" << std::endl;
					req_curr_out << std::endl;
				}
			}
			else {
				nreq_point_out << pc.x << " " << pc.y << " " << "2" << std::endl;
				if(!at_end) {
					nreq_curr_out << po.x << " " << po.y << " " << "2" << std::endl;
					nreq_curr_out << pc.x << " " << pc.y << " " << "2" << std::endl;
					nreq_curr_out << std::endl;
				}
			}

			req_out.close();
			nreq_out.close();
			req_curr_out.close();
			nreq_curr_out.close();
			req_point_out.close();
			nreq_point_out.close();

			std::stringstream ss;
			ss << std::setw(4) << std::setfill('0') << iStep;
			std::string s = ss.str();

			GnuplotMap(frame_filename_ + s + ".pdf", curr_req, req, non_req, at_end);

			std::string systemCall = "gnuplot " + plot_filename_;
			char* cstr = new char [systemCall.size() + 1];
			strcpy(cstr, systemCall.c_str());	// or pass &s[0]
			auto ret = system(cstr);
			if (ret) {
				std::cout<<ret <<std::endl;
			}
			delete [] cstr;
		}

	}

} /* lclibrary */

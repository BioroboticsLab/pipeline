/*
 * fill_convex_poly_tests.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: tobias
 */

#include "fill_convex_poly_tests.h"

namespace heyho {

	namespace tests {

		void benchmark_fill_convex_poly_functions(const std::vector<fill_convex_poly_f> &fill_functions, int major, size_t times)
		{
			constexpr int img_type = CV_8UC1;
			const int axis_major = major;
			const int axis_minor = axis_major / 2;
			const int dim_y = 2 * std::max(axis_major, axis_minor) + 10;
			const int dim_x = dim_y;
			const cv::Point center(dim_x/2, dim_y/2);
			std::vector<cv::Point> poly;
			cv::ellipse2Poly(center, cv::Size(axis_major, axis_minor), 45, 0, 360, 1, poly);
			cv::Mat img(dim_y, dim_x, img_type, white<img_type>());

			std::cout << "benchmark multiple functions (" << heyho::img_type_to_str(img.type()) << ") :\n";

			for (size_t i = 0; i < fill_functions.size(); ++i) {
				std::cout << "  function "<< std::setw(2) << i << ": ";
				std::cout.flush();
				timer t;
				for (size_t j = 0; j < times; ++j) {
					fill_functions[i](img, poly, default_color<img_type>(), 4);
				}
			}
		}

		void compare_convex_poly()
		{
			std::cout << "FOREACH( cv::fillConvexPoly == heyho::fill_convex_poly ) ... ";
			std::cout.flush();
			for (int a = 0; a < 45; a += 5) {
				for (int a1 = 25; a1 < 50; ++a1) {
					for (int a2 = a1; a2 < 50; ++a2) {
						foreach()(compare{{a1, a2}, a});
					}
				}
			}
			std::cout << "passed :)\n";
		}

	}
}

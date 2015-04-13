/*
 * fill_convex_poly_tests.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: tobias
 */

#include "fill_convex_poly_tests.h"
#include <iomanip>
#include <algorithm>

namespace heyho {

	namespace tests {

		void benchmark_fill_convex_poly_functions(const std::vector<std::pair<std::string, fill_convex_poly_f>> &fill_functions, int major, size_t times)
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

			std::cout << "benchmark multiple fill functions (" << dim_x << " x " << dim_y << ", " << heyho::img_type_to_str(img.type()) << "), " << times << " times :\n";

			size_t fill = 0;
			for (const auto &name_f : fill_functions) {
				fill = std::max(fill, name_f.first.size());
			}
			++fill;
			for (const auto &name_f : fill_functions) {
				std::cout << "  " << std::left << std::setw(static_cast<int>(fill)) << name_f.first << ": " << std::right;
				std::cout.flush();
				timer t;
				for (size_t j = 0; j < times; ++j) {
					name_f.second(img, poly, default_color<img_type>(), connectivity::eight_connected);
				}
			}
		}

		void benchmark_count_convex_poly_functions(const std::vector<std::pair<std::string, count_convex_poly_f>> &count_functions, int major, size_t times)
		{
			constexpr int img_type = CV_8UC1;
			const int axis_major = major;
			const int axis_minor = axis_major / 2;
			const int dim_y = 2 * std::max(axis_major, axis_minor) + 10;
			const int dim_x = dim_y;
			const cv::Point center(dim_x/2, dim_y/2);
			std::vector<cv::Point> poly;
			cv::ellipse2Poly(center, cv::Size(axis_major, axis_minor), 45, 0, 360, 1, poly);

			cv::Mat img(dim_y, dim_x, img_type);
			{
				uchar val = 0;
				for(auto ptr = img.datastart; ptr < img.dataend; ++ptr) {
					*ptr = val;
					++val;
				}
			}

			std::cout << "benchmark multiple count functions (" << dim_x << " x " << dim_y << ", " << heyho::img_type_to_str(img.type()) << "), " << times << " times :\n";

			size_t fill = 0;
			for (const auto &name_f : count_functions) {
				fill = std::max(fill, name_f.first.size());
			}
			++fill;
			for (const auto &name_f : count_functions) {
				std::cout << "  " << std::left << std::setw(static_cast<int>(fill)) << name_f.first << ": " << std::right;
				std::cout.flush();
				timer t;
				for (size_t j = 0; j < times; ++j) {
					name_f.second(img, poly, connectivity::eight_connected);
				}
			}
		}

		template<typename A, typename B>
		void compare_convex_poly()
		{
			using cmp = compare_paint<A, B>;

			{
				std::cout << "FOREACH( " << A::name() << " == " << B::name() << " ) ... ";
				std::cout.flush();
				for (int angle = 0; angle < 45; angle += 5) {
					for (int axis_minor = 25; axis_minor < 50; ++axis_minor) {
						for (int axis_major = axis_minor; axis_major < 50; ++axis_major) {
							foreach()(cmp{{axis_minor, axis_major}, angle});
						}
					}
				}
				std::cout << "passed :)\n";
			}

			{
				std::cout << "FOREACH( cropped " << A::name() << " == cropped " << B::name() << " ) ... ";
				std::cout.flush();

				const cv::Size img_dim(100, 100);

				// crossing image boundaries
				// =========================

				// left
				foreach()(cmp{{50, 10}, 0, img_dim, {10, 50}});

				// right
				foreach()(cmp{{50, 10}, 0, img_dim, {90, 50}});

				// top
				foreach()(cmp{{10, 50}, 0, img_dim, {50, 10}});

				// bottom
				foreach()(cmp{{10, 50}, 0, img_dim, {50, 90}});

				// all sides
				foreach()(cmp{{55, 55}, 0, img_dim, {50, 50}});


				// completly outside
				// =================

				// left
				foreach()(cmp{{50, 10}, 0, img_dim, {-55, 50}});

				// right
				foreach()(cmp{{50, 10}, 0, img_dim, {105, 50}});

				// top
				foreach()(cmp{{10, 50}, 0, img_dim, {50, -55}});

				// bottom
				foreach()(cmp{{10, 50}, 0, img_dim, {50, 105}});

				std::cout << "passed :)\n";
			}
		}

		void compare_convex_poly() {
			compare_convex_poly<
				heyho::tests::open_cv_fill_poly_f,
				heyho::tests::heyho_fill_poly_cv_f<line_iterator_cv>
			>();
		}

	}
}

/*
 * fill_convex_poly_tests.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: tobias
 */

#include "../../../util/opencv_fill_poly/fill_convex_poly_tests.h"
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
				for(auto ptr = img.data; ptr < img.dataend; ++ptr) {
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
				std::cout << "ellipse FOREACH( " << A::name() << " == " << B::name() << " ) ... ";
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
				std::cout << "ellipse FOREACH( cropped " << A::name() << " == cropped " << B::name() << " ) ... ";
				std::cout.flush();

				const cv::Size img_dim(100, 100);

				// ellipse crossing image boundaries
				// =================================

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


				// ellipse completly outside
				// =========================

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

			{
				std::cout << "non-convex polygon FOREACH( cropped " << A::name() << " == cropped " << B::name() << " ) ... ";
				std::cout.flush();

				const auto shift = [](std::vector<cv::Point> pts, cv::Point offset) {
					for (auto &p : pts) { p += offset; }
					return std::move(pts);
				};

				/*
				 *  1 ______ 2
				 *    \    /
				 *     \  /
				 *    6 \/ 3
				 *      /\
				 *     /  \
				 *  5 /____\ 4
				 *
				 */
				const std::vector<cv::Point> poly{{10,10}, {50,10}, {35,30}, {50,50}, {10,50}, {25,30}};
				const cv::Size img_dim(60, 60);
				const cv::Point offset_x(20, 0);
				const cv::Point offset_y(0, 20);

				// non-convex polygon crossing image boundaries
				// ============================================

				// left
				foreach()(cmp{shift(poly, -offset_x), img_dim});

				// right
				foreach()(cmp{shift(poly,  offset_x), img_dim});

				// top
				foreach()(cmp{shift(poly, -offset_y), img_dim});

				// bottom
				foreach()(cmp{shift(poly,  offset_y), img_dim});

				std::cout << "passed :)\n";
			}

			{
				std::cout << "convex polygon FOREACH( " << A::name() << " == " << B::name() << " ) ... ";
				std::cout.flush();

				const cv::Size img_size(60, 60);
				for (int y = 10 + 1; y < 50 - 1; ++y)
				{
					/*
					 * P1             P2
					 * (10,10)________(25,10)
					 *        \       \
					 *         \       \
					 *     P6   \       \ P3
					 *    (35,y)/       / (50,y)
					 *         /       /
					 *  P5    /_______/P4
					 * (10,50)        (25,50)
					 *
					 * loop over y-coords of P3 & P6
					 *
					 */
					const std::vector<cv::Point> poly{{10,10}, {25,10}, {50, y}, {25,50}, {10,50}, {35,y}};
					foreach()(cmp(poly, img_size));
				}
				std::cout << "passed :)\n";
			}

		}

		void compare_convex_poly() {
			compare_convex_poly<
				heyho::tests::open_cv_fill_poly_f,
				heyho::tests::heyho_fill_poly_cv_f<line_iterator_cv>
			>();

			compare_convex_poly<
				heyho::tests::open_cv_fill_poly_f,
				heyho::tests::heyho_fill_poly_cv2_f<line_iterator>
			>();
		}

	}
}

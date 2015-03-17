/*
 * fill_convex_poly_tests.h
 *
 *  Created on: Mar 4, 2015
 *      Author: tobias
 */

#ifndef FILL_CONVEX_POLY_TESTS_H_
#define FILL_CONVEX_POLY_TESTS_H_

#include "cstring"               // memcmp
#include <opencv2/opencv.hpp>    // cv::Size, cv::Mat, cv::Point, ... cv::fillConvexPoly, ellipse2Poly
#include <chrono>                // std::chrono::...
#include <iostream>              // std::cout
#include <iomanip>               // std::setw
#include <vector>                // std::vector
#include <stdexcept>             // std::runtime_error
#include <utility>               // std::pair
#include <string>                // std::string
#include "fill_convex_poly_cv.h" // heyho::fill_convex_poly_cv
#include "fill_convex_poly.h"    // heyho::fill_convex_poly
#include "helper.h"              // heyho::img_type_2_str
#include "fill convex_poly_test_colors.h"
#include "fill_convex_poly_test_functors.h"

namespace heyho {

	namespace tests {

		/**
		 * RAII timer
		 */
		class timer {
		private:
			const std::chrono::system_clock::time_point start;
		public:
			explicit timer()
			: start(std::chrono::system_clock::now())
			{}
			~timer() {
				const std::chrono::system_clock::time_point stop = std::chrono::system_clock::now();
				const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
				std::cout << std::setw(5) << ms.count() << " ms\n";
			}
		};

		/**
		 * compares the results of two fill convex poly functions wrapped into functors
		 * that provide a static paint method
		 *
		 * (e.g. cv::fillConvexPoly wrapped in hey::tests::open_cv_fill_poly_f)
		 *
		 * - displays diffrences
		 *
		 */
		template<typename A, typename B>
		struct compare_paint {

			const cv::Size axes;
			const int angle;
			const int dim_x;
			const int dim_y;
			const cv::Point center;

			compare_paint(cv::Size axes, int angle)
				: axes(axes)
				, angle(angle)
				, dim_x(2 * std::max(axes.width, axes.height) + 10)
				, dim_y(dim_x)
				, center(dim_x / 2, dim_y / 2)
			{}

			compare_paint(cv::Size axes, int angle, int dim_x, int dim_y, cv::Point center)
				: axes(axes)
				, angle(angle)
				, dim_x(dim_x)
				, dim_y(dim_y)
				, center(center)
			{}

			template<typename pixel_t>
			bool operator()(int line_type) const
			{
				constexpr int img_type = cv::DataType<pixel_t>::type;
				std::vector<cv::Point> points;
				cv::ellipse2Poly(center, axes, angle, 0, 360, 1, points);


				cv::Mat img1(dim_y, dim_x, img_type, white<img_type>());
				A::template paint<pixel_t>(img1, points, default_color<img_type>(), line_type);


				cv::Mat img2(dim_y, dim_x, img_type, white<img_type>());
				B::template paint<pixel_t>(img2, points, default_color<img_type>(), line_type);


				const bool equal =  0 == std::memcmp(img1.datastart, img2.datastart, img1.dataend - img1.datastart);

				if (! equal)
				{
					cv::namedWindow( A::name(), cv::WINDOW_AUTOSIZE);
					cv::imshow( A::name(), img1);

					cv::namedWindow( B::name(), cv::WINDOW_AUTOSIZE);
					cv::imshow( B::name(), img2);

					const cv::Mat diff = img1 != img2;

					cv::namedWindow( "diff", cv::WINDOW_AUTOSIZE);
					cv::imshow( "diff", diff);

					cv::waitKey(0);
				}
				return equal;
			}
		};


		/**
		 * benchmarks cv::fillConvexPoly & heyho::fill_convex_poly_cv & heyho::fill_convex_poly
		 */
		struct benchmark {

			const cv::Size axes;
			const int angle;
			std::size_t times;

			template<typename pixel_t>
			bool operator()(int line_type) const
			{
				constexpr int img_type = cv::DataType<pixel_t>::type;
				constexpr int shift = 0;
				const int dim = 2 * std::max(axes.width, axes.height) + 10;
				const cv::Point center(dim / 2, dim / 2);
				std::vector<cv::Point> points;
				cv::ellipse2Poly(center, axes, angle, 0, 360, 1, points);
				cv::Mat img(dim, dim, img_type, white<img_type>());

				std::cout << "img type: " << heyho::img_type_to_str(img_type) << " line type: " << line_type << '\n';
				{
					std::cout << "    opencv:    ";
					timer t;
					for (size_t i = 0; i < times; ++i) {
						cv::fillConvexPoly(img, points, default_color<img_type>(), line_type, shift);
					}
				}
				{
					std::cout << "    heyho_cv:  ";
					timer t;
					for (size_t i = 0; i < times; ++i) {
						heyho::fill_convex_poly_cv<line_iterator_cv, pixel_t>(img, default_color<img_type>(), points, line_type);
					}
				}
				{
					std::cout << "    heyho:     ";
					timer t;
					for (size_t i = 0; i < times; ++i) {
						heyho::fill_convex_poly<pixel_t, line_iterator_cv>(img, points, default_color<img_type>(), line_type);
					}
				}
				return true;
			}
		};

		/**
		 * calls f with each combination of CV image type & CV line type
		 *
		 * @throws iff f returns false
		 *
		 */
		class foreach {
		private:
			template<typename T, typename F>
			void helper(F f) const
			{
				// line_types: {8, 4}
				if (! f.template operator()<T>(8) ) {throw std::runtime_error("");}
				if (! f.template operator()<T>(4) ) {throw std::runtime_error("");}
			}

		public:
			template<typename F>
			void operator()(F f) const
			{
				// img_types:  {CV_8UC1, CV_8SC1, CV_8UC3, CV_32FC1, CV_32FC3}
				this->helper<uint8_t>(f);
				this->helper<int8_t>(f);
				this->helper<cv::Vec<uint8_t, 3>>(f);
				this->helper<float>(f);
				this->helper<cv::Vec<float, 3>>(f);
			}

		};


		/**
		 * benchmarks multiple fill convex poly functions
		 */
		void benchmark_fill_convex_poly_functions( const std::vector<std::pair<std::string, fill_convex_poly_f>>  &fill_functions,  int major, size_t times);

		void benchmark_count_convex_poly_functions(const std::vector<std::pair<std::string, count_convex_poly_f>> &count_functions, int major, size_t times);

		void compare_convex_poly();


	}
}



#endif /* FILL_CONVEX_POLY_TESTS_H_ */

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


		template<int>
		cv::Scalar white();
		template<>
		inline cv::Scalar white<CV_8UC1>() { return {255}; }
		template<>
		inline cv::Scalar white<CV_8SC1>() { return {127}; }
		template<>
		inline cv::Scalar white<CV_8UC3>() { return {255, 255, 255}; }
		template<>
		inline cv::Scalar white<CV_32FC1>() { return {1.0f}; }
		template<>
		inline cv::Scalar white<CV_32FC3>() { return {1.0f, 1.0f, 1.0f}; }


		template<int>
		cv::Scalar default_color();
		template<>
		inline cv::Scalar default_color<CV_8UC1>() { return {127}; }
		template<>
		inline cv::Scalar default_color<CV_8SC1>() { return {-56}; }
		template<>
		inline cv::Scalar default_color<CV_8UC3>() { return {127, 255, 5}; }
		template<>
		inline cv::Scalar default_color<CV_32FC1>() { return {0.4f}; }
		template<>
		inline cv::Scalar default_color<CV_32FC3>() { return {1.0f, 0.0f, 0.2f}; }



		/**
		 * compares cv::fillConvexPoly & heyho::fill_convex_poly_cv
		 *
		 * - displays diffrences
		 *
		 */
		struct compare {

			const cv::Size axes;
			const int angle;
			const int dim_x;
			const int dim_y;
			const cv::Point center;

			compare(cv::Size axes, int angle)
				: axes(axes)
				, angle(angle)
				, dim_x(2 * std::max(axes.width, axes.height) + 10)
				, dim_y(dim_x)
				, center(dim_x / 2, dim_y / 2)
			{}

			compare(cv::Size axes, int angle, int dim_x, int dim_y, cv::Point center)
				: axes(axes)
				, angle(angle)
				, dim_x(dim_x)
				, dim_y(dim_y)
				, center(center)
			{}

			template<typename pixel_t, int line_type>
			bool operator()() const
			{
				constexpr int img_type = cv::DataType<pixel_t>::type;
				constexpr int shift = 0;
				std::vector<cv::Point> points;
				cv::ellipse2Poly(center, axes, angle, 0, 360, 1, points);


				cv::Mat img1(dim_y, dim_x, img_type, white<img_type>());
				cv::fillConvexPoly(img1, points, default_color<img_type>(), line_type, shift);

				cv::Mat img2(dim_y, dim_x, img_type, white<img_type>());
				heyho::fill_convex_poly_cv<pixel_t, line_iterator_cv>(img2, points, default_color<img_type>(), line_type);


				const bool equal =  0 == std::memcmp(img1.datastart, img2.datastart, img1.dataend - img1.datastart);

				if (! equal) {
					cv::namedWindow( "opencv", cv::WINDOW_AUTOSIZE);
					cv::imshow( "opencv", img1);

					cv::namedWindow( "heyho", cv::WINDOW_AUTOSIZE);
					cv::imshow( "heyho", img2);

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

			template<typename pixel_t, int line_type>
			bool operator()() const
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
						heyho::fill_convex_poly_cv<pixel_t, line_iterator_cv>(img, points, default_color<img_type>(), line_type);
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
		struct foreach {

			template<typename F>
			void operator()(F f) const {

				// img_types:  {CV_8UC1, CV_8SC1, CV_8UC3, CV_32FC1, CV_32FC3}
				// line_types: {8, 4}

				using t_8UC1 = uint8_t;
				using t_8SC1 = int8_t;
				using t_8UC3 = cv::Vec<uint8_t, 3>;
				using t_32FC1 = float;
				using t_32FC3 = cv::Vec<float, 3>;

				if (! f.operator()<t_8UC1,  8    >() ) {throw std::runtime_error("");}
				if (! f.operator()<t_8UC1,  4    >() ) {throw std::runtime_error("");}

				if (! f.operator()<t_8SC1,  8    >() ) {throw std::runtime_error("");}
				if (! f.operator()<t_8SC1,  4    >() ) {throw std::runtime_error("");}

				if (! f.operator()<t_8UC3,  8    >() ) {throw std::runtime_error("");}
				if (! f.operator()<t_8UC3,  4    >() ) {throw std::runtime_error("");}

				if (! f.operator()<t_32FC1, 8    >() ) {throw std::runtime_error("");}
				if (! f.operator()<t_32FC1, 4    >() ) {throw std::runtime_error("");}

				if (! f.operator()<t_32FC3, 8    >() ) {throw std::runtime_error("");}
				if (! f.operator()<t_32FC3, 4    >() ) {throw std::runtime_error("");}
			}

		};

		using fill_convex_poly_f = void (*) (cv::InputOutputArray img, cv::InputArray points, const cv::Scalar& color, int line_type);
		using count_convex_poly_f = std::pair<size_t, size_t> (*) (const cv::Mat &img, const std::vector<cv::Point> &points, int line_type);

		inline void cv_fill_convex_poly(cv::InputOutputArray img, cv::InputArray points, const cv::Scalar& color, int line_type) {
			cv::fillConvexPoly(img, points, color, line_type);
		}

		inline std::pair<size_t, size_t> cv_count_convex_poly(const cv::Mat &img, const std::vector<cv::Point> &points, int line_type) {
			cv::Mat poly_img(img.size(), img.type(), cv::Scalar(0));
			cv::fillConvexPoly(poly_img, points, cv::Scalar(255), line_type);
			const size_t all = static_cast<size_t>(cv::countNonZero(poly_img));
			poly_img &= img;
			const size_t non_zero = static_cast<size_t>(cv::countNonZero(poly_img));
			const size_t zero = all - non_zero;
			return {zero, non_zero};
		}

		template<typename LINE_IT>
		inline std::pair<size_t, size_t> heyho_count_convex_poly_cv(const cv::Mat &img, const std::vector<cv::Point> &points, int line_type) {
			const auto counts = heyho::convex_poly_cv<pixel_counter<uchar>, LINE_IT>(img.size(), points, pixel_counter<uchar>{img, 0}, line_type).count();
			return {counts.zero(), counts.non_zero()};
		}

		template<typename LINE_IT>
		inline std::pair<size_t, size_t> heyho_count_convex_poly(const cv::Mat &img, const std::vector<cv::Point> &points, int line_type) {
			const auto counts = heyho::convex_poly<std::vector<cv::Point>::const_iterator, pixel_counter<uchar>, LINE_IT>(img.size(), points.cbegin(), points.cend(), pixel_counter<uchar>{img, 0}, line_type).count();
			return {counts.zero(), counts.non_zero()};
		}

		/**
		 * benchmarks multiple fill convex poly functions
		 */
		void benchmark_fill_convex_poly_functions( const std::vector<std::pair<std::string, fill_convex_poly_f>>   &fill_functions, int major, size_t times);

		void benchmark_count_convex_poly_functions(const std::vector<std::pair<std::string, count_convex_poly_f>> &count_functions, int major, size_t times);

		void compare_convex_poly();


	}
}



#endif /* FILL_CONVEX_POLY_TESTS_H_ */

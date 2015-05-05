/*
 * fill_poly_main.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: tobias
 */



#include <iostream>

#include "fill_convex_poly_cv.h"    // fill_convex_poly
#include "ring_iterator.h"          // ring_iterator_tests
#include "poly_line_iterator.h"     // poly_line_iterator_tests
#include "fill_convex_poly_tests.h" // compare_convex_poly
#include "poly_line_vertical_iterator.h"
#include "fill_convex_poly.h"
#include <opencv2/opencv.hpp>
#include <map>

#include "line_iterator.h"
#include "line_iterator_cv.h"          // line_iterator_tests
#include "helper.h"



int main_impl() {

	{

		std::map<const std::string, std::vector<cv::Point>> m;

		// all points in clockwise order
		//                                   top-left          top-right         bottom-right      bottom-left
		m["top"]    = std::vector<cv::Point>{{10,10}, {50,50}, {90,10},          {90,90},          {10,90}         };
		m["right"]  = std::vector<cv::Point>{{10,10},          {90,10}, {50,50}, {90,90},          {10,90}         };
		m["bottom"] = std::vector<cv::Point>{{10,10},          {90,10},          {90,90}, {50,50}, {10,90}         };
		m["left"]   = std::vector<cv::Point>{{10,10},          {90,10},          {90,90},          {10,90}, {50,50}};

		// test heyho::fillConvexPoly
		if (true and false)
		{
			for (const auto &key_val : m) {
				cv::Mat img(100, 100, CV_8UC1, cv::Scalar(0));
				heyho::fill_convex_poly<heyho::line_iterator_cv, uchar>(img, cv::Scalar(255), key_val.second, heyho::connectivity::eight_connected);
				const auto name = "heyho: " + key_val.first;
				cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
				cv::imshow(name, img);
			}
			cv::waitKey();
		}

		// test cv::fillConvexPoly with invalid inputs
		if (false)
		{
			for (const auto &key_val : m) {
				cv::Mat img(100, 100, CV_8UC1, cv::Scalar(0));
				cv::fillConvexPoly(img, key_val.second, cv::Scalar(255), 8, 0);
				const auto name = "cv: " + key_val.first;
				cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
				cv::imshow(name, img);
			}
			cv::waitKey();
		}
	}

	// tests
	{
		heyho::tests::helper_tests();
		heyho::tests::line_iterator_cv_tests();
		heyho::tests::line_iterator_tests();
		heyho::tests::ring_iterator_tests();
		heyho::tests::ring_iterator_bd_tests();
		heyho::tests::poly_line_iterator_tests();
		heyho::tests::poly_line_vertical_iterator_tests();

		heyho::tests::compare_convex_poly();
	}

	// benchmark
	{
		if (false or true)
		{
			using namespace heyho::tests;
			using heyho::line_iterator_cv;
			using heyho::line_iterator;

			const std::vector<std::pair<std::string, heyho::tests::fill_convex_poly_f>> fill_functions {
				paint_function<open_cv_fill_poly_f,                    uchar>(),

				paint_function<heyho_fill_poly_cv_f<line_iterator_cv>, uchar>(),
				paint_function<heyho_fill_poly_cv_f<line_iterator>,    uchar>(),

				paint_function<heyho_fill_poly_cv2_f<line_iterator>,   uchar>(),

				paint_function<heyho_fill_poly_f<line_iterator_cv>,    uchar>(),
				paint_function<heyho_fill_poly_f<line_iterator>,       uchar>()
			};
			heyho::tests::benchmark_fill_convex_poly_functions(fill_functions,  20, 50000);
			std::cout << '\n';
			heyho::tests::benchmark_fill_convex_poly_functions(fill_functions, 400,  5000);
			std::cout << '\n';

			const std::vector<std::pair<std::string, heyho::tests::count_convex_poly_f>> count_functions {
				count_function<open_cv_fill_poly_f,                    uchar>(),

				count_function<heyho_fill_poly_cv_f<line_iterator_cv>, uchar>(),
				count_function<heyho_fill_poly_cv_f<line_iterator>,    uchar>(),

				count_function<heyho_fill_poly_cv2_f<line_iterator>,   uchar>(),

				count_function<heyho_fill_poly_f<line_iterator_cv>,    uchar>(),
				count_function<heyho_fill_poly_f<line_iterator>,       uchar>()
			};
			heyho::tests::benchmark_count_convex_poly_functions(count_functions,  20, 50000);
			std::cout << '\n';
			heyho::tests::benchmark_count_convex_poly_functions(count_functions, 400,  5000);
			std::cout << '\n';
		}

		if (false)
		{
			std::cout << "BENCHMARK\n==================\n";
			heyho::tests::foreach()(heyho::tests::benchmark{{400, 200}, 45, 500});
			std::cout << '\n';
		}
	}

	return 0;
}

#ifdef MAIN
int main() {
	return main_impl();
}
#endif

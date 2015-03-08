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



int main() {

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
				heyho::fill_convex_poly<uchar>(img, key_val.second, cv::Scalar(255), 8);
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
		heyho::tests::line_iterator_cv_tests();
		heyho::tests::line_iterator_tests();
		heyho::tests::ring_iterator_tests();
		heyho::tests::poly_line_iterator_tests();
		heyho::tests::poly_line_vertical_iterator_tests();

		heyho::tests::compare_convex_poly();
	}

	// benchmark
	{
		if (false or true)
		{
			const std::vector<heyho::tests::fill_convex_poly_f> fill_functions {
				&heyho::tests::cv_fill_confex_poly,
				static_cast<heyho::tests::fill_convex_poly_f>(&heyho::fill_convex_poly_cv<uint8_t>),
				static_cast<heyho::tests::fill_convex_poly_f>(&heyho::fill_convex_poly<uint8_t>)
			};
			heyho::tests::benchmark_fill_convex_poly_functions(fill_functions, 400, 5000);
			std::cout << '\n';
		}

		if (false or true)
		{
			std::cout << "BENCHMARK\n==================\n";
			heyho::tests::foreach()(heyho::tests::benchmark{{400, 200}, 45, 500});
			std::cout << '\n';
		}
	}

	return 0;
}

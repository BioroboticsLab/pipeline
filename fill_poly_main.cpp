/*
 * fill_poly_main.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: tobias
 */



#include <iostream>

#include "fill_convex_poly_cv.h"    // fill_convex_poly
#include "ring_iterator.h"          // ring_iterator_tests
#include "line_iterator.h"          // line_iterator_tests
#include "poly_line_iterator.h"     // poly_line_iterator_tests
#include "fill_convex_poly_tests.h" // compare_convex_poly
#include "poly_line_vertical_iterator.h"
#include <opencv2/opencv.hpp>





int main() {

	// test cv::fillConvexPoly with invalid inputs
	if (false)
	{
		// all points in clockwise order
		//                                   top-left          top-right         bottom-right      bottom-left
		const std::vector<cv::Point> top   {{10,10}, {50,50}, {90,10},          {90,90},          {10,90}         };
		const std::vector<cv::Point> right {{10,10},          {90,10}, {50,50}, {90,90},          {10,90}         };
		const std::vector<cv::Point> bottom{{10,10},          {90,10},          {90,90}, {50,50}, {10,90}         };
		const std::vector<cv::Point> left  {{10,10},          {90,10},          {90,90},          {10,90}, {50,50}};

		cv::Mat img_top(100, 100, CV_8UC1, cv::Scalar(0));
		cv::fillConvexPoly(img_top, top, cv::Scalar(255), 8, 0);
		cv::namedWindow( "top", cv::WINDOW_AUTOSIZE);
		cv::imshow( "top", img_top);

		cv::Mat img_right(100, 100, CV_8UC1, cv::Scalar(0));
		cv::fillConvexPoly(img_right, right, cv::Scalar(255), 8, 0);
		cv::namedWindow( "right", cv::WINDOW_AUTOSIZE);
		cv::imshow( "right", img_right);

		cv::Mat img_bottom(100, 100, CV_8UC1, cv::Scalar(0));
		cv::fillConvexPoly(img_bottom, bottom, cv::Scalar(255), 8, 0);
		cv::namedWindow( "bottom", cv::WINDOW_AUTOSIZE);
		cv::imshow( "bottom", img_bottom);

		cv::Mat img_left(100, 100, CV_8UC1, cv::Scalar(0));
		cv::fillConvexPoly(img_left, left, cv::Scalar(255), 8, 0);
		cv::namedWindow( "left", cv::WINDOW_AUTOSIZE);
		cv::imshow( "left", img_left);

		cv::waitKey();
	}

	// tests
	{
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

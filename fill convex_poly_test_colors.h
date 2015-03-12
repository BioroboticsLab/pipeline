/*
 * fill convex_poly_test_colors.h
 *
 *  Created on: Mar 12, 2015
 *      Author: tobias
 */

#ifndef FILL_CONVEX_POLY_TEST_COLORS_H_
#define FILL_CONVEX_POLY_TEST_COLORS_H_

#include <opencv2/opencv.hpp> // cv::Scalar, CV_ img type defines

namespace heyho {
	namespace tests {

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
	}

}



#endif /* FILL_CONVEX_POLY_TEST_COLORS_H_ */

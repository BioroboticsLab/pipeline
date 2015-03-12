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

			template<typename pixel_t>
			inline cv::Scalar pixel_2_white() {
				return white<cv::DataType<pixel_t>::type>();
			}


			template<int>
			cv::Scalar black();
			template<>
			inline cv::Scalar black<CV_8UC1>() { return {0}; }
			template<>
			inline cv::Scalar black<CV_8SC1>() { return {-128}; }
			template<>
			inline cv::Scalar black<CV_8UC3>() { return {0, 0, 0}; }
			template<>
			inline cv::Scalar black<CV_32FC1>() { return {0.0f}; }
			template<>
			inline cv::Scalar black<CV_32FC3>() { return {0.0f, 0.0f, 0.0f}; }

			template<typename pixel_t>
			inline cv::Scalar pixel_2_black() {
				return black<cv::DataType<pixel_t>::type>();
			}


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

			template<typename pixel_t>
			inline cv::Scalar pixel_2_default() {
				return default_color<cv::DataType<pixel_t>::type>();
			}
	}

}



#endif /* FILL_CONVEX_POLY_TEST_COLORS_H_ */
